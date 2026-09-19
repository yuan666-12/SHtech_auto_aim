//
// Hero XUC driver.
//

#include "xuc_driver.hpp"

#include <cerrno>
#include <array>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace {
constexpr float MIN_BULLET_SPEED_MPS = 10.0f;

bool calibration_rx_only()
{
    const char* value = std::getenv("AUTOAIM_CALIBRATION_RX_ONLY");
    return value != nullptr && std::strcmp(value, "1") == 0;
}

bool test_enemy_blue()
{
    const char* value = std::getenv("AUTOAIM_TEST_ENEMY_BLUE");
    return value != nullptr && std::strcmp(value, "1") == 0;
}

bool track_only()
{
    const char* value = std::getenv("AUTOAIM_TRACK_ONLY");
    return value != nullptr && std::strcmp(value, "1") == 0;
}

bool configure_port(int fd, speed_t baud)
{
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) return false;

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    tcflush(fd, TCIOFLUSH);
    return tcsetattr(fd, TCSANOW, &tty) == 0;
}
} // namespace

XucDriver::XucDriver(const std::string& stm32_device, const std::string& imu_device)
    : stm32_device_(stm32_device), ignored_imu_device_(imu_device)
{
}

XucDriver::~XucDriver()
{
    close();
}

void XucDriver::set_attitude_callback(AttitudeCallback cb)
{
    attitude_cb_ = std::move(cb);
}

void XucDriver::set_robot_status_callback(RobotStatusCallback cb)
{
    status_cb_ = std::move(cb);
}

uint16_t XucDriver::crc16_buffer(const uint8_t* buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= buf[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x0001) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
        }
    }
    return crc;
}

bool XucDriver::verify_crc16(const uint8_t* buf, size_t len)
{
    if (buf == nullptr || len <= sizeof(uint16_t)) return false;
    const uint16_t expected = crc16_buffer(buf, len - sizeof(uint16_t));
    return static_cast<uint8_t>(expected & 0x00ff) == buf[len - 2] &&
           static_cast<uint8_t>((expected >> 8) & 0x00ff) == buf[len - 1];
}

void XucDriver::append_crc16(uint8_t* buf, size_t len)
{
    if (buf == nullptr || len <= sizeof(uint16_t)) return;
    const uint16_t crc = crc16_buffer(buf, len - sizeof(uint16_t));
    buf[len - 2] = static_cast<uint8_t>(crc & 0x00ff);
    buf[len - 1] = static_cast<uint8_t>((crc >> 8) & 0x00ff);
}

bool XucDriver::open_stm32_port()
{
    stm32_fd_ = open(stm32_device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (stm32_fd_ < 0) {
        LOGE_S("[HeroXuc] cannot open STM32 port %s: %s", stm32_device_.c_str(), strerror(errno));
        return false;
    }

    if (!configure_port(stm32_fd_, B460800)) {
        LOGE_S("[HeroXuc] tcsetattr failed for %s: %s", stm32_device_.c_str(), strerror(errno));
        ::close(stm32_fd_);
        stm32_fd_ = -1;
        return false;
    }

    LOGM_S("[HeroXuc] STM32 port %s opened @ 460800", stm32_device_.c_str());
    if (!ignored_imu_device_.empty()) {
        LOGW_S("[HeroXuc] ignoring second IMU port %s; Hero 0x5A status frame carries IMU attitude",
               ignored_imu_device_.c_str());
    }
    return true;
}

ssize_t XucDriver::write_port(const uint8_t* data, size_t len)
{
    if (stm32_fd_ < 0) return -1;
    return write(stm32_fd_, data, len);
}

bool XucDriver::init()
{
    return open_stm32_port();
}

void XucDriver::start()
{
    if (stm32_fd_ < 0 || running_) return;
    running_ = true;
    rx_thread_ = std::thread(&XucDriver::receive_loop, this);
    LOGM_S("[HeroXuc] receive thread started");
}

void XucDriver::close()
{
    running_ = false;
    if (rx_thread_.joinable()) rx_thread_.join();
    if (stm32_fd_ >= 0) {
        ::close(stm32_fd_);
        stm32_fd_ = -1;
    }
}

void XucDriver::transmit_cmd(float yaw, float yaw_spd, float pitch, float pitch_spd,
                             float yaw_acc, float pitch_acc, float dist,
                             uint8_t shoot, uint8_t target_id)
{
    // Hand-eye collection must never command the lower controller.  This is
    // deliberately opt-in so normal runtime behaviour is unchanged.
    if (calibration_rx_only() || !fire_mode_enabled_.load()) return;

    (void)yaw_acc;
    (void)pitch_acc;

    // The Hero lower controller accepts its IMU-world pitch target only in
    // the calibrated mechanical range [0, 43] degrees.  The predictor can
    // legitimately produce a world target outside that travel range; never
    // serialize such a target to the STM32.
    constexpr float HERO_PITCH_MIN_DEG = 0.0f;
    constexpr float HERO_PITCH_MAX_DEG = 43.0f;
    const float constrained_pitch = std::clamp(
        pitch, HERO_PITCH_MIN_DEG, HERO_PITCH_MAX_DEG);
    if (constrained_pitch != pitch) {
        LOGW_S("[HeroXuc] pitch target clamped: %.2f -> %.2f deg", pitch,
               constrained_pitch);
    }

    // The flashed STM32 firmware validates protocol-v2's explicit 31-byte
    // layout. Do not use a C++ struct here: host-side packing put v_y at 22,
    // while the MCU requires bytes 22..24 to be padding and v_y at 25.
    std::array<uint8_t, 31> pkt{};
    pkt[0] = 0xA5;
    const auto put_float = [&pkt](size_t offset, float value) {
        std::memcpy(pkt.data() + offset, &value, sizeof(value));
    };
    put_float(1, constrained_pitch);
    put_float(5, yaw);
    put_float(9, yaw_spd);
    put_float(13, pitch_spd);
    put_float(17, (target_id != 0) ? dist : 0.0f);
    // Tracking trials may actuate yaw/pitch but must never advise firing.
    pkt[21] = track_only() ? 0U : static_cast<uint8_t>(shoot & 0x01);
    put_float(25, 0.0f);
    append_crc16(pkt.data(), pkt.size());

    const ssize_t written = write_port(pkt.data(), pkt.size());
    if (written == static_cast<ssize_t>(pkt.size())) {
        tx_fps_.count();
    } else {
        LOGW_S("[HeroXuc] send incomplete: %zd/%zu", written, pkt.size());
    }
}

void XucDriver::receive_loop()
{
    uint8_t byte = 0;
    while (running_) {
        const ssize_t n = read(stm32_fd_, &byte, 1);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                usleep(1000);
                continue;
            }
            LOGE_S("[HeroXuc] read error: %s", strerror(errno));
            break;
        }
        if (n == 0) {
            usleep(1000);
            continue;
        }
        process_rx_byte(byte);
    }
    LOGM_S("[HeroXuc] receive thread ended");
}

void XucDriver::process_rx_byte(uint8_t byte)
{
    if (rx_pos_ < RX_BUF_SIZE) {
        rx_buf_[rx_pos_++] = byte;
    } else {
        memmove(rx_buf_, rx_buf_ + 1, RX_BUF_SIZE - 1);
        rx_buf_[RX_BUF_SIZE - 1] = byte;
    }

    while (rx_pos_ >= sizeof(HeroXucStatusPacket)) {
        size_t head = 0;
        while (head < rx_pos_ && rx_buf_[head] != 0x5A) ++head;

        if (head >= rx_pos_) {
            rx_pos_ = 0;
            return;
        }

        if (head > 0) {
            memmove(rx_buf_, rx_buf_ + head, rx_pos_ - head);
            rx_pos_ -= head;
        }

        if (rx_pos_ < sizeof(HeroXucStatusPacket)) return;

        if (verify_crc16(rx_buf_, sizeof(HeroXucStatusPacket)) &&
            process_status_packet(rx_buf_)) {
            memmove(rx_buf_, rx_buf_ + sizeof(HeroXucStatusPacket),
                    rx_pos_ - sizeof(HeroXucStatusPacket));
            rx_pos_ -= sizeof(HeroXucStatusPacket);
            continue;
        }

        memmove(rx_buf_, rx_buf_ + 1, rx_pos_ - 1);
        --rx_pos_;
    }
}

bool XucDriver::process_status_packet(const uint8_t* data)
{
    HeroXucStatusPacket pkt{};
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.header != 0x5A) return false;

    // STM32 mirrors its CRC-validated tracking gate in status bit 1.  This
    // diagnostic bit has no control effect and is available only in logs.
    static int last_mcu_tracking = -1;
    const int mcu_tracking = (pkt.flags & HeroXucFlags::ResetTracker) ? 1 : 0;
    const bool fire_mode = (pkt.flags & HeroXucFlags::FireMode) != 0;
    const bool previous_fire_mode = fire_mode_enabled_.exchange(fire_mode);
    if (fire_mode != previous_fire_mode) {
        LOGM_S("[HeroXuc] fire_mode: %s", fire_mode ? "ENABLED" : "DISABLED");
    }
    if (mcu_tracking != last_mcu_tracking) {
        LOGM_S("[HeroXuc] MCU track gate: %s", mcu_tracking ? "ACTIVE" : "inactive");
        last_mcu_tracking = mcu_tracking;
    }

    rx_fps_.count();

    if (attitude_cb_) {
        attitude_cb_(Attitude(pkt.yaw, pkt.pitch, pkt.roll));
    }

    RobotStatus sts{};
    sts.robot_speed_mps = MIN_BULLET_SPEED_MPS;
    sts.enemy_color = test_enemy_blue()
        ? EnemyColor::BLUE
        : ((pkt.flags & HeroXucFlags::DetectColor) ? EnemyColor::BLUE : EnemyColor::RED);
    if (status_cb_) {
        status_cb_(sts);
    }

    return true;
}
