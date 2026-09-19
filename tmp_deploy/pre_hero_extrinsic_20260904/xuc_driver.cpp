//
// Hero XUC driver.
//

#include "xuc_driver.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace {
constexpr float MIN_BULLET_SPEED_MPS = 10.0f;

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
    (void)yaw_acc;
    (void)pitch_acc;

    HeroXucControlPacket pkt{};
    pkt.header = 0xA5;
    pkt.pitch = pitch;
    pkt.yaw = yaw;
    pkt.yaw_diff = yaw_spd;
    pkt.pitch_diff = pitch_spd;
    pkt.distance = (target_id != 0) ? dist : 0.0f;
    pkt.fireadvice = static_cast<uint8_t>(shoot & 0x01);
    pkt.v_y = 0.0f;
    append_crc16(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));

    const ssize_t written = write_port(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    if (written == static_cast<ssize_t>(sizeof(pkt))) {
        tx_fps_.count();
    } else {
        LOGW_S("[HeroXuc] send incomplete: %zd/%zu", written, sizeof(pkt));
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

    rx_fps_.count();

    if (attitude_cb_) {
        attitude_cb_(Attitude(pkt.yaw, pkt.pitch, pkt.roll));
    }

    RobotStatus sts{};
    sts.robot_speed_mps = MIN_BULLET_SPEED_MPS;
    sts.enemy_color = (pkt.flags & 0x01) ? EnemyColor::BLUE : EnemyColor::RED;
    if (status_cb_) {
        status_cb_(sts);
    }

    return true;
}
