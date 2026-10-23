//
// Hero XUC driver.
//

#include "xuc_driver.hpp"
#include "xuc_conventions.hpp"
#include "hero_diagnostics.hpp"
#include "hit_diagnostics.hpp"

#include <cerrno>
#include <cmath>
#include <array>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <utility>
#include <cstdio>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>

namespace {
    float configured_bullet_speed_mps() {
        const char *text = std::getenv("AUTOAIM_BULLET_SPEED_MPS");
        if (!text) return 18.5f;
        char *end = nullptr;
        const double speed = std::strtod(text, &end);
        if (end == text || !end || *end || !std::isfinite(speed) || speed <= 0 || speed > 100) return NAN;
        return static_cast<float>(speed);
    }

    bool test_enemy_blue() {
        const char *value = std::getenv("AUTOAIM_TEST_ENEMY_BLUE");
        return value != nullptr && std::strcmp(value, "1") == 0;
    }

    bool xuc_dry_run() {
        const char *value = std::getenv("AUTOAIM_XUC_DRY_RUN");
        return value != nullptr && std::strcmp(value, "1") == 0;
    }

    bool xuc_debug() {
        const char *debug = std::getenv("AUTOAIM_XUC_DEBUG");
        const char *dry_run = std::getenv("AUTOAIM_XUC_DRY_RUN");
        return (debug != nullptr && std::strcmp(debug, "1") == 0) ||
               (dry_run != nullptr && std::strcmp(dry_run, "1") == 0);
    }

    std::string hex_dump(const uint8_t *data, size_t len) {
        std::string out;
        out.reserve(len * 3);
        char byte[4]{};
        for (size_t i = 0; i < len; ++i) {
            std::snprintf(byte, sizeof(byte), "%02X", data[i]);
            if (i != 0) out.push_back(' ');
            out.append(byte);
        }
        return out;
    }

    bool track_only() {
        const char *value = std::getenv("AUTOAIM_TRACK_ONLY");
        // 默认只跟踪，不给出开火建议；显式设置为 0 才允许开火建议。
        return value == nullptr || std::strcmp(value, "1") == 0;
    }

    bool configure_port(int fd, speed_t baud) {
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

XucDriver::XucDriver(const std::string &stm32_device, const std::string &imu_device)
    : stm32_device_(stm32_device), ignored_imu_device_(imu_device) {
    bullet_speed_mps_ = configured_bullet_speed_mps();
}

XucDriver::~XucDriver() {
    close();
}

void XucDriver::set_attitude_callback(AttitudeCallback cb) {
    attitude_cb_ = std::move(cb);
}

void XucDriver::set_robot_status_callback(RobotStatusCallback cb) {
    status_cb_ = std::move(cb);
}

uint16_t XucDriver::crc16_buffer(const uint8_t *buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= buf[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x0001) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
        }
    }
    return crc;
}

bool XucDriver::verify_crc16(const uint8_t *buf, size_t len) {
    if (buf == nullptr || len <= sizeof(uint16_t)) return false;
    const uint16_t expected = crc16_buffer(buf, len - sizeof(uint16_t));
    return static_cast<uint8_t>(expected & 0x00ff) == buf[len - 2] &&
           static_cast<uint8_t>((expected >> 8) & 0x00ff) == buf[len - 1];
}

void XucDriver::append_crc16(uint8_t *buf, size_t len) {
    if (buf == nullptr || len <= sizeof(uint16_t)) return;
    const uint16_t crc = crc16_buffer(buf, len - sizeof(uint16_t));
    buf[len - 2] = static_cast<uint8_t>(crc & 0x00ff);
    buf[len - 1] = static_cast<uint8_t>((crc >> 8) & 0x00ff);
}

bool XucDriver::open_stm32_port() {
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

ssize_t XucDriver::write_port(const uint8_t *data, size_t len) {
    if (stm32_fd_ < 0) return -1;
    return write(stm32_fd_, data, len);
}

bool XucDriver::init() {
    if (!std::isfinite(bullet_speed_mps_) || bullet_speed_mps_ <= 0) {
        LOGE_S("[HeroXuc] AUTOAIM_BULLET_SPEED_MPS must be finite and in (0,100] m/s");
        return false;
    }
    return open_stm32_port();
}

void XucDriver::start() {
    if (stm32_fd_ < 0 || running_) return;
    running_ = true;
    rx_thread_ = std::thread(&XucDriver::receive_loop, this);
    LOGM_S("[HeroXuc] receive thread started");
}

void XucDriver::close() {
    running_ = false;
    if (rx_thread_.joinable()) rx_thread_.join();
    if (stm32_fd_ >= 0) {
        ::close(stm32_fd_);
        stm32_fd_ = -1;
    }
}

bool XucDriver::fire_actuation_enabled() const {
    return fire_mode_enabled_.load() && !track_only() && !xuc_dry_run();
}

void XucDriver::log_transmission(const std::array<uint8_t, 31> &pkt, float pitch, float dist, uint8_t target_id,
                                 bool fire_mode_enabled, bool dry_run, ssize_t written) {
    float constrained_pitch, mcu_yaw;
    std::memcpy(&constrained_pitch, pkt.data() + 1, sizeof(float));
    std::memcpy(&mcu_yaw, pkt.data() + 5, sizeof(float));
    hit_diagnostics::append(fire_diagnostics_,
        {mcu_yaw,constrained_pitch,pkt[21],static_cast<int>(written),fire_mode_enabled,dry_run,track_only(),
         dry_run?"dry_run":written==31?"written":"incomplete_write"},PROJECT_DIR);
    const bool limited = constrained_pitch != pitch;
    const uint64_t key = uint64_t(target_id) | (uint64_t(pkt[21]) << 8) | (uint64_t(fire_mode_enabled) << 16) |
                         (uint64_t(limited) << 17);
    const auto report = hero_log::permit(hero_log::Stream::Tx, key);
    if (report.emit) {
        std::ostringstream debug;
        debug << "[TX" << (dry_run ? "/DRY_RUN" : "") << (fire_mode_enabled ? "/ENABLED" : "/BLOCKED")
              << "] pitch=" << std::fixed << std::setprecision(3) << constrained_pitch << " yaw=" << mcu_yaw
              << " distance=" << ((target_id != 0) ? dist : 0.0f) << " target_id=" << unsigned(target_id)
              << " fire=" << unsigned(pkt[21]) << " pitch_requested=" << pitch << " pitch_limited=" << limited
              << " written=" << written << " seq=" << report.sequence << " skipped=" << report.skipped
              << " event=" << report.event;
        const auto &d = fire_diagnostics_;
        const auto &hit = d.decision.window.hit;
        debug << " bullet_speed_mps=" << bullet_speed_mps_ << " track_only=" << track_only() << " dry_run=" << dry_run
              << " diag_valid=" << d.valid;
        if (d.valid) {
            debug << " requested=" << d.requested << " gate_requested=" << d.gate_requested
                  << " advice=" << unsigned(d.decision.advice)
                  << " gate_reason=" << fire_control::name(d.decision.reason)
                  << " window_reason=" << d.decision.window.reason << " aim_mode=" << d.aim_mode
                  << " eligible=" << d.eligible << " model_trusted=" << d.model_trusted
                  << " model_rotation_count=" << d.model_rotation_count << " tracker_model=" << d.tracker_model
                  << " model_speed_mps=" << d.model_speed_mps << " rx_age_ms=" << d.rx_age_ms
                  << " obs_fresh=" << d.observation_fresh << " obs_remaining_ms=" << d.observation_remaining_ms
                  << " shot_hold=" << d.shot_hold << " hit_available=" << hit.crossed << " hit_front=" << hit.front
                  << " hit_armor=" << hit.armor << " hit_flight_ms=" << hit.flight_s * 1000
                  << " hit_x_mm=" << hit.lateral_m * 1000 << " hit_z_mm=" << hit.vertical_m * 1000
                  << " window_half_x_mm=" << d.decision.window.half_width_m * 1000
                  << " window_half_z_mm=" << d.decision.window.half_height_m * 1000;
        }
        if (hero_log::raw_enabled()) debug << " raw=" << hex_dump(pkt.data(), pkt.size());
        hero_log::append(debug.str(), report.event);
        static thread_local hero_log::RateGate console_gate;
        const auto display = console_gate.sample(hero_log::Clock::now(), key, std::chrono::milliseconds(1000));
        if (xuc_debug() && display.emit) LOGM_S("[HeroXuc]%s", debug.str().c_str());
    }
}

void XucDriver::transmit_cmd(float yaw, float yaw_spd, float pitch, float pitch_spd, float yaw_acc, float pitch_acc,
                             float dist, uint8_t shoot, uint8_t target_id) {
    // Hand-eye collection must never command the lower controller.  This is
    // deliberately opt-in so normal runtime behaviour is unchanged.
    // Bit 7 grants the auto-aim link. Dry-run allows packet construction and
    // logging while guaranteeing that nothing is written to the UART.
    const bool fire_mode_enabled = fire_mode_enabled_.load();
    const bool dry_run = xuc_dry_run();
    if (!fire_mode_enabled && !dry_run) {
        hit_diagnostics::append(fire_diagnostics_,{NAN,NAN,0,0,false,false,track_only(),"link_disabled"},PROJECT_DIR);
        return;
    }
    if (!std::isfinite(yaw) || !std::isfinite(pitch) || !std::isfinite(dist) || dist < 0.0f) {
        hit_diagnostics::append(fire_diagnostics_,
            {NAN,NAN,0,0,fire_mode_enabled,dry_run,track_only(),"invalid_command"},PROJECT_DIR);
        LOGW_S("[HeroXuc] rejected invalid final angle/distance; no UART write");
        LOGW_F("[HeroXuc] rejected invalid final angle/distance; no UART write");
        return;
    }
    (void)yaw_spd;
    (void)pitch_spd;

    (void)yaw_acc;
    (void)pitch_acc;

    // The Hero lower controller accepts its IMU-world pitch target only in
    // the calibrated mechanical range [0, 43] degrees.  The predictor can
    // legitimately produce a world target outside that travel range; never
    // serialize such a target to the STM32.
    constexpr float HERO_PITCH_MIN_DEG = 0.0f;
    constexpr float HERO_PITCH_MAX_DEG = 43.0f;
    const float constrained_pitch = std::clamp(pitch, HERO_PITCH_MIN_DEG, HERO_PITCH_MAX_DEG);

    // The flashed STM32 firmware validates protocol-v2's explicit 31-byte
    // layout. Do not use a C++ struct here: host-side packing put v_y at 22,
    // while the MCU requires bytes 22..24 to be padding and v_y at 25.
    std::array<uint8_t, 31> pkt{};
    pkt[0] = 0xA5;
    const auto put_float = [&pkt](size_t offset, float value) {
        std::memcpy(pkt.data() + offset, &value, sizeof(value));
    };
    const float mcu_yaw = xuc_conventions::mcu_yaw_from_internal(yaw);
    const float mcu_yaw_spd = 0.0f; // Unused Hero legacy slot; final angle is above.
    put_float(1, constrained_pitch);
    put_float(5, mcu_yaw);
    put_float(9, mcu_yaw_spd);
    put_float(13, 0.0f);
    put_float(17, (target_id != 0) ? dist : 0.0f);
    // Tracking trials may actuate yaw/pitch but must never advise firing.
    pkt[21] = (track_only() || constrained_pitch != pitch || target_id == 0 || dist <= 0.0f || shoot > 2) ? 0U : shoot;
    put_float(25, 0.0f);
    append_crc16(pkt.data(), pkt.size());

    ssize_t written = 0;
    if (!dry_run && fire_mode_enabled) {
        written = write_port(pkt.data(), pkt.size());
        if (written == static_cast<ssize_t>(pkt.size()))
            tx_fps_.count();
        else {
            const auto report = hero_log::permit(hero_log::Stream::Pnp, 100);
            if (report.emit) {
                LOGW_S("[HeroXuc] send incomplete: %zd/%zu", written, pkt.size());
                hero_log::append("[TX_ERROR] UART write incomplete", true);
            }
        }
    }
    log_transmission(pkt, pitch, dist, target_id, fire_mode_enabled, dry_run, written);
}

void XucDriver::receive_loop() {
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

void XucDriver::process_rx_byte(uint8_t byte) {
    if (rx_pos_ < RX_BUF_SIZE) {
        rx_buf_[rx_pos_++] = byte;
    } else {
        memmove(rx_buf_, rx_buf_ + 1, RX_BUF_SIZE - 1);
        rx_buf_[RX_BUF_SIZE - 1] = byte;
    }

    while (rx_pos_ >= sizeof(HeroXucStatusPacket)) {
        size_t head = 0;
        while (head < rx_pos_ && rx_buf_[head] != 0x5A)
            ++head;

        if (head >= rx_pos_) {
            rx_pos_ = 0;
            return;
        }

        if (head > 0) {
            memmove(rx_buf_, rx_buf_ + head, rx_pos_ - head);
            rx_pos_ -= head;
        }

        if (rx_pos_ < sizeof(HeroXucStatusPacket)) return;

        if (verify_crc16(rx_buf_, sizeof(HeroXucStatusPacket)) && process_status_packet(rx_buf_)) {
            memmove(rx_buf_, rx_buf_ + sizeof(HeroXucStatusPacket), rx_pos_ - sizeof(HeroXucStatusPacket));
            rx_pos_ -= sizeof(HeroXucStatusPacket);
            continue;
        }

        memmove(rx_buf_, rx_buf_ + 1, rx_pos_ - 1);
        --rx_pos_;
    }
}

bool XucDriver::process_status_packet(const uint8_t *data) {
    HeroXucStatusPacket pkt{};
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.header != 0x5A) return false;
    if (!std::isfinite(pkt.yaw) || !std::isfinite(pkt.pitch) || !std::isfinite(pkt.roll)) {
        LOGW_S("[HeroXuc] rejected non-finite RX attitude");
        return false;
    }

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
        attitude_cb_(Attitude(xuc_conventions::internal_yaw_from_mcu(pkt.yaw), pkt.pitch, pkt.roll));
    }

    RobotStatus sts{};
    sts.robot_speed_mps = bullet_speed_mps_;
    sts.enemy_color = test_enemy_blue()
                          ? EnemyColor::BLUE
                          : ((pkt.flags & HeroXucFlags::DetectColor) ? EnemyColor::BLUE : EnemyColor::RED);
    if (status_cb_) {
        status_cb_(sts);
    }

    const auto report = hero_log::permit(hero_log::Stream::Rx, pkt.flags);
    if (report.emit) {
        std::ostringstream debug;
        debug << "[RX] color=" << ((pkt.flags & HeroXucFlags::DetectColor) ? 1U : 0U)
              << " reset_tracker=" << mcu_tracking << " fire_mode=" << fire_mode << std::fixed << std::setprecision(3)
              << " roll=" << pkt.roll << " pitch=" << pkt.pitch << " yaw=" << pkt.yaw << " seq=" << report.sequence
              << " skipped=" << report.skipped << " event=" << report.event;
        if (hero_log::raw_enabled()) debug << " raw=" << hex_dump(data, sizeof(pkt));
        hero_log::append(debug.str(), report.event);
        static thread_local hero_log::RateGate console_gate;
        const auto display = console_gate.sample(hero_log::Clock::now(), pkt.flags, std::chrono::milliseconds(1000));
        if (xuc_debug() && display.emit) LOGM_S("[HeroXuc]%s", debug.str().c_str());
    }
    return true;
}
