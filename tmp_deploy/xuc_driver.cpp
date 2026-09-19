//
// XUC Driver — Raw serial for TWLInfantry + optional CH010 IMU direct connect
//

#include "xuc_driver.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <cmath>

// ===================================================================
// CRC16 — 与 STM32 CRC.cpp 完全一致
// ===================================================================

static const uint16_t CRC16_TABLE[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
};

uint16_t XucDriver::crc16_update(uint8_t data, uint16_t crc)
{
    return (crc >> 8) ^ CRC16_TABLE[(crc ^ data) & 0xff];
}

uint16_t XucDriver::crc16_buffer(const uint8_t* buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc = crc16_update(buf[i], crc);
    }
    return crc;
}

// ===================================================================
// 构造 / 析构
// ===================================================================

XucDriver::XucDriver(const std::string& stm32_device, const std::string& imu_device)
    : stm32_device_(stm32_device), imu_device_(imu_device)
{
}

XucDriver::~XucDriver()
{
    close();
}

// ===================================================================
// 回调注册
// ===================================================================

void XucDriver::set_attitude_callback(AttitudeCallback cb)
{
    attitude_cb_ = std::move(cb);
}

void XucDriver::set_robot_status_callback(RobotStatusCallback cb)
{
    status_cb_ = std::move(cb);
}

// ===================================================================
// 串口底层 — STM32
// ===================================================================

static bool configure_port(int fd, speed_t baud)
{
    struct termios tty{};
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

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd, TCIOFLUSH);
    return tcsetattr(fd, TCSANOW, &tty) == 0;
}

bool XucDriver::open_stm32_port()
{
    stm32_fd_ = open(stm32_device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (stm32_fd_ < 0) {
        LOGE_S("[XucDriver] Cannot open STM32 port %s: %s", stm32_device_.c_str(), strerror(errno));
        return false;
    }

    if (!configure_port(stm32_fd_, B460800)) {
        LOGE_S("[XucDriver] STM32 port tcsetattr failed: %s", strerror(errno));
        ::close(stm32_fd_); stm32_fd_ = -1;
        return false;
    }

    // 切回阻塞模式
    int flags = fcntl(stm32_fd_, F_GETFL);
    fcntl(stm32_fd_, F_SETFL, flags & ~O_NONBLOCK);

    LOGM_S("[XucDriver] STM32 port %s opened @ 460800", stm32_device_.c_str());
    return true;
}

bool XucDriver::open_imu_port()
{
    if (imu_device_.empty()) {
        LOGW_S("[XucDriver] No IMU device specified, IMU disabled");
        return false;  // 不 fatal
    }

    imu_fd_ = open(imu_device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (imu_fd_ < 0) {
        LOGE_S("[XucDriver] Cannot open IMU port %s: %s", imu_device_.c_str(), strerror(errno));
        return false;
    }

    if (!configure_port(imu_fd_, B115200)) {
        LOGE_S("[XucDriver] IMU port tcsetattr failed: %s", strerror(errno));
        ::close(imu_fd_); imu_fd_ = -1;
        return false;
    }

    int flags = fcntl(imu_fd_, F_GETFL);
    fcntl(imu_fd_, F_SETFL, flags & ~O_NONBLOCK);

    LOGM_S("[XucDriver] IMU port %s opened @ 115200", imu_device_.c_str());
    return true;
}

ssize_t XucDriver::write_port(const uint8_t* data, size_t len)
{
    return write(stm32_fd_, data, len);
}

// ===================================================================
// 生命周期
// ===================================================================

bool XucDriver::init()
{
    if (!open_stm32_port()) return false;
    open_imu_port();  // IMU 失败不阻止启动（IMU 可选）
    return true;
}

void XucDriver::start()
{
    if (stm32_fd_ >= 0) {
        stm32_running_ = true;
        stm32_rx_thread_ = std::thread(&XucDriver::stm32_receive_loop, this);
        LOGM_S("[XucDriver] STM32 receive thread started");
    }
    if (imu_fd_ >= 0) {
        imu_running_ = true;
        imu_rx_thread_ = std::thread(&XucDriver::imu_receive_loop, this);
        LOGM_S("[XucDriver] IMU receive thread started");
    }
}

void XucDriver::close()
{
    stm32_running_ = false;
    imu_running_   = false;

    if (stm32_rx_thread_.joinable()) stm32_rx_thread_.join();
    if (imu_rx_thread_.joinable())   imu_rx_thread_.join();

    if (stm32_fd_ >= 0) { ::close(stm32_fd_); stm32_fd_ = -1; }
    if (imu_fd_   >= 0) { ::close(imu_fd_);   imu_fd_   = -1; }

    LOGM_S("[XucDriver] All ports closed");
}

// ===================================================================
// 发送 — 打包 RxPacket_TJ (NUC → STM32)
// ===================================================================

void XucDriver::transmit_cmd(float yaw, float yaw_spd, float pitch, float pitch_spd,
                             float yaw_acc, float pitch_acc, float dist,
                             uint8_t shoot, uint8_t target_id)
{
    (void)yaw_spd; (void)pitch_spd; (void)yaw_acc; (void)pitch_acc;
    (void)dist; (void)target_id;

    XucRxPacket pkt{};
    pkt.head[0]  = 'S';
    pkt.head[1]  = 'P';
    pkt.control  = 1;
    pkt.shoot    = shoot;
    pkt.yaw      = yaw;
    pkt.pitch    = pitch;
    pkt.imu_pitch = latest_imu_pitch_deg_;
    pkt.imu_yaw   = latest_imu_yaw_deg_;
    pkt.crc16    = 0;

    uint16_t crc = crc16_buffer(reinterpret_cast<const uint8_t*>(&pkt),
                                sizeof(XucRxPacket) - sizeof(uint16_t));
    pkt.crc16 = crc;

    ssize_t written = write_port(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    if (written < 0) {
        LOGE_S("[XucDriver] transmit_cmd write error: %s", strerror(errno));
    }
}

// ===================================================================
// STM32 接收线程 — 解析 TxPacket_TJ (12 或 24 字节)
// ===================================================================

void XucDriver::stm32_receive_loop()
{
    LOGM_S("[XucDriver] STM32 receive loop started");
    uint8_t byte;

    while (stm32_running_) {
        ssize_t n = read(stm32_fd_, &byte, 1);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            LOGE_S("[XucDriver] STM32 read error: %s", strerror(errno));
            break;
        }
        if (n == 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }
        process_rx_byte(byte);
    }
    LOGM_S("[XucDriver] STM32 receive loop ended");
}

void XucDriver::process_rx_byte(uint8_t byte)
{
    // 写入环形缓冲区
    if (rx_pos_ < RX_BUF_SIZE) {
        rx_buf_[rx_pos_++] = byte;
    } else {
        memmove(rx_buf_, rx_buf_ + 1, RX_BUF_SIZE - 1);
        rx_buf_[RX_BUF_SIZE - 1] = byte;
    }

    // 搜索 'S' 'P' 头
    if (rx_pos_ < 2) return;
    if (rx_buf_[rx_pos_ - 2] != 'S' || rx_buf_[rx_pos_ - 1] != 'P') {
        return;
    }

    const uint8_t* candidate = &rx_buf_[rx_pos_ - 2];
    const size_t remaining = rx_buf_ + rx_pos_ - candidate;

    // 先尝试扩展包（24 字节，含 IMU），再尝试基础包（12 字节）
    // CRC 长度不同，CRC 校验自然区分
    if (remaining >= sizeof(XucTxPacket)) {
        uint16_t crc = crc16_buffer(candidate, sizeof(XucTxPacket) - sizeof(uint16_t));
        const auto* pkt = reinterpret_cast<const XucTxPacket*>(candidate);
        if (crc == pkt->crc16) {
            stm32_fps_.count();

            // IMU 数据由直连 IMU 提供，此处忽略 STM32 发来的 IMU
            RobotStatus sts{};
            sts.robot_speed_mps = pkt->bullet_speed;
            if (pkt->robot_id > 0 && pkt->robot_id < 20) {
                sts.enemy_color = EnemyColor::BLUE;
            } else if (pkt->robot_id >= 100) {
                sts.enemy_color = EnemyColor::RED;
            }
            if (sts.robot_speed_mps < 10.0f) sts.robot_speed_mps = 10.0f;
            if (status_cb_) status_cb_(sts);
            return;
        }
    }

    if (remaining >= sizeof(XucTxPacketV1)) {
        uint16_t crc = crc16_buffer(candidate, sizeof(XucTxPacketV1) - sizeof(uint16_t));
        const auto* pkt = reinterpret_cast<const XucTxPacketV1*>(candidate);
        if (crc == pkt->crc16) {
            stm32_fps_.count();

            RobotStatus sts{};
            sts.robot_speed_mps = pkt->bullet_speed;
            if (pkt->robot_id > 0 && pkt->robot_id < 20) {
                sts.enemy_color = EnemyColor::BLUE;
            } else if (pkt->robot_id >= 100) {
                sts.enemy_color = EnemyColor::RED;
            }
            if (sts.robot_speed_mps < 10.0f) sts.robot_speed_mps = 10.0f;
            if (status_cb_) status_cb_(sts);
            return;
        }
    }
}

// ===================================================================
// IMU 接收线程 — 解析 CH010 协议 (115200 baud)
//
// CH010 串口协议（与 STM32 imu.cpp 一致）:
//   每帧 ~60 字节，yaw/pitch/roll 以 int16 × scale 格式存放在
//   数据区的[0..1]=yaw_q, [2..3]=pitch_q, [4..5]=roll_q
//
//   IMU::Decode() 解析方式:
//     y_q = rd_i16_le(&frame[0])
//     p_q = rd_i16_le(&frame[2])
//     r_q = rd_i16_le(&frame[4])
//     angle = q / 100.0   (度)
//
// CH010 帧特征:
//   帧头 0x55 0x5X, 第 3 字节 = 包类型
//   总帧长 = 11 + 数据长度 (通常 11+44=55 左右)
//   四元数/欧拉角在数据段的固定偏移位置
// ===================================================================

void XucDriver::imu_receive_loop()
{
    LOGM_S("[XucDriver] IMU receive loop started");
    uint8_t byte;
    constexpr float IMU_SCALE = 100.0f;  // CH010 量化为 0.01 度

    while (imu_running_) {
        ssize_t n = read(imu_fd_, &byte, 1);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            LOGE_S("[XucDriver] IMU read error: %s", strerror(errno));
            break;
        }
        if (n == 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }

        // 写入 IMU 缓冲区
        if (imu_buf_pos_ < IMU_BUF_SIZE) {
            imu_buf_[imu_buf_pos_++] = byte;
        } else {
            memmove(imu_buf_, imu_buf_ + 1, IMU_BUF_SIZE - 1);
            imu_buf_[IMU_BUF_SIZE - 1] = byte;
        }

        // CH010 帧: 0x55 0x5? ... 读取到足够数据后尝试解析
        // 帧格式参考 STM32 imu.cpp Decode():
        //   串口协议直接发 6 字节 int16 数据 (yaw_q, pitch_q, roll_q)
        //   这是 STM32 队友自定义的简化格式，不是 CH010 原始帧
        //
        // 也就是 STM32 的 CH010 IMU 本身走的是另一个协议，
        // 而 imu.cpp Decode() 期望的格式就是简单的 6 字节 int16 LE

        // 我们只需要 ≥6 字节就能尝试解析
        if (imu_buf_pos_ >= 6) {
            // 取最近 6 字节尝试，参考 STM32 Decode() 的 6 字节格式:
            //   int16 yaw_q (bytes 0-1), int16 pitch_q (bytes 2-3), int16 roll_q (bytes 4-5)
            const uint8_t* raw = &imu_buf_[imu_buf_pos_ - 6];

            auto rd_i16 = [](const uint8_t* p) -> int16_t {
                return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
            };

            int16_t y_q = rd_i16(&raw[0]);
            int16_t p_q = rd_i16(&raw[2]);
            int16_t r_q = rd_i16(&raw[4]);

            float yaw   = (float)y_q / IMU_SCALE;   // 度
            float pitch = (float)p_q / IMU_SCALE;
            float roll  = (float)r_q / IMU_SCALE;

            // 合理性检查：角度范围 -360~360，量化值 -36000~36000
            if (y_q > -36000 && y_q < 36000 &&
                p_q > -36000 && p_q < 36000 &&
                r_q > -36000 && r_q < 36000) {

                imu_fps_.count();

                // 应用陀螺仪轴映射（gyro 装反修正）
                // 之前标定结果: yaw=+raw[2], pitch=-raw[0], roll=-raw[1]
                // 也就是 IMU 发来的 raw 是 [yaw_q, pitch_q, roll_q] = [y, p, r]
                // 映射后: actual_yaw = +r, actual_pitch = -y, actual_roll = -p
                float actual_yaw   = +roll;
                float actual_pitch = -yaw;
                float actual_roll  = -pitch;

                latest_imu_yaw_deg_   = actual_yaw;
                latest_imu_pitch_deg_ = actual_pitch;
                latest_imu_roll_deg_  = actual_roll;

                // Attitude 构造函数参数是 度 (degrees)
                Attitude att(actual_yaw, actual_pitch, actual_roll);
                if (attitude_cb_) {
                    attitude_cb_(att);
                }
            }
        }
    }
    LOGM_S("[XucDriver] IMU receive loop ended");
}
