//
// UART Driver Implementation - TWLInfantry XUC Protocol
//

#include "uart_driver.hpp"
#include "packet.hpp"
#include <functional>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

using namespace std::chrono;

// ==================== CRC16 Implementation (from TWLInfantry) ====================

static const uint16_t CRC_INIT = 0xffff;
static const uint16_t wCRC_Table[256] = {
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
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

static uint16_t GetCRC16CheckSum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL) return 0xFFFF;
    while (dwLength--) {
        chData = *pchMessage++;
        wCRC = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}

static uint8_t VerifyCRC16CheckSum(uint8_t* pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2)) return 0;
    wExpected = GetCRC16CheckSum(pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

static void AppendCRC16CheckSum(uint8_t* pchMessage, uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == NULL) || (dwLength <= 2)) return;
    wCRC = GetCRC16CheckSum((uint8_t*)pchMessage, dwLength - 2, CRC_INIT);
    pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
}

// ==================== UartDriver Implementation ====================

constexpr float MIN_BULLET_SPEED_MPS = 10.0f;

UartDriver::UartDriver(const std::string device_name)
    : m_device_name(device_name), m_serial(), serial_fd_(-1)
{
    // 不再注册上科大的协议处理器（TWL 协议不同）
}

bool UartDriver::init()
{
    LOGM_S("Opening %s ", m_device_name.c_str());

    // TWLInfantry 波特率 460800
    m_serial.open(m_device_name, 460800);

    // 同时用 Linux 原始串口打开（用于 TWL 协议收发）
    serial_fd_ = open(m_device_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
        LOGE_S("Failed to open raw serial fd for %s", m_device_name.c_str());
        return m_serial.is_open();  // 至少让 m_serial 能用
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd_, &tty) != 0) {
        LOGE_S("tcgetattr failed");
        ::close(serial_fd_);
        serial_fd_ = -1;
        return m_serial.is_open();
    }

    cfsetospeed(&tty, B460800);
    cfsetispeed(&tty, B460800);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        LOGE_S("tcsetattr failed");
        ::close(serial_fd_);
        serial_fd_ = -1;
    }

    return m_serial.is_open();
}

void UartDriver::start()
{
    m_serial.start_async_receive();
    recv_running_ = true;
    recv_thread_ = std::thread(&UartDriver::recv_loop, this);
}

void UartDriver::close()
{
    recv_running_ = false;
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
    m_serial.stop_async_receive();
    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }
}

void UartDriver::set_attitude_callback(AttitudeCallback cb)
{
    attitude_cb_ = cb;
}

void UartDriver::set_robot_status_callback(RobotStatusCallback cb)
{
    status_cb_ = cb;
}

void UartDriver::on_receive_imu(drivers::packet_data_t* packet_ptr, drivers::packet_length_t len)
{
    // 保留但不再使用（上科大协议已废弃）
}

void UartDriver::on_receive_sts(drivers::packet_data_t* packet_ptr, drivers::packet_length_t len)
{
    // 保留但不再使用（上科大协议已废弃）
}

void UartDriver::transmit_cmd(float yaw, float yaw_spd, float pitch, float pitch_spd,
                              float yaw_acc, float pitch_acc, float dist,
                              uint8_t shoot, uint8_t target_id)
{
    twl_tx_packet_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.head[0] = 'S';
    tx.head[1] = 'P';
    tx.control_TJ = (target_id != 0) ? 1 : 0;
    tx.shoot_TJ = shoot;
    tx.yaw_TJ = yaw;
    tx.pitch_TJ = pitch;
    tx.imu_pitch_TJ = 0.0f;
    tx.imu_yaw_TJ = 0.0f;

    AppendCRC16CheckSum((uint8_t*)&tx, sizeof(twl_tx_packet_t));

    // 通过原始 fd 发送（绕过 m_serial 的协议层）
    if (serial_fd_ >= 0) {
        ssize_t n = write(serial_fd_, &tx, sizeof(twl_tx_packet_t));
        if (n != sizeof(twl_tx_packet_t)) {
            LOGW_S("[UART] send incomplete: %zd/%zu", n, sizeof(twl_tx_packet_t));
        }
    }
}

void UartDriver::recv_loop()
{
    uint8_t buf[256];
    uint8_t frame[sizeof(twl_rx_packet_t)];
    size_t frame_idx = 0;

    while (recv_running_) {
        if (serial_fd_ < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        ssize_t n = read(serial_fd_, buf, sizeof(buf));
        if (n <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        for (ssize_t i = 0; i < n; ++i) {
            uint8_t byte = buf[i];

            if (frame_idx == 0 && byte != 'S') continue;
            if (frame_idx == 1 && byte != 'P') { frame_idx = 0; continue; }

            frame[frame_idx++] = byte;

            if (frame_idx >= sizeof(twl_rx_packet_t)) {
                if (VerifyCRC16CheckSum(frame, sizeof(twl_rx_packet_t))) {
                    twl_rx_packet_t rx;
                    memcpy(&rx, frame, sizeof(twl_rx_packet_t));

                    // 构造 RobotStatus
                    RobotStatus temp_status;
                    temp_status.robot_speed_mps = rx.bullet_speed_TJ;
                    if (temp_status.robot_speed_mps < MIN_BULLET_SPEED_MPS) {
                        temp_status.robot_speed_mps = MIN_BULLET_SPEED_MPS;
                    }

                    // 根据 robot_id 判断敌方颜色
                    if (rx.robot_id == 3) {
                        temp_status.enemy_color = EnemyColor::RED;  // 红方，敌方是蓝
                    } else if (rx.robot_id == 103) {
                        temp_status.enemy_color = EnemyColor::BLUE; // 蓝方，敌方是红
                    } else {
                        temp_status.enemy_color = EnemyColor::GRAY;
                    }

                    if (status_cb_) {
                        status_cb_(temp_status);
                    }
                }
                frame_idx = 0;
            }
        }
    }
}
