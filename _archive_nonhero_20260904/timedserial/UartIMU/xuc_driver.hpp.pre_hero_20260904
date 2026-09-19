//
// XUC Driver - Raw serial driver for TWLInfantry protocol
// Communicates with STM32F405 via UART4 @ 460800 baud
// Optionally reads CH010 IMU directly from a second serial port
// No RMCVSerial dependency — uses raw Linux termios
//

#ifndef TIMEDSERIAL_XUC_DRIVER_H
#define TIMEDSERIAL_XUC_DRIVER_H

#include "common.hpp"
#include "../serial_interface.hpp"

#include <stdint.h>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>

/**
 * @brief XUC 协议数据包定义（与 TWLInfantry xuc_can.h 对齐）
 */

#pragma pack(push, 1)
struct XucRxPacket {
    // NUC → STM32 (对应 STM32 的 RxPacket_TJ)
    uint8_t  head[2];       // 'S' 'P'
    uint8_t  control;       // 1=控制云台
    uint8_t  shoot;         // 0=不开火, 1=开火
    float    yaw;           // 目标 yaw (rad)
    float    pitch;         // 目标 pitch (rad)
    float    imu_pitch;     // IMU pitch (rad) — NUC 回传给 STM32 参考
    float    imu_yaw;       // IMU yaw (rad)   — NUC 回传给 STM32 参考
    uint16_t crc16;         // CRC16 校验
};

struct XucTxPacket {
    // STM32 → NUC (对应 STM32 修改后的 TxPacket_TJ，含 IMU)
    uint8_t  head[2];       // 'S' 'P'
    uint8_t  mode;          // 模式
    uint8_t  robot_id;      // 机器人ID
    float    bullet_speed;  // 弹速 (m/s)
    uint16_t bullet_count;  // 子弹计数
    float    imu_yaw;       // IMU yaw (rad)   — 新增
    float    imu_pitch;     // IMU pitch (rad) — 新增
    float    imu_roll;      // IMU roll (rad)  — 新增
    uint16_t crc16;         // CRC16 校验
};
#pragma pack(pop)

static_assert(sizeof(XucRxPacket) == 22, "XucRxPacket size mismatch");
static_assert(sizeof(XucTxPacket) == 24, "XucTxPacket size mismatch");

// V1: 当前 STM32 固件 TxPacket_TJ（12 字节，无 IMU 字段）
#pragma pack(push, 1)
struct XucTxPacketV1 {
    uint8_t  head[2];       // 'S' 'P'
    uint8_t  mode;          // 模式
    uint8_t  robot_id;      // 机器人ID
    float    bullet_speed;  // 弹速 (m/s)
    uint16_t bullet_count;  // 子弹计数
    uint16_t crc16;         // CRC16 校验
};
#pragma pack(pop)
static_assert(sizeof(XucTxPacketV1) == 12, "XucTxPacketV1 size mismatch");

/**
 * @brief XucDriver — TWLInfantry 裸串口驱动 + 可选 IMU 直连
 *
 * 串口架构:
 *   STM32 口 (460800 baud): 收发 XUC 协议 (yaw/pitch 指令 + 弹速/机器人ID)
 *   IMU  口 (115200 baud):  只读 CH010 陀螺仪数据 (6 字节原始帧)
 *
 * 不修改 STM32 固件时，IMU 通过 USB 转串口直连 NUC。
 * 两个串口独立工作，互不阻塞。
 */
class XucDriver : public SerialInterface
{
public:
    /**
     * @param stm32_device  STM32 串口设备路径 (如 /dev/ttyUSB0)
     * @param imu_device    IMU 串口设备路径 (如 /dev/ttyUSB1)，空串表示不用 IMU
     */
    XucDriver(const std::string& stm32_device, const std::string& imu_device = "");
    ~XucDriver() override;

    // --- SerialInterface implementation ---
    bool init() override;
    void start() override;
    void close() override;

    void set_attitude_callback(AttitudeCallback cb) override;
    void set_robot_status_callback(RobotStatusCallback cb) override;

    void transmit_cmd(float yaw, float yaw_spd, float pitch, float pitch_spd,
                      float yaw_acc, float pitch_acc, float dist,
                      uint8_t shoot = 0, uint8_t target_id = 0) override;

private:
    // ========== STM32 串口 ==========
    void stm32_receive_loop();
    void process_rx_byte(uint8_t byte);

    static uint16_t crc16_update(uint8_t data, uint16_t crc);
    static uint16_t crc16_buffer(const uint8_t* buf, size_t len);

    bool open_stm32_port();
    ssize_t write_port(const uint8_t* data, size_t len);

    // ========== IMU 串口 (CH010) ==========
    bool open_imu_port();
    void imu_receive_loop();

    // ========== 回调 ==========
    AttitudeCallback   attitude_cb_;
    RobotStatusCallback status_cb_;

    // ========== STM32 串口状态 ==========
    const std::string stm32_device_;
    int stm32_fd_ = -1;

    std::thread  stm32_rx_thread_;
    std::atomic<bool> stm32_running_{false};

    static constexpr size_t RX_BUF_SIZE = 256;
    uint8_t  rx_buf_[RX_BUF_SIZE]{};
    size_t   rx_pos_ = 0;

    // 最新 IMU 缓存（用于回传给 STM32）
    float latest_imu_yaw_deg_   = 0.0f;
    float latest_imu_pitch_deg_ = 0.0f;
    float latest_imu_roll_deg_  = 0.0f;

    // ========== IMU 串口状态 ==========
    const std::string imu_device_;
    int imu_fd_ = -1;

    std::thread  imu_rx_thread_;
    std::atomic<bool> imu_running_{false};

    static constexpr size_t IMU_BUF_SIZE = 64;
    uint8_t  imu_buf_[IMU_BUF_SIZE]{};
    size_t   imu_buf_pos_ = 0;

    // ========== 统计 ==========
    fps_counter stm32_fps_{"xuc_stm32_fps"};
    fps_counter imu_fps_{"xuc_imu_fps"};
};

#endif // TIMEDSERIAL_XUC_DRIVER_H
