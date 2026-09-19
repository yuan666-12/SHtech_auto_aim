//
// Hero XUC driver.
//
// Matches Yang-pure/hero STM32F405/xuc.cpp as of commit eaa237f:
//   STM32 -> NUC: 0x5A, 28 bytes, packed TxPacket
//   NUC   -> STM32: 0xA5, 31 bytes, byte layout consumed by XUC::Decode()
//

#ifndef TIMEDSERIAL_XUC_DRIVER_H
#define TIMEDSERIAL_XUC_DRIVER_H

#include "common.hpp"
#include "../serial_interface.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>

#pragma pack(push, 1)
struct HeroXucStatusPacket {
    uint8_t header;       // 0x5A
    uint8_t flags;        // bit0 detect_color, bit1 reset_tracker, bit7 fire_mode
    float roll;           // degrees, from imu_pantile.GetAngleRoll()
    float pitch;          // degrees, from imu_pantile.GetAnglePitch()
    float yaw;            // degrees, from imu_pantile.GetAngleYaw()
    float aim_x;
    float aim_y;
    float aim_z;
    uint16_t checksum;    // CRC16, low byte first
};

namespace HeroXucFlags {
constexpr uint8_t DetectColor = 1u << 0;
constexpr uint8_t ResetTracker = 1u << 1;
constexpr uint8_t FireMode = 1u << 7;
} // namespace HeroXucFlags

struct HeroXucControlPacket {
    uint8_t header;       // 0xA5
    float pitch;          // degrees; STM32 converts to radians
    float yaw;            // degrees; STM32 currently keeps this value as-is
    float yaw_diff;       // degrees/s or feedforward value for STM32 side
    float pitch_diff;     // degrees/s; STM32 converts to radians
    float distance;       // meters; <=0 means no target
    uint8_t fireadvice;   // bit0 only; keep 0 unless planner explicitly asks
    uint8_t reserved[3];  // pad so v_y starts at offset 25, matching xuc.cpp
    float v_y;
    uint16_t checksum;    // CRC16, low byte first
};
#pragma pack(pop)

static_assert(sizeof(HeroXucStatusPacket) == 28, "HeroXucStatusPacket size mismatch");
static_assert(sizeof(HeroXucControlPacket) == 31, "HeroXucControlPacket size mismatch");

class XucDriver : public SerialInterface
{
public:
    XucDriver(const std::string& stm32_device, const std::string& imu_device = "");
    ~XucDriver() override;

    bool init() override;
    void start() override;
    void close() override;

    void set_attitude_callback(AttitudeCallback cb) override;
    void set_robot_status_callback(RobotStatusCallback cb) override;

    void transmit_cmd(float yaw, float yaw_spd, float pitch, float pitch_spd,
                      float yaw_acc, float pitch_acc, float dist,
                      uint8_t shoot = 0, uint8_t target_id = 0) override;

private:
    bool open_stm32_port();
    ssize_t write_port(const uint8_t* data, size_t len);

    void receive_loop();
    void process_rx_byte(uint8_t byte);
    bool process_status_packet(const uint8_t* data);

    static uint16_t crc16_buffer(const uint8_t* buf, size_t len);
    static bool verify_crc16(const uint8_t* buf, size_t len);
    static void append_crc16(uint8_t* buf, size_t len);

    AttitudeCallback attitude_cb_;
    RobotStatusCallback status_cb_;

    const std::string stm32_device_;
    const std::string ignored_imu_device_;
    int stm32_fd_ = -1;

    std::thread rx_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> fire_mode_enabled_{false};

    static constexpr size_t RX_BUF_SIZE = 256;
    uint8_t rx_buf_[RX_BUF_SIZE]{};
    size_t rx_pos_ = 0;

    fps_counter rx_fps_{"hero_xuc_rx_fps"};
    fps_counter tx_fps_{"hero_xuc_tx_fps"};
};

#endif // TIMEDSERIAL_XUC_DRIVER_H
