//
// Hero XUC driver.
// Wire yaw: clockwise decreases (top view). Pitch: upward increases.
// RX yaw and TX yaw/rate retain the same sign used by the planner.
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
    // STM32 -> NUC: 28-byte status/attitude packet.
    uint8_t header;       // byte 0, fixed 0x5A
    uint8_t flags;        // byte 1: bit0 enemy color (0 red/1 blue),
                          // bit1 reset_tracker, bits2..6 reserved,
                          // bit7 fire_mode (1 enables the auto-aim link)
    float roll;           // bytes 2..5, degrees, IMU roll
    float pitch;          // bytes 6..9, degrees, IMU pitch
    float yaw;            // bytes 10..13, degrees, IMU yaw
    float aim_x;          // bytes 14..17, currently ignored by the NUC driver
    float aim_y;          // bytes 18..21, currently ignored by the NUC driver
    float aim_z;          // bytes 22..25, currently ignored by the NUC driver
    uint16_t checksum;    // bytes 26..27, CRC16 of bytes 0..25, low byte first
};
namespace HeroXucFlags {
constexpr uint8_t DetectColor = 1u << 0;
constexpr uint8_t ResetTracker = 1u << 1;
constexpr uint8_t FireMode = 1u << 7;
} // namespace HeroXucFlags

struct HeroXucControlPacket {
    // NUC -> STM32: 31-byte gimbal-control packet.
    uint8_t header;       // byte 0, fixed 0xA5
    float pitch;          // bytes 1..4, target pitch angle, degrees
    float yaw;            // bytes 5..8, target yaw angle, degrees
    float yaw_diff;       // bytes 9..12, unused Hero legacy slot; always 0
    float pitch_diff;     // bytes 13..16, unused Hero legacy slot; always 0
    float distance;       // bytes 17..20, target distance, metres; <=0 no target
    uint8_t fireadvice;   // byte 21: 0 stop, 1 continuous fire, 2 single shot
    uint8_t reserved[3];  // bytes 22..24, protocol padding
    float v_y;            // bytes 25..28, reserved motion term; currently 0
    uint16_t checksum;    // bytes 29..30, CRC16 of bytes 0..28, low byte first
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
    bool fire_actuation_enabled() const override;

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
