//
// UART Driver - TWLInfantry XUC Protocol
//

#ifndef TIMEDSERIAL_UART_DRIVER_H
#define TIMEDSERIAL_UART_DRIVER_H

#include "common.hpp"
#include "../serial_interface.hpp"
#include <RMCVSerial/RMCVSerial.hpp>
#include <stdint.h>
#include <string>
#include <thread>
#include <atomic>

class UartDriver : public SerialInterface 
{
private:
    AttitudeCallback attitude_cb_;
    RobotStatusCallback status_cb_;
    
    drivers::RMCVSerial m_serial;
    const std::string m_device_name;
    
    // TWL 协议原始串口
    int serial_fd_ = -1;
    std::atomic<bool> recv_running_{false};
    std::thread recv_thread_;

public:
    UartDriver(const std::string device_name);
    
    void set_attitude_callback(AttitudeCallback cb) override;
    void set_robot_status_callback(RobotStatusCallback cb) override;

    bool init() override;
    void start() override;
    void close() override;
    
    void transmit_cmd(float yaw, float yaw_spd, float pitch, float pitch_spd, 
                      float yaw_acc, float pitch_acc, float dist, 
                      uint8_t shoot = 0, uint8_t target_id = 0);

    bool is_open() { return m_serial.is_open(); }
    ~UartDriver() { close(); }

private:
    void on_receive_imu(drivers::packet_data_t *packet_ptr, drivers::packet_length_t len);
    void on_receive_sts(drivers::packet_data_t *packet_ptr, drivers::packet_length_t len);
    void recv_loop();  // TWL 协议接收线程
};

#endif // TIMEDSERIAL_UART_DRIVER_H
