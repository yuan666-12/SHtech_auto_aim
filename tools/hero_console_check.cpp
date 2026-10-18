// Exercise production TimedSerial logging and send cadence using an in-memory driver.
#include "common.hpp"
#include "timed_serial.hpp"
#include <atomic>
#include <iostream>
#include <thread>
using namespace std::chrono_literals;
struct Driver : SerialInterface {
    AttitudeCallback attitude;
    RobotStatusCallback status;
    std::atomic<int> sent{0};
    bool init() override { return true; }
    void start() override { attitude(Attitude(0,10,0)); }
    void close() override {}
    void set_attitude_callback(AttitudeCallback f) override { attitude=std::move(f); }
    void set_robot_status_callback(RobotStatusCallback f) override { status=std::move(f); }
    void transmit_cmd(float,float,float,float,float,float,float,uint8_t,uint8_t) override { ++sent; }
};
int main() {
    Log console; screen=&console;
    pipeline::bridge::PlannerToSerialBridge planner;
    pipeline::bridge::SensorFromSerialAttitudeBridge attitude;
    pipeline::bridge::SensorFromSerialRobotStatusBridge status;
    auto fake=std::make_unique<Driver>(); auto* driver=fake.get();
    hardware::TimedSerialConfig config; config.debug.log_text=true;
    hardware::TimedSerial task(config,std::move(fake),planner,attitude,status);
    task.start(); std::thread worker(std::ref(task));
    bool ok=false;
    try {
        pipeline::bridge::PlannerToSerialMessage msg{};
        msg.attitude=Attitude(0,10,0); msg.plan_period=2000us;
        for(auto& c:msg.command_array){ c.target_id=2; c.distance=3; }
        RobotStatus rx{}; rx.enemy_color=EnemyColor::RED;
        rx.program_mode=ProgramMode::AUTO_AIM;
        const auto before=std::chrono::steady_clock::now();
        for(int i=0;i<210;++i) {
            rx.robot_speed_mps=10.0f+float(i)*.001f;
            driver->status(rx); driver->attitude(Attitude(0,10,0));
            msg.observation_deadline=std::chrono::steady_clock::now()+100ms;
            planner.send(msg); std::this_thread::sleep_for(5ms);
        }
        const auto elapsed=std::chrono::steady_clock::now()-before;
        const auto snapshot=status.get().robotstatus;
        const int sent=driver->sent.load();
        // A color event must print immediately, without waiting for the heartbeat.
        rx.enemy_color=EnemyColor::BLUE; driver->status(rx);
        ok=sent>120 && sent<400 && elapsed<2s &&
            snapshot.enemy_color==EnemyColor::RED &&
            std::abs(snapshot.robot_speed_mps-rx.robot_speed_mps)<.0001f &&
            status.get().robotstatus.enemy_color==EnemyColor::BLUE;
        std::cout<<"CONSOLE_METRICS updates=211 packets="<<sent<<" elapsed_ms="
                 <<std::chrono::duration<double,std::milli>(elapsed).count()<<"\n";
    } catch(...) { task.terminate();worker.join();throw; }
    task.terminate(); worker.join();
    if(!ok) { std::cerr<<"FAIL console cadence or latest RX status\n";return 1; }
    std::cout<<"PASS console throttling retains per-packet status updates and fast TX\n";
}
