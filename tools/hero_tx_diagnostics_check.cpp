// In-memory RX packets and production TX logging; no serial/camera is opened.
#include "common.hpp"
#include "fire_control.hpp"
#include <atomic>
#include <array>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
// Access the decoder without opening a device or adding production test hooks.
#define private public
#include "xuc_driver.hpp"
#undef private
using namespace std::chrono_literals;
int checks=0;
void require(bool ok,const char* message) {
    ++checks;if(!ok) throw std::runtime_error(message);
}
void feed_rx(XucDriver& driver) {
    HeroXucStatusPacket packet{};
    packet.header=0x5a;packet.flags=HeroXucFlags::FireMode;packet.pitch=10;
    std::array<uint8_t,sizeof(packet)> bytes{};
    std::memcpy(bytes.data(),&packet,sizeof(packet));
    XucDriver::append_crc16(bytes.data(),bytes.size());
    for(auto byte:bytes) driver.process_rx_byte(byte);
}
std::vector<std::string> tx_rows() {
    std::ifstream file(std::getenv("AUTOAIM_XUC_LOG_FILE"));
    std::vector<std::string> rows;std::string line;
    while(std::getline(file,line))if(line.find("[TX/")!=std::string::npos)rows.push_back(line);
    return rows;
}
const std::string& last(const std::vector<std::string>& rows) {
    if(rows.empty()) throw std::runtime_error("missing production TX log");
    return rows.back();
}
void wire_check(const std::string& row,uint8_t expected) {
    const auto pos=row.find(" raw=");require(pos!=std::string::npos,"raw test packet is present");
    std::istringstream input(row.substr(pos+5));unsigned value;std::vector<uint8_t> bytes;
    while(input>>std::hex>>value)bytes.push_back(static_cast<uint8_t>(value));
    require(bytes.size()==31 && bytes[21]==expected && XucDriver::verify_crc16(bytes.data(),bytes.size()),
            "TX diagnostics preserve 31-byte fire field and CRC");
}
int main() {
    try {
        Log console;screen=&console;
        unsetenv("AUTOAIM_BULLET_SPEED_MPS");
        setenv("AUTOAIM_XUC_DRY_RUN","1",1);setenv("AUTOAIM_XUC_LOG_RAW","1",1);
        setenv("AUTOAIM_TRACK_ONLY","0",1);
        XucDriver driver("no-hardware");float received=0;
        driver.set_robot_status_callback([&](const auto& status){received=status.robot_speed_mps;});
        feed_rx(driver);
        require(received==15.0f,"default 15 m/s passes through the real CRC decoder and status callback");
        require(driver.stm32_fd_==-1,"decoder test never opens a UART");
        const auto now=fire_control::Clock::now();
        fire_control::Input in;in.now=in.rx_time=now;in.eligible=true;in.target_id=2;
        in.distance=3;in.aim_mode=4;in.requested=2;in.rx_pitch=10;
        in.desired_pitch=in.sent_pitch=10;
        auto& s=in.center_window;s.valid=true;s.state_time=now;s.bullet_speed=received;
        s.release_delay_s=.23;s.air_resistance=true;
        s.yaw_comp_rad=.03;s.pitch_comp_rad=.02; // Only pitch correction changes the window trajectory.
        const double angle=in.rx_pitch*center_fire::pi/180-s.pitch_comp_rad;
        const double flight=std::expm1(center_fire::drag_k*3)/center_fire::drag_k/(received*std::cos(angle));
        s.state={3.26,0,0,0,received*std::sin(angle)*flight-.5*center_fire::gravity*flight*flight,0,0,0,.26,0,0};
        fire_control::Gate gate;
        fire_control::Diagnostics d;d.valid=true;d.eligible=d.model_trusted=d.observation_fresh=true;
        d.requested=d.gate_requested=2;d.aim_mode=4;d.model_speed_mps=received;d.rx_age_ms=0;d.observation_remaining_ms=200;
        d.decision=gate.evaluate(in);require(d.decision.advice==2,"real gate opens the measured 15 m/s model window");
        driver.set_fire_diagnostics(d);driver.transmit_cmd(0,0,10,0,0,0,3,d.decision.advice,2);
        auto rows=tx_rows();auto row=last(rows);
        require(row.find("bullet_speed_mps=15.000")!=std::string::npos &&
                row.find("model_speed_mps=15.000")!=std::string::npos,"TX exposes configured and model speed");
        require(row.find("gate_reason=ready window_reason=ready")!=std::string::npos &&
                row.find("model_trusted=1")!=std::string::npos &&
                row.find("hit_available=1 hit_front=1 hit_armor=0")!=std::string::npos,
                "TX carries the actual evaluated window and trusted-model provenance");
        require(row.find("window_half_x_mm=57.500 window_half_z_mm=23.000")!=std::string::npos,
                "TX includes effective dimensions used to interpret signed miss errors");
        wire_check(row,2);
        setenv("AUTOAIM_TRACK_ONLY","1",1);
        driver.transmit_cmd(0,0,10,0,0,0,3,2,2);rows=tx_rows();row=last(rows);
        require(row.find("fire=0")!=std::string::npos && row.find("track_only=1")!=std::string::npos &&
                row.find("advice=2")!=std::string::npos,"TX separates a ready gate from driver track-only suppression");
        wire_check(row,0);
        setenv("AUTOAIM_TRACK_ONLY","0",1);
        fire_control::Gate miss;in.rx_pitch=11;d.decision=miss.evaluate(in);
        driver.set_fire_diagnostics(d);driver.transmit_cmd(0,0,10,0,0,0,3,d.decision.advice,4);
        rows=tx_rows();row=last(rows);
        require(row.find("gate_reason=center_window_closed window_reason=outside_rectangle")!=std::string::npos &&
                d.decision.window.hit.vertical_m>.023,"TX explains a high ballistic crossing with the same production decision");
        wire_check(row,0);
        in.eligible=false;d.eligible=d.model_trusted=false;d.decision=miss.evaluate(in);
        driver.set_fire_diagnostics(d);driver.transmit_cmd(0,0,10,0,0,0,3,0,5);
        rows=tx_rows();row=last(rows);
        require(row.find("gate_reason=invalid_target window_reason=not_evaluated")!=std::string::npos &&
                row.find("hit_available=0")!=std::string::npos && row.find("hit_z_mm=nan")!=std::string::npos,
                "an upstream refusal cannot display a previous frame's hit");
        setenv("AUTOAIM_BULLET_SPEED_MPS","19.2",1);
        XucDriver override_driver("no-hardware");
        override_driver.set_robot_status_callback([&](const auto& status){received=status.robot_speed_mps;});
        feed_rx(override_driver);require(std::abs(received-19.2f)<1e-5,"startup override reaches the production status callback");
        setenv("AUTOAIM_BULLET_SPEED_MPS","20",1);feed_rx(override_driver);
        require(std::abs(received-19.2f)<1e-5,"bullet speed is captured at startup, not changed mid-run");
        override_driver.transmit_cmd(0,0,10,0,0,0,3,0,6);rows=tx_rows();row=last(rows);
        require(row.find("bullet_speed_mps=19.200")!=std::string::npos && row.find("diag_valid=0")!=std::string::npos,
                "direct driver TX reports speed without inventing a gate decision");
        for(const char* value:{"","nan","inf","0","-1","100.1","abc","1e-100"}) {
            setenv("AUTOAIM_BULLET_SPEED_MPS",value,1);XucDriver invalid("no-hardware");
            require(!invalid.init() && invalid.stm32_fd_==-1,"invalid speed fails initialization before opening a device");
        }
        std::cout<<"PASS "<<checks<<" speed/configuration/TX/wire checks; no camera/UART opened\n";
        return 0;
    }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
