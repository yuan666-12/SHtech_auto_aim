#include "timed_serial.hpp"
#include "hit_diagnostics.hpp"
#include <atomic>
#include <fstream>
#include <iostream>
#include <thread>
using namespace std::chrono_literals;
namespace {
int checks=0;
void require(bool v,const char *message) {++checks;if(!v)throw std::runtime_error(message);}
first_shot::Input input() {
    first_shot::Input in;in.now=in.rx_time=first_shot::Clock::time_point(1s);
    in.active=true;in.target_id=2;in.yaw=0;in.pitch=10;in.planned_yaw=3;in.planned_pitch=10;
    return in;
}
first_shot::Result advance(first_shot::Gate &gate,first_shot::Input &in,int steps,double yaw_step=0,double pitch_step=0) {
    first_shot::Result result;
    for(int i=0;i<steps;++i) {
        in.now+=5ms;in.rx_time=in.now;in.yaw+=yaw_step;in.pitch+=pitch_step;result=gate.update(in);
    }
    return result;
}
void unit_checks() {
    first_shot::Gate gate;auto in=input();
    require(!gate.update(in).ready,"first sample cannot prove stable motion");
    for(int i=0;i<80;++i) {
        auto r=advance(gate,in,1,.25);
        require(!r.ready,"50 deg/s sweep cannot arm even while crossing an armor");
    }
    require(!advance(gate,in,10).ready,"recent fast motion still blocks during deceleration");
    require(advance(gate,in,11).ready,"fresh stationary RX arms despite 3 degree aim bias");
    gate.note_dispatched(in);
    require(advance(gate,in,1,0,.5).ready,"normal post-shot recoil does not repeat first-shot waiting");
    in.planned_yaw+=6;
    require(!gate.update(in).ready,"large planned turn rearms preparation");
    require(advance(gate,in,21).ready,"large-turn preparation recovers");
    gate.note_dispatched(in);in.yaw+=6;in.now+=5ms;in.rx_time=in.now;
    require(!gate.update(in).ready,"large actual reposition also rearms preparation");
    advance(gate,in,21);gate.note_dispatched(in);in.active=false;
    require(!gate.update(in).ready,"leaving auto-aim clears preparation");
    in.active=true;
    require(!gate.update(in).ready,"reentry requires new RX history");
    require(advance(gate,in,21).ready,"reentry can settle normally");
    in.target_id=3;
    require(!gate.update(in).ready,"target change cannot inherit readiness");
    require(advance(gate,in,21).ready,"new target settles");
    in.now+=60ms;
    require(!gate.update(in).ready,"stale RX resets readiness");
    in.rx_time=in.now;
    require(!gate.update(in).ready,"one fresh sample after a gap is insufficient");
    for(int i=0;i<9;++i){in.now+=5ms;require(!gate.update(in).ready,"repeated RX timestamp cannot earn dwell");}
    in.now+=5ms;in.rx_time=in.now;advance(gate,in,21);
    in.rx_time-=20ms;
    require(!gate.update(in).ready,"out-of-order RX restarts observation");
    first_shot::Gate wrap;in=input();in.yaw=179.99;wrap.update(in);
    for(int i=0;i<22;++i){in.yaw=i%2?179.99:-179.99;advance(wrap,in,1);}
    require(wrap.update(in).ready,"yaw wrap is a tiny movement, not a full turn");
    first_shot::Config invalid;invalid.valid=false;first_shot::Gate bad(invalid);
    require(!bad.update(in).ready,"invalid configuration blocks");
    first_shot::Config disabled;disabled.stable_ms=0;first_shot::Gate off(disabled);
    require(off.update(in).ready,"explicit zero disables preparation");
    first_shot::Gate oscillation;in=input();oscillation.update(in);
    for(int i=0;i<25;++i){in.pitch=i%2?9:11;advance(oscillation,in,1);}
    require(!oscillation.update(in).ready,"near-zero mean speed cannot hide wide oscillation");

    fire_control::Input fire;fire.now=fire.rx_time=in.now;fire.eligible=true;fire.target_id=2;
    fire.aim_mode=4;fire.distance=3;fire.requested=2;fire.rx_pitch=fire.sent_pitch=fire.desired_pitch=10;
    auto &s=fire.center_window;s.valid=true;s.state_time=fire.now;s.bullet_speed=18.5;s.shot_interval_s=.5;
    const double p=10*center_fire::pi/180;
    const double t=std::expm1(center_fire::drag_k*3)/center_fire::drag_k/(18.5*std::cos(p));
    s.state={3.26,0,0,0,18.5*std::sin(p)*t-.5*center_fire::gravity*t*t,0,0,0,.26,0,0};
    fire_control::Gate window;
    fire.first_shot_ready=false;
    const auto blocked=window.evaluate(fire);
    require(blocked.advice==0 && blocked.reason==fire_control::Reason::FirstShotUnstable,
            "first-shot motion block is diagnosed before consuming the hit window");
    fire.first_shot_ready=true;
    require(window.evaluate(fire).advice==2,"waiting did not consume the available geometric window");
    require(window.evaluate(fire).advice==0,"follow-up still consumes each window once");
    fire.first_shot_ready=false;window.evaluate(fire);fire.first_shot_ready=true;
    require(window.evaluate(fire).advice==0,"rearming preparation cannot erase 0.5 s cooldown");

    fire_control::Diagnostics d;d.input=fire;
    const auto encoded=hit_diagnostics::encode(d,{},1);
    const auto names=hit_diagnostics::split(hit_diagnostics::legacy_columns);
    auto values=hit_diagnostics::split(encoded);values.resize(names.size());
    std::string legacy;for(const auto &v:values){if(!legacy.empty())legacy+=',';legacy+=v;}
    require(hit_diagnostics::Row(legacy).input().first_shot_ready,"v1 diagnostics retain original no-preparation replay");
    d.input.first_shot_ready=false;
    require(!hit_diagnostics::Row(hit_diagnostics::encode(d,{},1)).input().first_shot_ready,
            "v2 records the exact preparation decision");
}
void trace_check(const char *path) {
    std::ifstream file(path);require(bool(file),"recorded trace opens");
    std::string line;std::getline(file,line);first_shot::Gate gate;bool first=false,later=false;
    while(std::getline(file,line)) {
        const auto v=hit_diagnostics::split(line);require(v.size()==9,"complete trace sample");
        first_shot::Input in;const auto base=first_shot::Clock::time_point(1s);
        in.now=base+std::chrono::nanoseconds(std::stoll(v[0]));in.rx_time=base+std::chrono::nanoseconds(std::stoll(v[1]));
        in.yaw=std::stod(v[2]);in.pitch=std::stod(v[3]);in.planned_yaw=std::stod(v[4]);in.planned_pitch=std::stod(v[5]);
        in.active=std::stoi(v[6]);in.target_id=std::stoi(v[7]);
        const auto result=gate.update(in);
        if(v[8]=="2" && !first) {
            require(!result.ready && std::abs(result.yaw_rate_dps)>50,"08:22:05.378 first shot is blocked on recorded fast motion");
            first=true;
        } else if(first && result.ready)later=true;
    }
    require(first && later,"recorded trajectory blocks early shot and later regains readiness");
    std::cout<<"TRACE blocked_original_first=1 later_ready=1; recorded feedback only, no closed-loop claim\n";
}
struct Driver:SerialInterface {
    AttitudeCallback attitude;RobotStatusCallback status;
    std::mutex mutex;std::vector<fire_control::Diagnostics> packets;
    fire_control::Diagnostics diagnostic;
    bool init()override{return true;}void close()override{}void start()override{attitude(Attitude(0,10,0));}
    bool fire_actuation_enabled()const override{return true;}
    void set_attitude_callback(AttitudeCallback cb)override{attitude=std::move(cb);}
    void set_robot_status_callback(RobotStatusCallback cb)override{status=std::move(cb);}
    void set_fire_diagnostics(const fire_control::Diagnostics &d)override{diagnostic=d;}
    void transmit_cmd(float,float,float,float,float,float,float,uint8_t fire,uint8_t)override {
        std::lock_guard<std::mutex> lock(mutex);if(fire==2)packets.push_back(diagnostic);
    }
    size_t count(){std::lock_guard<std::mutex> lock(mutex);return packets.size();}
};
void tx_check() {
    pipeline::bridge::PlannerToSerialBridge plans;
    pipeline::bridge::SensorFromSerialAttitudeBridge attitudes;
    pipeline::bridge::SensorFromSerialRobotStatusBridge status;
    auto driver=std::make_unique<Driver>();auto *fake=driver.get();
    hardware::TimedSerial serial({},std::move(driver),plans,attitudes,status);
    struct Run {hardware::TimedSerial &s;std::thread t;
        Run(hardware::TimedSerial &v):s(v),t(std::ref(v)){s.start();}~Run(){s.terminate();t.join();}} run(serial);
    pipeline::bridge::PlannerToSerialMessage msg{};
    msg.attitude=Attitude(0,10,0);msg.plan_period=2000us;
    for(auto &c:msg.command_array){c.target_id=2;c.distance=3;c.yaw_angle=3;c.fire_enable=2;}
    msg.aim_mode=AimedTargetType::VEHICLE_CENTER_WITH_VEHICLE_MODEL;
    msg.fire_control_eligible=true;msg.vehicle_model_trusted=true;
    auto &s=msg.center_window;s.valid=true;s.bullet_speed=18.5;s.release_delay_s=.23;s.shot_interval_s=.5;
    const double p=10*center_fire::pi/180;
    const double t=std::expm1(center_fire::drag_k*3)/center_fire::drag_k/(18.5*std::cos(p));
    s.state={3.26,0,0,0,18.5*std::sin(p)*t-.5*center_fire::gravity*t*t,0,0,0,.26,0,0};
    const auto publish=[&] {
        s.state_time=center_fire::Clock::now();msg.observation_deadline=s.state_time+200ms;plans.send(msg);
    };
    for(int i=0;i<=40;++i) {
        fake->attitude(Attitude(-10+i*.25,10,0));publish();std::this_thread::sleep_for(5ms);
        require(fake->count()==0,"real TX cannot fire while sweeping through the valid geometric window");
    }
    for(int i=0;i<40;++i){fake->attitude(Attitude(0,10,0));publish();std::this_thread::sleep_for(5ms);}
    require(fake->count()==1,"real TX emits one shot after stable RX, with persistent aim offset");
    std::lock_guard<std::mutex> lock(fake->mutex);
    require(fake->packets[0].input.first_shot_ready && fake->packets[0].first_shot.span_ms>=100 &&
            fake->packets[0].decision.window.hit.inside,"shot carries actual stable history and geometric hit diagnostics");
}
}
int main(int argc,char **argv) {
    try {
        Log console;screen=&console;
        if(argc>1 && std::string(argv[1])=="--tx")tx_check();
        else if(argc>1)trace_check(argv[1]);
        else unit_checks();
        std::cout<<"PASS "<<checks<<" first-shot checks; no camera/UART opened\n";return 0;
    }catch(const std::exception &e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
