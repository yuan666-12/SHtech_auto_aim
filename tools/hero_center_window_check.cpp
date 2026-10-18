#include "common.hpp"
#include "fire_control.hpp"
#include "planner_submodule.hpp"
#include "timed_serial.hpp"
#include <condition_variable>
#include <iostream>
#include <thread>
using namespace std::chrono_literals;
int checks=0;
void require(bool value,const char* message) {
    ++checks;if(!value)throw std::runtime_error(message);
}
center_fire::Snapshot fixture(center_fire::Clock::time_point now,bool drag=false,double omega=0) {
    center_fire::Snapshot s;s.valid=true;s.state_time=now;s.bullet_speed=10;
    s.release_delay_s=.05;s.shot_interval_s=.2;s.air_resistance=drag;
    const double pitch=10*center_fire::pi/180;
    const double D=drag?std::expm1(center_fire::drag_k*3)/center_fire::drag_k:3;
    const double t=D/(10*std::cos(pitch));
    s.state={3.26,0,0,0,10*std::sin(pitch)*t-.5*center_fire::gravity*t*t,0,
             -omega*(s.release_delay_s+t),omega,.26,0,0};
    return s;
}
void geometry_checks() {
    using namespace center_fire;
    const auto now=Clock::now()+1s;Config c;
    auto s=fixture(now);
    auto hit=intersect(s,now,0,10,c);
    require(hit.inside && hit.armor==0,"actual ray hits near-side armor at 3 m");
    require(std::abs(hit.flight_s-3/(10*std::cos(10*pi/180)))<1e-6,"flight time matches no-drag model");
    require(!intersect(s,now,2,10,c).inside,"yaw miss cannot be authorized by a correct ideal angle");
    require(!intersect(s,now,0,11,c).inside,"pitch miss is rejected by armor height");
    require(!intersect(s,now+51ms,0,10,Config{.135,.056,.07,.005,30,false}).inside,
            "invalid rectangle tuning fails closed");
    s=fixture(now,true);
    require(intersect(s,now,0,10,c).inside,"horizontal drag and gravity match ballistic solver");
    for(double w:{-10.,-6.,-1.,1.,6.,10.}) {
        s=fixture(now,true,w);
        require(intersect(s,now,0,10,c).inside,"rotating armor at arrival is used, not current phase");
    }
    s=fixture(now,true,4);
    s.state_time-=100ms;
    const double delay=.1+s.release_delay_s;
    const double t=intersect(fixture(now,true),now,0,10,c).flight_s;
    s.state[6]=-4*(delay+t);s.state[3]=1;s.state[2]=-(delay+t);
    require(intersect(s,now,0,10,c).inside,"image age, translation, trigger latency and flight are all included");
    s=fixture(now);s.state[6]=-pi/2;s.state[0]=3.36;s.state[9]=.1;s.state[10]=.15;s.state[4]-=.15;
    hit=intersect(s,now,0,10,c);
    require(hit.inside && hit.armor==1,"alternate radius and height are respected");
    // A real off-centre interception: the near armor meets a ray ~2.8 deg
    // away from the ideal centre direction, beyond the old 0.5 deg gate.
    s=fixture(now,true,6);
    const double phase=.6,px=-.26*std::sin(phase),py=3.26-.26*std::cos(phase);
    const double ray_yaw=std::atan2(-px,py)*180/pi;
    const double travel=std::expm1(drag_k*std::hypot(px,py))/drag_k/(10*std::cos(10*pi/180));
    s.state[6]=phase-6*(s.release_delay_s+travel);
    s.state[4]=10*std::sin(10*pi/180)*travel-.5*gravity*travel*travel;
    require(ray_yaw>2 && intersect(s,now,ray_yaw,10,c).inside,
            "rotating armor can enter a real ray far from centre preaim");
    require(!intersect(s,now,0,10,c).inside,
            "centre phase window and off-centre actual window occur at different times");
    s=fixture(now);s.yaw_comp_rad=.03;s.pitch_comp_rad=.02;
    require(intersect(s,now,.03*180/pi,10+.02*180/pi,c).inside,"empirical angle compensation is reversed for actual launch direction");
    s=fixture(now);s.valid=false;
    require(!intersect(s,now,0,10,c).inside,"missing trusted model cannot open a window");
    s=fixture(now);s.state[8]=0;
    require(!intersect(s,now,0,10,c).inside,"invalid rotation radius is rejected");
    s=fixture(now);s.state[0]=NAN;
    require(!intersect(s,now,0,10,c).inside,"non-finite state is rejected");
    s=fixture(now);s.state_time=now+1ms;
    require(!intersect(s,now,0,10,c).inside,"future state timestamps are rejected");
    s=fixture(now);s.state_time=now-3s;
    require(!intersect(s,now,0,10,c).inside,"ancient image model is rejected");
    s=fixture(now);s.state[0]=-3.26;
    require(!intersect(s,now,0,10,c).inside,"armor behind the launch direction is rejected");
    Window window;
    s=fixture(now);
    require(window.evaluate(s,now,0,10,2),"first crossing emits one shot");
    require(!window.evaluate(s,now+300ms,0,10,2),"wide window does not refire after cooldown");
    auto missing=s;missing.valid=false;
    require(!window.evaluate(missing,now+350ms,0,10,2) &&
            !window.evaluate(s,now+400ms,0,10,2),"invalid model interruption cannot rearm a held crossing");
    auto outside=s;outside.state[2]=.5;
    require(!window.evaluate(outside,now+410ms,0,10,2) &&
            !window.evaluate(outside,now+445ms,0,10,2),"rearm requires sustained actual geometric miss");
    require(window.evaluate(s,now+450ms,0,10,2),"a later crossing can fire again");
    Window cooldown;
    require(cooldown.evaluate(s,now,0,10,2),"cooldown test begins with a shot");
    cooldown.evaluate(outside,now+5ms,0,10,2);cooldown.evaluate(outside,now+40ms,0,10,2);
    require(!cooldown.evaluate(s,now+45ms,0,10,2) &&
            !cooldown.evaluate(s,now+250ms,0,10,2),"cooldown-blocked entry cannot become a queued late shot");
    setenv("AUTOAIM_CENTER_MARGIN_X_M","nan",1);
    require(!Config::from_environment().valid,"invalid environment tuning disables window");
    unsetenv("AUTOAIM_CENTER_MARGIN_X_M");
    const auto before=Clock::now();
    bool repeat_ok=true;
    for(int i=0;i<100;++i) { auto v=fixture(now,true,6);repeat_ok=repeat_ok&&intersect(v,now,0,10,c).inside; }
    require(repeat_ok,"repeated numerical intersections remain valid");
    const auto average=std::chrono::duration<double,std::micro>(Clock::now()-before).count()/100;
    std::cout<<"INTERSECTION_AVERAGE_US "<<average<<'\n';
    require(average<4000,"intersection fits in the 5 ms serial cycle");
}
void gate_checks() {
    using namespace fire_control;
    const auto now=Clock::now()+1s;
    Input in;in.now=in.rx_time=now;in.target_id=2;in.distance=3;in.aim_mode=4;
    in.eligible=true;in.requested=2;in.center_window=fixture(now);
    in.rx_yaw=0;in.rx_pitch=10;in.desired_yaw=in.sent_yaw=5;
    in.desired_pitch=in.sent_pitch=12;
    Gate gate;
    require(gate.evaluate(in).advice==2,"center mode fires on actual hit even far from both ideal angles");
    in.now+=5ms;in.rx_time=in.now;
    require(gate.evaluate(in).advice==0,"center emits 2 once then zero");
    const auto fresh=[&](Input x){Gate g;return g.evaluate(x).advice;};
    in.now=in.rx_time=now;in.rx_yaw=2;
    require(fresh(in)==0,"center mode requires actual intersection");
    in.rx_yaw=0;in.center_window.valid=false;
    require(fresh(in)==0,"center requires explicit valid model metadata");
    in.center_window=fixture(now);in.rx_time=now-51ms;
    require(fresh(in)==0,"center retains latest RX freshness gate");
    in.rx_time=now;in.eligible=false;
    require(fresh(in)==0,"rejected/lost/untrusted observations remain prohibited");
    in.eligible=true;in.rx_pitch=44;
    require(fresh(in)==0,"center retains mechanical travel checks");
    in.rx_pitch=10;in.aim_mode=2;
    require(fresh(in)==0,"ordinary modes still require arrival at desired angles");
}
struct Driver:SerialInterface {
    struct Packet{center_fire::Clock::time_point time;float yaw,pitch;uint8_t fire;};
    AttitudeCallback attitude;RobotStatusCallback status;
    std::mutex mutex;std::condition_variable changed;std::vector<Packet> packets;
    bool actuate=true;
    bool fire_actuation_enabled()const override{return actuate;}
    bool init()override{return true;}void close()override{}
    void start()override{attitude(Attitude(0,10,0));}
    void set_attitude_callback(AttitudeCallback f)override{attitude=std::move(f);}
    void set_robot_status_callback(RobotStatusCallback f)override{status=std::move(f);}
    void transmit_cmd(float y,float,float p,float,float,float,float,uint8_t fire,uint8_t)override{
        std::lock_guard<std::mutex> lock(mutex);packets.push_back({center_fire::Clock::now(),y,p,fire});changed.notify_all();
    }
    int singles(){std::lock_guard<std::mutex> lock(mutex);return std::count_if(packets.begin(),packets.end(),[](auto p){return p.fire==2;});}
    bool wait_single(int count){std::unique_lock<std::mutex> lock(mutex);return changed.wait_for(lock,300ms,[&]{return std::count_if(packets.begin(),packets.end(),[](auto p){return p.fire==2;})>=count;});}
};
void serial_checks(bool actuate=true) {
    pipeline::bridge::PlannerToSerialBridge planner;
    pipeline::bridge::SensorFromSerialAttitudeBridge attitude;
    pipeline::bridge::SensorFromSerialRobotStatusBridge status;
    auto fake=std::make_unique<Driver>();auto* driver=fake.get();driver->actuate=actuate;
    hardware::TimedSerial task({},std::move(fake),planner,attitude,status);
    struct Run {hardware::TimedSerial& task;std::thread worker;
        explicit Run(hardware::TimedSerial& t):task(t),worker(std::ref(t)){task.start();}
        ~Run(){task.terminate();worker.join();}
    } run(task);
    pipeline::bridge::PlannerToSerialMessage msg{};
    msg.attitude=Attitude(0,10,0);msg.plan_period=2000us;
    for(auto& c:msg.command_array){c.target_id=2;c.distance=3;c.yaw_angle=5;c.fire_enable=2;}
    msg.aim_mode=AimedTargetType::VEHICLE_CENTER_WITH_VEHICLE_MODEL;msg.fire_control_eligible=true;
    msg.center_window=fixture(center_fire::Clock::now());
    msg.observation_deadline=center_fire::Clock::now()+200ms;planner.send(msg);
    require(driver->wait_single(1),"production TX opens window without ideal-angle arrival");
    for(int i=0;i<54;++i){
        driver->attitude(Attitude(0,10,0));
        if(i%5==0){msg.observation_deadline=center_fire::Clock::now()+200ms;planner.send(msg);}
        std::this_thread::sleep_for(5ms);
    }
    require(driver->singles()==1,"many visual updates and 200 Hz TX cannot refire one window");
    {std::lock_guard<std::mutex> lock(driver->mutex);
        auto shot=std::find_if(driver->packets.begin(),driver->packets.end(),[](auto p){return p.fire==2;});
        bool hold=shot!=driver->packets.end(),released=false;
        for(auto it=shot;it!=driver->packets.end();++it){
            if(it->time-shot->time<40ms)hold=hold&&std::abs(it->yaw)<.01&&std::abs(it->pitch-10)<.01;
            if(it->time-shot->time>70ms && it->yaw>1)released=true;
        }
        if(actuate) require(hold,"single packet and trigger delay hold the actual RX direction");
        else require(!hold,"inhibited/dry-run actuation does not freeze preaim");
        require(released,"preaim resumes after configured trigger hold");
    }
    msg.center_window.state[2]=.5;
    for(int i=0;i<9;++i){msg.observation_deadline=center_fire::Clock::now()+200ms;planner.send(msg);driver->attitude(Attitude(0,10,0));std::this_thread::sleep_for(5ms);}
    msg.center_window.state[2]=0;planner.send(msg);driver->attitude(Attitude(0,10,0));
    require(driver->wait_single(2),"new geometric crossing fires after rearm");
}
void bridge_checks(const std::string& root) {
    mathutils::CoordTransformer::Init(root+"/asset/camParam/cam_param_hero.yml",false);
    pipeline::bridge::PlannerToSerialBridge bridge;
    pipeline::bridge::PlannerToSerialMessage captured{};int messages=0;
    bridge.set_receiver([&](const auto& msg){captured=msg;++messages;});
    plan::PlannerConfig config;config.debug.show_image=false;config.debug.log_text=false;
    plan::PlannerSubModule planner(config,bridge,root+"/asset/plannerParam/planner_param_hero.yml");
    auto data=std::make_shared<ThreadDataPack>();
    data->has_fixed_target=true;data->attitude_valid=true;data->robotstatus.robot_speed_mps=10;
    data->attitude=Attitude(0,10,0);data->time=std::chrono::high_resolution_clock::now()-200ms;
    auto& target=data->target;target.predictor_state=TrackingState::TRACKING;
    target.vehicle_model_trust=true;target.observation_updated=true;
    target.last_observation_time=data->time;target.last_observation_update_time=center_fire::Clock::now();
    target.tracked_armor.tag_id=2;target.tracked_measurement<<3,0,.1,0;
    target.armor_y_state<<3,0;target.armor_z_state<<.1,0;
    target.tracked_state<<3.26,0,0,0,.1,0,0,0,.26,0,0;
    planner.process(data,nullptr);
    require(messages==1 && captured.aim_mode==AimedTargetType::VEHICLE_CENTER_WITH_VEHICLE_MODEL,
            "trusted vehicle uses center at zero rotation speed");
    require(captured.fire_control_eligible && captured.center_window.valid,"production bridge publishes valid window model");
    require(std::chrono::duration<double>(center_fire::Clock::now()-captured.center_window.state_time).count()>=.2,
            "model snapshot retains image age");
    require(std::abs(captured.center_window.release_delay_s-.23)<1e-8,"snapshot uses configured trigger delay");
    target.vehicle_model_trust=false;planner.process(data,nullptr);
    require(!captured.center_window.valid &&
            captured.command_array[0].fire_enable==0,"untrusted vehicle only tracks and cannot arm fire");
}
int main(int argc,char**argv) {
    try {
        Log console;screen=&console;
        geometry_checks();gate_checks();serial_checks();serial_checks(false);bridge_checks(argv[1]);
        std::cout<<"PASS "<<checks<<" center-window checks; no camera/UART opened\n";
        return 0;
    }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
