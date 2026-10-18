#include "Planner.hpp"
#include "Tracker.hpp"
#include "timed_serial.hpp"
#include "hero_diagnostics.hpp"
#include "attitude_history.hpp"
#include "xuc_driver.hpp"
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <thread>
using namespace std::chrono_literals;
int checks=0;
void require(bool value,const char* message) {
    ++checks;if(!value)throw std::runtime_error(message);
}
void fire_gate_checks() {
    using namespace fire_control;
    const auto t=Clock::now()+1s;
    Input in;in.now=t;in.rx_time=t;in.target_id=2;in.aim_mode=2;
    in.eligible=true;in.distance=3;in.rx_yaw=in.sent_yaw=in.desired_yaw=20;
    in.rx_pitch=in.sent_pitch=in.desired_pitch=10;in.requested=1;
    Gate settle(Config{});
    require(settle.evaluate(in).advice==0,"arrival starts with fire disabled");
    in.now=t+40ms;
    require(settle.evaluate(in).advice==0,"one old RX sample cannot prove sustained arrival");
    in.rx_time=in.now;
    require(settle.evaluate(in).advice==1,"fresh RX and 40 ms arrival allow continuous fire");
    in.now+=5ms;in.rx_time=in.now;in.rx_yaw=21;
    require(settle.evaluate(in).advice==0,"yaw dead-zone tracking error immediately stops fire");
    in.now+=5ms;in.rx_time=in.now;in.rx_yaw=20;
    require(settle.evaluate(in).advice==0,"arrival dwell restarts after a tracking error");
    Config immediate;immediate.settle_ms=0;
    const auto check=[&](Input x){Gate gate(immediate);return gate.evaluate(x);};
    in.now=t;in.rx_time=t;
    auto out=check(in);
    require(out.advice==1&&std::abs(out.yaw_limit_deg-.5)<1e-9&&std::abs(out.pitch_limit_deg-.2)<1e-9,
        "3 m range uses configured separate yaw and pitch limits");
    auto x=in;x.distance=6;out=check(x);
    require(out.yaw_limit_deg<.251&&out.pitch_limit_deg<.101,"farther slant range tightens angular gates");
    x=in;x.distance=1;require(std::abs(check(x).yaw_limit_deg-.5)<1e-9,"near range never loosens arrival gate");
    x=in;x.rx_yaw=20.6;require(check(x).advice==0,"yaw outside tolerance blocks fire");
    x=in;x.rx_pitch=10.3;require(check(x).advice==0,"pitch outside tolerance blocks fire");
    x=in;x.desired_yaw=x.sent_yaw=179.9;x.rx_yaw=-179.9;
    require(check(x).advice==1,"arrival compares yaw across plus/minus 180 degrees");
    x=in;x.sent_yaw=18;require(check(x).reason==Reason::Slewing,"slew-limited intermediate target cannot allow fire");
    x=in;x.rx_time=t-51ms;require(check(x).advice==0,"stale RX stops fire even while vision is fresh");
    x=in;x.rx_time={};require(check(x).advice==0,"missing RX never allows fire");
    x=in;x.rx_time=t+1ms;require(check(x).advice==0,"future RX timestamp cannot allow fire");
    x=in;x.rx_yaw=NAN;require(check(x).advice==0,"NaN RX cannot allow fire");
    x=in;x.desired_pitch=x.sent_pitch=x.rx_pitch=44;
    require(check(x).reason==Reason::PitchLimited,"out-of-travel pitch cannot allow fire");
    x=in;x.eligible=false;require(check(x).advice==0,"rejected/lost/reset observations cannot fire");
    x=in;x.distance=0;require(check(x).advice==0,"zero distance cannot fire");
    x=in;x.requested=3;require(check(x).advice==0,"obsolete internal code 3 cannot escape arrival gate");
    Gate pulse(immediate);x=in;x.aim_mode=2;x.requested=0;
    require(pulse.evaluate(x).advice==0,"center alignment can be maintained outside phase window");
    x.requested=2;x.single_shot_id=1;
    require(pulse.evaluate(x).advice==2,"legacy single event emits a single-shot code 2");
    int repeats=0;for(int i=0;i<20;++i){x.now+=5ms;x.rx_time=x.now;repeats+=pulse.evaluate(x).advice==2;}
    require(repeats==0,"200 Hz repeats of the same shot event cannot refire");
    x.single_shot_id=2;require(pulse.evaluate(x).advice==2,"new center event can fire again");
    x.single_shot_id=3;x.rx_yaw=22;
    require(pulse.evaluate(x).advice==0,"misaligned center event is dropped");
    x.rx_yaw=20;require(pulse.evaluate(x).advice==0,"blocked event is never queued for late firing");
    x.single_shot_id=0;require(pulse.evaluate(x).advice==0,"single shot requires an explicit event ID");
    Gate context(Config{});x=in;context.evaluate(x);x.now+=40ms;x.rx_time=x.now;
    require(context.evaluate(x).advice==1,"arrival context becomes ready");
    x.aim_mode=4;require(context.evaluate(x).advice==0,"strategy changes restart arrival dwell");
    Config bad=immediate;bad.valid=false;Gate invalid(bad);
    require(invalid.evaluate(in).advice==0,"invalid configuration fails closed");
    setenv("AUTOAIM_FIRE_YAW_TOL_DEG","nan",1);
    require(!Config::from_environment().valid,"non-finite tuning is rejected");
    unsetenv("AUTOAIM_FIRE_YAW_TOL_DEG");
}
void policy_checks() {
    using C=hero_log::Clock;
    const auto t=C::now();
    hero_log::RateGate gate;int emitted=0;
    for(int i=0;i<500;++i)emitted+=gate.sample(t+i*5ms,1,100ms).emit;
    require(emitted==25,"10 Hz sampling of 200 Hz input");
    auto event=gate.sample(t+2496ms,2,100ms);
    require(event.emit&&event.event&&event.skipped>0,"state event bypasses sampling");
    AttitudeHistory history;history.push(t,Attitude(179,0,0));history.push(t+10ms,Attitude(-179,10,0));
    const auto middle=history.sample(t+5ms);
    require(middle.valid&&std::abs(middle.attitude.yaw()-180)<1e-5,"yaw interpolation across wrap");
    require(std::abs(middle.attitude.pitch()-5)<1e-5,"pitch time interpolation");
    require(!history.sample(t-40ms).valid,"reject stale history boundary");
    require(!history.sample(t+50ms).valid,"reject stale latest sample");
    history.push(t+150ms,Attitude(10,0,0));
    require(!history.sample(t+70ms).valid,"reject RX outage interpolation");
    tracking_policy::ObservationGate observations;
    const auto q=std::chrono::high_resolution_clock::now();
    Eigen::Vector4d base; base<<1,0,.2,0;
    observations.reset(base,q);
    auto spike=base;spike(0)=.1;
    require(!observations.consider(spike,q+50ms),"isolated depth collapse rejected");
    require(observations.consider(base,q+100ms),"valid position recovers");
    auto moved=base;moved(0)=1.5;observations.reset(base,q);
    require(!observations.consider(moved,q+50ms),"new position awaits confirmation");
    require(observations.consider(moved,q+100ms),"repeated new position accepted");
    observations.reset(base,q);moved(0)=1.5;
    require(!observations.consider(moved,q+50ms),"fast motion first sample awaits confirmation");
    moved(0)=2.0;
    require(observations.consider(moved,q+100ms),"coherent fast motion accepted");
    for(int i=3;i<=10;++i) {
        moved(0)=1.0+0.5*i;
        require(observations.consider(moved,q+i*50ms),"continuous fast motion is not repeatedly rejected");
    }
}
void tracker_checks() {
    predict::Tracker tracker;
    using TP=predict::Tracker::TP;
    TP t=TP::clock::now()-2s;
    Eigen::Vector4d z;z<<1,0,.2,0;
    tracker.reset_target(z,t);
    for(int i=0;i<5;++i){t+=20ms;tracker.track(z,z,1,2,t,0);}
    require(tracker.get_target().predictor_state==TrackingState::TRACKING,"tracker acquisition");
    const auto seen=tracker.get_target().last_observation_time;
    const auto update_seen=tracker.get_target().last_observation_update_time;
    require(update_seen!=std::chrono::steady_clock::time_point{},"accepted observation has independent processing time");
    for(int i=0;i<40;++i){t+=2ms;tracker.track(z,z,0,2,t,0);}
    require(tracker.get_target().predictor_state==TrackingState::TEMP_LOST,"40 fast missed frames stay below time deadline");
    require(tracker.get_target().last_observation_time==seen,"predictions do not refresh observation timestamp");
    require(tracker.get_target().last_observation_update_time==update_seen,"missed frames do not renew acceptance deadline");
    t+=10ms;tracker.track(z,z,1,2,t,0);
    const auto& acquired=tracker.get_target();
    require(acquired.predictor_state==TrackingState::DETECTING&&acquired.filter_reset,"reacquisition resets filters");
    require(acquired.armor_x_state(1)==0&&acquired.armor_y_state(1)==0&&acquired.armor_z_state(1)==0&&acquired.yaw_state(1)==0,"reacquisition clears velocity");
    for(int i=0;i<5;++i){t+=20ms;tracker.track(z,z,1,2,t,0);}
    t+=201ms;tracker.track(z,z,0,2,t,0);
    require(tracker.get_target().predictor_state==TrackingState::IDLE,"single slow missed frame expires by elapsed time");
}

void fast_translation_checks() {
    predict::Tracker tracker;
    auto t=predict::Tracker::TP::clock::now()-2s;
    Eigen::Vector4d z;z<<1,0,.2,0;tracker.reset_target(z,t);
    for(int i=0;i<5;++i){t+=20ms;tracker.track(z,z,1,2,t,0);}
    int rejected=0;
    for(int i=1;i<=12;++i){
        t+=110ms;z(1)=i*.3;
        tracker.track(z,z,1,2,t,0);
        rejected+=tracker.get_target().measurement_rejected;
    }
    const auto& target=tracker.get_target();
    require(rejected<=1&&target.observation_updated,"continuous translation remains observed");
    require(target.predictor_state==TrackingState::TRACKING,"translation does not repeatedly reacquire");
    require(target.armor_x_state(1)>.1,"translation learns velocity instead of resetting every 20 cm");
}
void planner_checks(const std::string& root,const std::string& config) {
    mathutils::CoordTransformer::Init(root+"/asset/camParam/cam_param_hero.yml",false);
    auto& ct=mathutils::CoordTransformer::Get();
    cv::Point2f crossed[4]={{0,0},{10,10},{0,10},{10,0}};
    Eigen::Vector4d measured;float yaw=0;
    require(!ct.pnp_get_measurement(crossed,2,0,0,Attitude().R_world2imu(),yaw,measured),"reject crossed PnP corners");
    crossed[0].x=NAN;
    require(!ct.pnp_get_measurement(crossed,2,0,0,Attitude().R_world2imu(),yaw,measured),"reject NaN PnP corners");
    predict::Planner planner(config,false);
    Target target;target.predictor_state=TrackingState::DETECTING;
    target.last_observation_time=std::chrono::high_resolution_clock::now();
    target.last_observation_update_time=std::chrono::steady_clock::now();
    target.tracked_measurement<<3,0,.3,0;
    target.armor_x_measurement<<0;target.armor_y_measurement<<3;target.armor_z_measurement<<.3;
    target.armor_x_state<<0,0;target.armor_y_state<<3,0;target.armor_z_state<<.3,0;
    auto plan=planner.make_plan(target,10,0,0,Attitude().R_world2imu(),std::chrono::high_resolution_clock::now());
    require(plan.fire_enable==0,"initial detection does not request continuous fire");
    const auto observed_yaw=plan.target_yaw,observed_pitch=plan.target_pitch;
    target.predictor_state=TrackingState::TRACKING;
    plan=planner.make_plan(target,10,0,0,Attitude().R_world2imu(),std::chrono::high_resolution_clock::now());
    require(plan.aimed_target_type==AimedTargetType::ARMOR_WITH_ARMOR_MODEL&&plan.fire_enable==0,
        "untrusted single-armor fallback tracks with fire disabled under center-only policy");
    target.predictor_state=TrackingState::TEMP_LOST;target.armor_y_state(1)=100;
    plan=planner.make_plan(target,10,0,0,Attitude().R_world2imu(),std::chrono::high_resolution_clock::now());
    require(plan.aimed_target_type!=AimedTargetType::NONE&&plan.target_yaw==observed_yaw&&plan.target_pitch==observed_pitch,"TEMP_LOST holds last angles");
    require(plan.target_yaw_speed==0&&plan.target_pitch_speed==0&&plan.fire_enable==0,"lost hold disables extrapolation and fire");
    target.last_observation_update_time=std::chrono::steady_clock::now()-210ms;
    plan=planner.make_plan(target,10,0,0,Attitude().R_world2imu(),std::chrono::high_resolution_clock::now());
    require(plan.aimed_target_type==AimedTargetType::NONE&&plan.target_distance==0,"planner expires stale visual observation");
    target.predictor_state=TrackingState::DETECTING;
    target.last_observation_time=std::chrono::high_resolution_clock::now();
    target.last_observation_update_time=std::chrono::steady_clock::now();
    planner.make_plan(target,10,0,0,Attitude().R_world2imu(),std::chrono::high_resolution_clock::now());
    target.predictor_state=TrackingState::TRACKING;target.armor_y_state(1)=-100;
    plan=planner.make_plan(target,10,0,0,Attitude().R_world2imu(),std::chrono::high_resolution_clock::now());
    require(plan.prediction_rejected&&std::abs(plan.target_distance-std::hypot(3.,.3))<1e-9,"gross predicted range falls back to observation");
    target.armor_y_state(1)=0;target.yaw_state(1)=8;target.vehicle_model_trust=true;
    target.tracked_state<<3.26,0,0,0,.3,0,-8*.23,8,.26,0,0;
    std::this_thread::sleep_for(210ms);
    target.last_observation_update_time=std::chrono::steady_clock::now();
    plan=planner.make_plan(target,10,0,0,Attitude().R_world2imu(),std::chrono::high_resolution_clock::now());
    require(plan.aimed_target_type==AimedTargetType::VEHICLE_CENTER_WITH_VEHICLE_MODEL&&plan.fire_enable==0&&!plan.center_window.valid,
        "geometry-only center planning never arms a real ballistic window");
    plan=planner.make_plan(target,10,0,0,Attitude().R_world2imu(),std::chrono::high_resolution_clock::now());
    require(plan.fire_enable==0,"geometry-only center planning stays unarmed across updates");
    target.tracked_state(6)+=.2;std::this_thread::sleep_for(210ms);
    target.last_observation_update_time=std::chrono::steady_clock::now();
    plan=planner.make_plan(target,10,0,0,Attitude().R_world2imu(),std::chrono::high_resolution_clock::now());
    require(plan.fire_enable==0,"geometry-only planning cannot arm fire at any phase");
}

void pipeline_delay_checks(const std::string& config) {
    predict::Tracker tracker;
    auto image_time=predict::Tracker::TP::clock::now()-230ms;
    Eigen::Vector4d z;z<<3,0,.3,0;
    tracker.reset_target(z,image_time);
    Target target=tracker.get_target();
    require(target.last_observation_time==image_time,"capture timestamp remains unchanged for motion estimation");
    target.last_observation_update_time=std::chrono::steady_clock::now()-110ms;
    predict::Planner planner(config,false);
    planner.aim_target_init();
    auto plan=planner.make_plan(target,10,0,0,Attitude().R_world2imu(),image_time);
    require(plan.aimed_target_type!=AimedTargetType::NONE&&plan.target_distance>0,
        "230 ms capture age with 110 ms acceptance age remains a valid target");
    const auto yaw_before=plan.target_yaw,pitch_before=plan.target_pitch;
    const auto deadline=target.last_observation_update_time+200ms;
    require(deadline>std::chrono::steady_clock::now()&&deadline<std::chrono::steady_clock::now()+100ms,
        "serial deadline retains remaining acceptance budget, not a fresh 200 ms");
    image_time+=110ms;z(1)=.1;z(2)=.45;
    tracker.track(z,z,1,2,image_time,0);
    target=tracker.get_target();
    require(target.observation_updated,"moving plate updates accepted observation");
    planner.aim_target_init();
    plan=planner.make_plan(target,10,0,0,Attitude().R_world2imu(),image_time);
    require(plan.aimed_target_type!=AimedTargetType::NONE&&plan.target_distance>0,
        "moving plate retains nonzero distance after pipeline delay");
    require(std::abs(plan.target_yaw-yaw_before)>.01&&std::abs(plan.target_pitch-pitch_before)>.01,
        "moving plate changes both planned angles");
    const auto accepted=tracker.get_target().last_observation_update_time;
    auto spike=z;spike(0)=.1;image_time+=20ms;
    tracker.track(spike,spike,1,2,image_time,0);
    require(tracker.get_target().measurement_rejected&&
        tracker.get_target().last_observation_update_time==accepted,
        "rejected observations do not renew acceptance deadline");
    target.last_observation_time=predict::Tracker::TP::clock::now();
    target.last_observation_update_time=std::chrono::steady_clock::now()-210ms;
    plan=planner.make_plan(target,10,0,0,Attitude().R_world2imu(),image_time);
    require(plan.aimed_target_type==AimedTargetType::NONE&&plan.target_distance==0,
        "210 ms without accepted observations still clears target even with recent image");
    target.last_observation_update_time={};
    plan=planner.make_plan(target,10,0,0,Attitude().R_world2imu(),image_time);
    require(plan.aimed_target_type==AimedTargetType::NONE,"missing acceptance time cannot enable target");
}

struct FakeDriver:SerialInterface {
    struct Packet {std::chrono::steady_clock::time_point stamp;float yaw,pitch,distance;uint8_t id,fire;};
    AttitudeCallback attitude;RobotStatusCallback status;
    std::mutex mutex;std::condition_variable changed;std::vector<Packet> packets;
    bool init()override{return true;}void close()override{}
    void start()override{if(attitude)attitude(Attitude(20,10,0));}
    void set_attitude_callback(AttitudeCallback a)override{attitude=std::move(a);}
    void set_robot_status_callback(RobotStatusCallback s)override{status=std::move(s);}
    void transmit_cmd(float y,float,float p,float,float,float,float d,uint8_t fire,uint8_t id)override {
        std::lock_guard<std::mutex> lock(mutex);
        packets.push_back({std::chrono::steady_clock::now(),y,p,d,id,fire});changed.notify_all();
    }
    bool wait_for(float yaw,uint8_t id) {
        std::unique_lock<std::mutex> lock(mutex);
        return changed.wait_for(lock,300ms,[&]{return !packets.empty()&&packets.back().id==id&&std::abs(packets.back().yaw-yaw)<.01;});
    }
    bool wait_fire(uint8_t fire) {
        std::unique_lock<std::mutex> lock(mutex);
        return changed.wait_for(lock,300ms,[&]{return !packets.empty()&&packets.back().id==2&&packets.back().fire==fire;});
    }
    int single_count() {
        std::lock_guard<std::mutex> lock(mutex);
        int n=0;for(const auto&p:packets)n+=p.fire==2;return n;
    }
    bool wait_single_count(int expected) {
        std::unique_lock<std::mutex> lock(mutex);
        return changed.wait_for(lock,300ms,[&]{
            int n=0;for(const auto&p:packets)n+=p.fire==2;return n>=expected;
        });
    }
};
void serial_checks() {
    pipeline::bridge::PlannerToSerialBridge planner;
    pipeline::bridge::SensorFromSerialAttitudeBridge attitude;
    pipeline::bridge::SensorFromSerialRobotStatusBridge status;
    auto fake=std::make_unique<FakeDriver>();auto* driver=fake.get();
    hardware::TimedSerial task(hardware::TimedSerialConfig{},std::move(fake),planner,attitude,status);
    struct Run {
        hardware::TimedSerial& task;std::thread thread;
        explicit Run(hardware::TimedSerial& t):task(t),thread(std::ref(t)){task.start();}
        ~Run(){task.terminate();thread.join();}
    } run(task);
    pipeline::bridge::PlannerToSerialMessage msg{};
    msg.attitude=Attitude(20,10,0);msg.plan_period=2000us;
    for(auto&c:msg.command_array){c.distance=1;c.target_id=2;c.yaw_angle=10;}
    msg.observation_deadline=std::chrono::steady_clock::now()+60ms;
    planner.send(msg);
    require(driver->wait_for(30,2),"fresh final angle reaches in-memory driver");
    require(driver->wait_for(20,0),"serial expires visual deadline before 200 ms plan timeout");
    const auto stale_since=std::chrono::steady_clock::now();
    for(int i=0;i<5;++i){planner.send(msg);std::this_thread::sleep_for(5ms);}
    {
        std::lock_guard<std::mutex> lock(driver->mutex);
        bool positive=false;for(const auto&p:driver->packets)if(p.stamp>=stale_since+2ms&&p.id!=0)positive=true;
        require(!positive,"repeated stale plans cannot extend visual tracking");
    }
    for(auto&c:msg.command_array)c.yaw_angle=5;
    msg.observation_deadline=std::chrono::steady_clock::now()+100ms;planner.send(msg);
    require(driver->wait_for(25,2),"fresh observation recovers after timeout");
    msg.command_array[0].yaw_angle=NAN;planner.send(msg);
    require(driver->wait_for(20,0),"NaN command cancels target");

    for(auto&c:msg.command_array){c=RobotCommand{};c.distance=3;c.target_id=2;}
    msg.aim_mode=AimedTargetType::ARMOR_WITH_ARMOR_MODEL;
    msg.fire_control_eligible=true;msg.observation_deadline=std::chrono::steady_clock::now()+200ms;
    planner.send(msg);
    for(int i=0;i<14;++i){driver->attitude(Attitude(20,10,0));std::this_thread::sleep_for(5ms);}
    for(auto&c:msg.command_array)c.fire_enable=2;
    msg.single_shot_id=1;planner.send(msg);
    require(driver->wait_single_count(1),"serial emits code 2 after live arrival and a center request");
    require(driver->wait_fire(0),"single shot returns to zero on the next serial cycle");
    planner.send(msg);std::this_thread::sleep_for(15ms);
    require(driver->single_count()==1,"interpolation and duplicate messages cannot repeat a shot ID");
    driver->attitude(Attitude(20,10,0));msg.single_shot_id=2;planner.send(msg);
    require(driver->wait_single_count(2),"serial accepts a distinct single-shot event");
    require(driver->wait_fire(0),"second single-shot event also returns to zero");
    for(auto&c:msg.command_array)c.fire_enable=1;
    msg.single_shot_id=0;msg.aim_mode=AimedTargetType::ARMOR_WITH_ARMOR_MODEL;
    msg.observation_deadline=std::chrono::steady_clock::now()+200ms;planner.send(msg);
    for(int i=0;i<14;++i){driver->attitude(Attitude(20,10,0));std::this_thread::sleep_for(5ms);}
    require(driver->wait_fire(1),"serial continuous fire requires live arrival");
    driver->attitude(Attitude(22,10,0));
    require(driver->wait_fire(0),"serial immediately stops continuous fire when actual yaw drifts");
    for(int i=0;i<14;++i){driver->attitude(Attitude(20,10,0));std::this_thread::sleep_for(5ms);}
    require(driver->wait_fire(1),"serial resumes only after arrival dwell recovers");
    msg.observation_deadline=std::chrono::steady_clock::now()+200ms;planner.send(msg);
    std::this_thread::sleep_for(60ms);
    require(driver->wait_fire(0),"RX timeout stops continuous fire without waiting for vision timeout");
}

void logging_checks() {
    const char* path=std::getenv("AUTOAIM_XUC_LOG_FILE");
    require(path&&*path,"isolated log destination configured");
    XucDriver driver("no-hardware");
    for(int i=0;i<200;++i)driver.transmit_cmd(30,4,12,8,0,0,1,0,2);
    driver.transmit_cmd(20,0,10,0,0,0,0,0,0);
    std::ifstream input(path);std::string line;int count=0;bool raw=false,skipped=false;
    while(std::getline(input,line)) {
        if(line.find("[TX/")!=std::string::npos) {
            ++count;
            raw|=line.find(" raw=")!=std::string::npos;
            skipped|=line.find("skipped=199")!=std::string::npos;
        }
    }
    require(count>=2&&count<=5,"actual driver samples normal packets and retains target transitions");
    require(!raw,"raw bytes disabled by default");
    require(skipped||count>2,"suppressed packet count retained");
    setenv("AUTOAIM_XUC_LOG_RAW","1",1);
    driver.transmit_cmd(31,4,12,8,0,0,1,0,3);
    input.close();input.open(path);bool detailed=false;
    while(std::getline(input,line))if(line.find("[TX/")!=std::string::npos && line.find(" raw=")!=std::string::npos)detailed=true;
    require(detailed,"raw packet logging is explicit opt-in");
    setenv("AUTOAIM_TRACK_ONLY","0",1);
    driver.transmit_cmd(31,0,80,0,0,0,1,1,3);
    input.close();input.open(path);bool clamped_no_fire=false;
    while(std::getline(input,line)) {
        if(line.find("pitch_requested=80.000")==std::string::npos)continue;
        auto pos=line.find(" raw=");
        if(pos==std::string::npos)continue;
        std::istringstream hex(line.substr(pos+5));unsigned int value;std::vector<unsigned int> bytes;
        while(hex>>std::hex>>value)bytes.push_back(value);
        clamped_no_fire=bytes.size()==31&&bytes[21]==0;
    }
    require(clamped_no_fire,"mechanically clamped pitch suppresses fire advice on wire");
    for(unsigned request: {0U,1U,2U,3U}) {
        const float requested_yaw=40+request;
        driver.transmit_cmd(requested_yaw,0,10,0,0,0,3,request,2);
        input.close();input.open(path);bool matched=false;
        while(std::getline(input,line)) {
            if(line.find("yaw="+std::to_string(int(requested_yaw))+".000")==std::string::npos)continue;
            auto pos=line.find(" raw=");if(pos==std::string::npos)continue;
            std::istringstream hex(line.substr(pos+5));unsigned value;std::vector<unsigned> bytes;
            while(hex>>std::hex>>value)bytes.push_back(value);
            matched=bytes.size()==31&&bytes[21]==(request<=2?request:0);
        }
        require(matched,"driver serializes 0/1/2 explicitly and rejects obsolete code 3");
    }
    unsetenv("AUTOAIM_TRACK_ONLY");
    unsetenv("AUTOAIM_XUC_LOG_RAW");
    std::cout<<"Driver log: "<<count<<" rows for 201 packets before raw opt-in\n";
}
int main(int argc,char**argv) {
    try {
        fire_gate_checks();policy_checks();tracker_checks();fast_translation_checks();planner_checks(argv[1],argv[2]);
        pipeline_delay_checks(argv[2]);serial_checks();logging_checks();
        mathutils::CoordTransformer::Destroy();
        std::cout<<"PASS "<<checks<<" lifecycle, timing and logging checks; no camera/UART opened\n";
        return 0;
    }catch(const std::exception&e){
        std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;
    }
}
