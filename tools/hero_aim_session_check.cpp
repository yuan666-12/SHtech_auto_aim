#include "common.hpp"
#include "CoordTransformer.hpp"
#include "Kalman.hpp"
#include "IESEKF_Double_Armor.hpp"
#include <condition_variable>
#include <future>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#define private public
#include "MultiPolicyPredictor_submodule.hpp"
#include "planner_submodule.hpp"
#include "timed_serial.hpp"
#include "xuc_driver.hpp"
#undef private
using namespace std::chrono_literals;
namespace {
int checks=0;
void require(bool ok,const char *text){++checks;if(!ok)throw std::runtime_error(text);}
using Clock=aim_session::Clock;
using Action=aim_session::Action;
void source_checks(){
    aim_session::Source source(true);
    auto t=Clock::now();auto off=source.observe(false,t);
    require(off.managed && !off.enabled && off.generation==0,"background starts without a shooting session");
    auto first=source.observe(true,t+1ms);
    require(first.generation==1 && first.started_at==t+1ms,"rising edge starts one session");
    require(source.observe(true,t+2ms).generation==1,"repeated enabled packets do not reset");
    require(!aim_session::accepts(first,{off,t+2ms}),"pre-entry work is rejected");
    require(!aim_session::accepts(first,{first,t}),"old image relabeled after entry is rejected");
    require(aim_session::accepts(first,{first,t+1ms}),"post-entry image is accepted");
    source.observe(false,t+3ms);auto second=source.observe(true,t+4ms);
    require(second.generation==2,"off-on between visual frames creates a distinct session");
    require(!aim_session::accepts(second,{first,t+5ms}),"late first-session result is rejected");
    require(!aim_session::accepts(source.observe(false,t+6ms),{second,t+5ms}),"disabled session does not accept commands");
    aim_session::StageGuard stage;
    require(stage.inspect(off,t)==Action::Continue,"background vision keeps running");
    require(stage.inspect(first,t)==Action::Reject,"stage rejects pre-entry acquisition");
    require(stage.inspect(first,t+2ms)==Action::Reset,"first fresh frame resets once");
    require(stage.inspect(first,t+3ms)==Action::Continue,"same session keeps model history");
    require(stage.inspect(off,t+4ms)==Action::Reject,"older generation cannot undo reset");
    require(stage.inspect(second,t+5ms)==Action::Reset,"new generation resets again");
    require(stage.inspect(second,t+4ms)==Action::Reject,"out-of-order frames cannot rewind tracking");
    aim_session::Source legacy(false);
    require(aim_session::accepts(legacy.observe(true),{}),"unmanaged drivers retain old command behavior");
    require(stage.inspect({},Clock::time_point{})==Action::Continue,"legacy frames need no session tag");
}
void source_lock_check(){
    aim_session::Source source(true);auto first=source.observe(true);
    std::promise<void> locked_p,release_p,attempt_p;
    auto locked=locked_p.get_future();auto release=release_p.get_future();
    auto attempt=attempt_p.get_future();
    auto tx=std::async(std::launch::async,[&]{
        return source.with_current([&](const auto &live){
            locked_p.set_value();release.wait();return live.generation;
        });
    });
    locked.wait();
    auto rx=std::async(std::launch::async,[&]{
        attempt_p.set_value();source.observe(false);return source.observe(true).generation;
    });
    attempt.wait();bool blocked=rx.wait_for(20ms)==std::future_status::timeout;
    release_p.set_value();
    require(tx.get()==first.generation && rx.get()==first.generation+1 && blocked,
            "mode transitions serialize with final packet validation and write");
}
void prime(predict::Tracker &tracker){
    auto tp=predict::Tracker::TP::clock::now();Eigen::Vector4d z(5,.4,.8,.2);
    tracker.reset_target(z,tp);
    auto x=tracker.target.tracked_state;x(7)=4.49;
    auto p=tracker.whole_state_ekf.get_P();p*=1e-8;
    tracker.whole_state_ekf.set_estimate(x,p);tracker.target.tracked_state=x;
    tracker.target.vehicle_model_trust=true;tracker.target.vehicle_model_rotation_count=3;
    for(int i=0;i<40;++i)tracker.rotation_prior_.observe(4.49,true,true,.02);
    require(tracker.rotation_prior_.active(),"fixture includes a previously confirmed direction");
}
    bbox_t projected_armor(const std::string &root, const Eigen::Vector3d &center, int id) {
        cv::FileStorage file(root + "/asset/camParam/cam_param_hero.yml", cv::FileStorage::READ);
        cv::Mat rotation_cv, intrinsic, distortion;
        file["R_c2i"] >> rotation_cv;
        file["K"] >> intrinsic;
        file["D"] >> distortion;
        Eigen::Matrix3d camera_to_imu, world_armor;
        cv::cv2eigen(rotation_cv, camera_to_imu);
        world_armor << 1, 0, 0, 0, 0, 1, 0, -1, 0;
        const auto attitude = Attitude();
        const Eigen::Matrix3d rotation = camera_to_imu.transpose() * attitude.R_world2imu() * world_armor;
        const auto camera = mathutils::CoordTransformer::Get().pw_to_pc(center, attitude.R_world2imu());
        cv::Mat matrix, rvec, tvec;
        cv::eigen2cv(rotation, matrix);
        cv::Rodrigues(matrix, rvec);
        cv::eigen2cv(camera, tvec);
        std::vector<cv::Point2d> pixels;
        cv::projectPoints(mathutils::pw_blue_small, rvec, tvec, intrinsic, distortion, pixels);
        bbox_t box{};
        for (int i = 0; i < 4; ++i)
            box.pts[i] = pixels[i];
        box.tag_id = id;
        box.color_id = 1;
        box.confidence = .9;
        box.source = DetectionSource::NEURAL_NETWORK;
        return box;
    }


void predictor_checks(const std::string &root){
    predict::MultiPolicyPredictorSubModule predictor(predict::MultiPolicyPredictorConfig{});
    for(auto &tracker:predictor.trackers)prime(tracker);
    aim_session::Source source(true);auto first=source.observe(true);
    auto data=std::make_shared<ThreadDataPack>();data->robotstatus.aim_session=first;
    data->capture_time=first.started_at+1ms;data->attitude_valid=false;
    predictor.predict(data);
    for(auto &tracker:predictor.trackers){
        require(tracker.get_tracker_state()==TrackingState::IDLE && !tracker.target.vehicle_model_trust &&
                tracker.target.vehicle_model_rotation_count==0 && !tracker.rotation_prior_.active(),
                "entry clears all target trust, switching and direction history");
        require(tracker.whole_state_ekf.get_P()(0,0)==1 &&
                tracker.whole_state_ekf.get_P()(4,4)==1 && tracker.whole_state_ekf.get_P()(7,7)==1,
                "entry restores center and angular-speed covariance");
        require(tracker.target.last_observation_time==predict::Tracker::TP{} &&
                tracker.target.tracked_state(4)==0,"entry drops old height and observation time");
    }
    prime(predictor.trackers[0]);
    data->capture_time+=1ms;predictor.predict(data);
    require(predictor.trackers[0].rotation_prior_.active(),"same enabled session does not repeatedly reset");
    auto stale=std::make_shared<ThreadDataPack>(*data);stale->robotstatus.aim_session.generation=0;
    stale->capture_time+=1ms;predictor.predict(stale);
    require(predictor.trackers[0].rotation_prior_.active(),"late background frame cannot alter new model");
    source.observe(false);auto second=source.observe(true);
    data->robotstatus.aim_session=second;data->capture_time=second.started_at+1ms;
    predictor.predict(data);
    require(!predictor.trackers[0].rotation_prior_.active(),"off-on creates a fresh model even with no off frame");
    data->capture_time+=1ms;
    data->time=predict::Tracker::TP::clock::now();
    data->attitude_valid=true;data->robotstatus.enemy_color=EnemyColor::BLUE;
    data->robotstatus.program_mode=ProgramMode::AUTO_AIM;
    data->frame=cv::Mat::zeros(1080,1440,CV_8UC3);
    data->bboxes={projected_armor(root,{.2,3,.15},2)};
    predictor.predict(data);
    auto &acquired=predictor.trackers[0];
    require(acquired.get_tracker_state()==TrackingState::DETECTING &&
            acquired.last_tp==data->time && acquired.target.tracked_state(7)==0 &&
            !acquired.target.vehicle_model_trust && std::abs(acquired.target.tracked_state(4)-.15)<.01,
            "fresh PnP acquisition replaces old location, height, time and trust");
    require(acquired.whole_state_ekf.get_P()(0,0)==1 && acquired.whole_state_ekf.get_P()(4,4)==1,
            "fresh acquisition uses initial covariance rather than old shrunken covariance");
    const auto new_state=acquired.target.tracked_state;
    auto late=std::make_shared<ThreadDataPack>(*data);
    late->robotstatus.aim_session=first;late->capture_time+=1ms;
    late->bboxes={projected_armor(root,{.4,5,.8},2)};
    predictor.predict(late);
    require((acquired.target.tracked_state-new_state).norm()==0,
            "old-session image cannot pull a freshly acquired model back to an old position");
}
void planner_checks(const std::string &root){
    pipeline::bridge::PlannerToSerialBridge bridge;
    int received=0;pipeline::bridge::PlannerToSerialMessage last{};
    bridge.set_receiver([&](const auto &msg){++received;last=msg;});
    plan::PlannerSubModule stage({},bridge,root+"/asset/plannerParam/planner_param_hero.yml");
    aim_session::Source source(true);auto first=source.observe(true);
    auto data=std::make_shared<ThreadDataPack>();data->robotstatus.aim_session=first;
    data->capture_time=first.started_at+1ms;data->attitude_valid=true;
    stage.process(data,nullptr);
    require(received==1 && last.aim_token.session.generation==1 &&
            last.aim_token.captured_at==data->capture_time,"planner carries capture and session provenance");
    source.observe(false);auto second=source.observe(true);
    auto next=std::make_shared<ThreadDataPack>(*data);
    next->robotstatus.aim_session=second;next->capture_time=second.started_at+1ms;
    stage.process(next,nullptr);
    require(received==2 && last.aim_token.session.generation==2,"planner accepts a fresh session");
    data->capture_time=next->capture_time+1ms;
    require(stage.process(data,nullptr)==SubModuleResult::SKIP && received==2,
            "old work completing late cannot overwrite a new planner result");
}
class Driver final:public SerialInterface {
public:
    aim_session::Source source{true};
    aim_session::Token token;
    AttitudeCallback attitude;RobotStatusCallback status;
    size_t writes=0;int last_fire=-1;
    std::function<void()> snapshot_hook;
    bool init() override{return true;}
    void start() override{attitude(Attitude(0,10,0));}
    void close() override{}
    bool aim_link_enabled() const override{return source.snapshot().enabled;}
    bool fire_actuation_enabled() const override{return aim_link_enabled();}
    aim_session::State aim_session_state() const override{
        auto state=source.snapshot();if(snapshot_hook)snapshot_hook();return state;
    }
    void set_command_session(const aim_session::Token &t)override{token=t;}
    void set_attitude_callback(AttitudeCallback cb)override{attitude=std::move(cb);}
    void set_robot_status_callback(RobotStatusCallback cb)override{status=std::move(cb);}
    void transmit_cmd(float,float,float,float,float,float,float,uint8_t fire,uint8_t)override{
        source.with_current([&](const auto &s){if(aim_session::accepts(s,token)){++writes;last_fire=fire;}});
    }
};
fire_control::Input hit_input(){
    fire_control::Input in;
    in.now=in.rx_time=Clock::now();in.target_id=2;in.aim_mode=4;
    in.eligible=true;in.requested=2;in.distance=3;
    in.desired_yaw=in.sent_yaw=3;in.desired_pitch=in.sent_pitch=in.rx_pitch=10;
    auto &s=in.center_window;s.valid=true;s.state_time=in.now;
    s.bullet_speed=18.5;s.release_delay_s=.23;s.shot_interval_s=.5;
    const double p=10*center_fire::pi/180;
    const double t=std::expm1(center_fire::drag_k*3)/center_fire::drag_k/(18.5*std::cos(p));
    s.state={3.265,0,0,0,18.5*std::sin(p)*t-.5*center_fire::gravity*t*t,0,0,0,.265,0,0};
    return in;
}
void serial_checks(){
    pipeline::bridge::PlannerToSerialBridge plans;
    pipeline::bridge::SensorFromSerialAttitudeBridge attitudes;
    pipeline::bridge::SensorFromSerialRobotStatusBridge statuses;
    auto driver=std::make_unique<Driver>();auto *fake=driver.get();
    auto first=fake->source.observe(true);
    hardware::TimedSerial serial({},std::move(driver),plans,attitudes,statuses);
    pipeline::bridge::PlannerToSerialMessage old{};
    old.attitude=Attitude(0,10,0);old.plan_period=2000us;
    old.aim_token={first,Clock::now()};old.observation_deadline=Clock::now()+200ms;
    old.aim_mode=AimedTargetType::VEHICLE_CENTER_WITH_VEHICLE_MODEL;
    old.fire_control_eligible=old.vehicle_model_trusted=true;
    old.center_window=hit_input().center_window;
    for(auto &c:old.command_array){c.target_id=2;c.distance=3;c.yaw_angle=3;c.fire_enable=2;}
    serial.handle_planner_message(old);
    require(serial.read_latest_command_and_attitude(),"fixture installs a valid old session plan");
    serial.transmit_cached_command();
    require(fake->writes==1 && fake->last_fire==0,"new session still waits for first-shot preparation");
    auto in=hit_input();
    require(serial.fire_gate_.evaluate(in).advice==2,"fixture creates a recent shot cooldown");
    first_shot::Input prep{in.now,in.rx_time,true,2,0,10,3,10,50};
    serial.first_shot_gate_.update(prep);serial.first_shot_gate_.note_dispatched(prep);
    serial.shot_hold_until_=Clock::now()+230ms;
    fake->source.observe(false);auto second=fake->source.observe(true);
    require(!serial.read_latest_command_and_attitude(),"entry drops cached commands before new vision");
    serial.transmit_cached_command();
    require(fake->writes==1 && !serial.model_trusted_cache_ &&
            serial.shot_hold_until_==Clock::time_point{} && !serial.first_shot_gate_.update(prep).dispatched,
            "old direction hold, trust and first-shot history do not cross sessions");
    require(serial.handle_robotstatus_get().aim_session.generation==second.generation,
            "sensor receives authoritative session even before a status callback");
    serial.handle_planner_message(old);
    require(serial.plan_period_.count()==0,"late old plan cannot repopulate an empty cache");
    auto fresh=old;fresh.aim_token={second,second.started_at-1ms};
    serial.handle_planner_message(fresh);
    require(serial.plan_period_.count()==0,"post-entry tag cannot authorize a pre-entry image");
    fresh.aim_token.captured_at=Clock::now();fresh.vehicle_model_trusted=false;
    fresh.fire_control_eligible=false;
    for(auto &c:fresh.command_array)c.yaw_angle=6;
    serial.handle_planner_message(fresh);
    serial.handle_planner_message(old);
    require(serial.command_array_[0].yaw_angle==6 && serial.command_token_.session.generation==second.generation,
            "late old result cannot replace an accepted fresh plan");
    require(serial.read_latest_command_and_attitude(),"new session plan restores tracking");
    serial.transmit_cached_command();
    require(fake->writes==2 && fake->last_fire==0,"untrusted reacquisition tracks without firing");
    in.now+=100ms;in.rx_time=in.now;in.center_window.state_time=in.now;
    const auto decision=serial.fire_gate_.evaluate(in);
    require(decision.advice==0 && std::string(decision.window.reason)=="shot_interval",
            "session reset preserves the original shot cooldown");
    // Pause a validated old callback while a new-session callback updates the cache.
    std::promise<void> checked_p,resume_p;
    auto checked=checked_p.get_future();auto resume=resume_p.get_future().share();
    std::atomic<bool> intercept{true};
    fake->snapshot_hook=[&]{if(intercept.exchange(false)){checked_p.set_value();resume.wait();}};
    auto late=std::async(std::launch::async,[&]{serial.handle_planner_message(fresh);});
    checked.wait();
    fake->source.observe(false);auto third=fake->source.observe(true);
    auto newest=fresh;newest.aim_token={third,Clock::now()};
    for(auto &c:newest.command_array)c.yaw_angle=9;
    serial.handle_planner_message(newest);
    resume_p.set_value();late.get();fake->snapshot_hook={};
    require(serial.command_token_.session.generation==third.generation &&
            serial.command_array_[0].yaw_angle==9,
            "callback paused before command lock cannot overwrite a newer session");

}
void driver_wire_check(bool fixed){
    int master=posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK);
    require(master>=0 && grantpt(master)==0 && unlockpt(master)==0,"virtual serial pair opens");
    int slave=open(ptsname(master),O_RDWR|O_NOCTTY|O_NONBLOCK);
    require(slave>=0,"virtual serial slave opens");
    termios attrs{};require(tcgetattr(slave,&attrs)==0,"virtual serial attributes readable");
    cfmakeraw(&attrs);require(tcsetattr(slave,TCSANOW,&attrs)==0,"virtual serial uses raw bytes");
    XucDriver driver("/dev/unused-by-session-check");driver.stm32_fd_=slave;
    RobotStatus status{};driver.set_robot_status_callback([&](const auto &s){status=s;});
    const auto mode=[&](bool on){
        HeroXucStatusPacket packet{};packet.header=0x5A;packet.pitch=10;
        packet.flags=on?HeroXucFlags::FireMode:0;
        require(driver.process_status_packet(reinterpret_cast<const uint8_t*>(&packet)),
                "valid simulated RX packet accepted");
    };
    const auto receive=[&]{
        uint8_t bytes[64];ssize_t count=read(master,bytes,sizeof(bytes));
        for(int i=0;count<0 && i<20;++i){std::this_thread::sleep_for(1ms);count=read(master,bytes,sizeof(bytes));}
        return count>0?count:ssize_t(0);
    };
    mode(false);mode(true);auto first=driver.aim_session_state();
    require(first.managed==fixed && (fixed?status.program_mode==ProgramMode::AUTO_AIM:true),
            "real driver enables session management only for fixed-target profile");
    driver.set_command_session({first,Clock::now()});
    driver.transmit_cmd(3,0,10,0,0,0,3,0,2);
    require(receive()==31,"fresh session writes a complete packet to virtual UART");
    mode(false);mode(true);
    driver.transmit_cmd(3,0,10,0,0,0,3,0,2);
    require(receive()==(fixed?0:31),"final real-driver check rejects an old generation, preserving free mode");
    auto second=driver.aim_session_state();
    driver.set_command_session({second,second.started_at-1ms});
    driver.transmit_cmd(3,0,10,0,0,0,3,0,2);
    require(receive()==(fixed?0:31),"final real-driver check rejects old capture time");
    driver.set_command_session({second,Clock::now()});
    driver.transmit_cmd(3,0,10,0,0,0,3,0,2);
    require(receive()==31,"fresh plan restores normal packet output");
    mode(false);
    require(!fixed || (status.program_mode==ProgramMode::IDLE && !status.aim_session.enabled),
            "real RX mode is propagated to vision status");
    close(master); // Driver owns the virtual slave. No camera or physical UART is opened.
}
}
int main(int argc,char **argv){try{
    require(argc>=2,"project directory argument required");
    Log console;screen=&console;
    source_checks();source_lock_check();
    const bool free=argc>2 && std::string(argv[2])=="--free";
    driver_wire_check(!free);
    if(!free){
        mathutils::CoordTransformer::Init(std::string(argv[1])+"/asset/camParam/cam_param_hero.yml",false);
        predictor_checks(argv[1]);planner_checks(argv[1]);serial_checks();
        mathutils::CoordTransformer::Destroy();
    }
    std::cout<<"PASS "<<checks<<" aim-session checks; virtual UART only, no camera/physical UART opened\n";
    return 0;
}catch(const std::exception &e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
