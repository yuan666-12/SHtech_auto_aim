// Accepted synthetic observations -> real Tracker -> Planner bridge -> 200 Hz TX.
// Never sets vehicle_model_trust=true and never opens camera or UART devices.
#include "Tracker.hpp"
#include "planner_submodule.hpp"
#include "timed_serial.hpp"
#include <condition_variable>
#include <iostream>
#include <thread>
using namespace std::chrono_literals;
int checks=0;
void require(bool ok,const char* message) {++checks;if(!ok)throw std::runtime_error(message);}
struct Motion {
    double omega=4;
    bool translating=true;
    double radius=.26;
    int armor(double t) const {
        const double view=std::atan2(x(t),3.26);int best=0;double score=-2;
        for(int j=0;j<4;++j){double s=std::cos(omega*t+j*M_PI_2-view);if(s>score){score=s;best=j;}}
        return best;
    }
    double x(double t)const{return translating?.15*std::sin(.2*t):0;}
    Eigen::Vector4d observation(double t)const {
        double a=mathutils::limit_rad(omega*t+armor(t)*M_PI_2);
        return {3.26-radius*std::cos(a),x(t)-radius*std::sin(a),.1,a};
    }
};
predict::Tracker::TP stamp(predict::Tracker::TP begin,double seconds) {
    return begin+std::chrono::duration_cast<predict::Tracker::TP::duration>(std::chrono::duration<double>(seconds));
}
void rotating_checks(double omega,double dt) {
    predict::Tracker tracker;Motion motion{omega};
    const int frames=int(12/dt);const auto start=predict::Tracker::TP::clock::now()-12100ms;
    int switches=0,last_id=-1,trust_frames=0,last_count=0,first_switches=-1,resets=0;
    double first=-1;
    for(int k=0;k<=frames;++k) {
        double t=k*dt;auto z=motion.observation(t);auto tp=stamp(start,t);
        int id=motion.armor(t);if(last_id>=0&&id!=last_id)++switches;last_id=id;
        const auto& target=k?tracker.track(z,z,1,2,tp,0):tracker.reset_target(z,tp);
        require(target.tracked_state.allFinite(),"rotating model stays finite");
        require(target.vehicle_model_rotation_count>=last_count,"normal rotation cannot erase confirmed switches");
        last_count=target.vehicle_model_rotation_count;resets+=target.filter_reset;
        require(!target.vehicle_model_trust || (target.vehicle_model_rotation_count==3&&switches>=3),
                "trust cannot precede three accepted switches");
        if(target.vehicle_model_trust){++trust_frames;if(first<0){first=t;first_switches=switches;}}
    }
    const auto& target=tracker.get_target();
    const double error=std::hypot(target.tracked_state[0]-3.26,target.tracked_state[2]-motion.x(frames*dt));
    std::cout<<"ROTATION omega="<<omega<<" dt="<<dt<<" visible_switches="<<switches
             <<" count="<<target.vehicle_model_rotation_count<<" trust_frames="<<trust_frames
             <<" first_trust_s="<<first<<" first_visible_switches="<<first_switches
             <<" center_error_m="<<error<<" omega_error="<<std::abs(target.tracked_state[7]-omega)<<"\n";
    require(first>=0&&target.vehicle_model_trust,"continuous rotating observations establish and retain trust");
    require(resets==1,"normal switches do not masquerade as full target resets");
    require(error<.05&&std::abs(target.tracked_state[7]-omega)<.15,
            "noise-free moving vehicle model converges within 50 mm and 0.15 rad/s");
    auto tp=stamp(start,frames*dt+(dt>.05?1.5:.25));auto z=motion.observation(frames*dt);
    const auto& lost=tracker.track(z,z,0,2,tp,0);
    require(lost.predictor_state==TrackingState::IDLE&&!lost.vehicle_model_trust&&
            lost.vehicle_model_rotation_count==0,"real observation expiry clears trust and switch count");
    tp+=20ms;tracker.reset_target(z,tp);
    require(!tracker.get_target().vehicle_model_trust&&tracker.get_target().vehicle_model_rotation_count==0,
            "a new/reacquired target cannot inherit an old vehicle's trust");
}
void negative_checks() {
    predict::Tracker tracker;auto tp=predict::Tracker::TP::clock::now()-5s;
    Eigen::Vector4d z(3,0,.1,0);tracker.reset_target(z,tp);
    for(int k=1;k<=100;++k){tp+=40ms;z[1]=.015*k;tracker.track(z,z,1,2,tp,0);}
    require(!tracker.get_target().vehicle_model_trust&&tracker.get_target().vehicle_model_rotation_count==0,
            "translation of one unchanged plate cannot earn rotation trust");
    auto bad=z;bad[0]+=2;bad[3]=1.57;tp+=20ms;
    const auto& rejected=tracker.track(bad,bad,1,2,tp,0);
    require(rejected.measurement_rejected&&rejected.vehicle_model_rotation_count==0,
            "one isolated pose outlier is not a confirmed armor switch");
    tp+=20ms;tracker.track(z,z,1,2,tp,0);
    require(tracker.get_target().vehicle_model_rotation_count==0,
            "returning to the original plate cannot confirm an isolated switch candidate");
    Motion motion;predict::Tracker rotating;auto start=predict::Tracker::TP::clock::now()-5s;
    bool held=false;Eigen::Vector4d hold;
    for(int k=0;k<120;++k){double t=.02*k;auto current=motion.observation(t);auto tpk=stamp(start,t);
        const auto& target=k?rotating.track(current,current,1,2,tpk,0):rotating.reset_target(current,tpk);
        if(target.vehicle_model_rotation_count==1){hold=current;tp=tpk;held=true;break;}}
    require(held,"a first real switch starts background vehicle modeling");
    for(int k=0;k<100;++k){tp+=20ms;rotating.track(hold,hold,1,2,tp,0);}
    require(rotating.get_target().vehicle_model_rotation_count==1&&!rotating.get_target().vehicle_model_trust,
            "time and stable tracking alone cannot promote one switch to three");
}
struct Driver:SerialInterface {
    struct Packet{uint8_t fire;fire_control::Diagnostics diagnostic;};
    AttitudeCallback attitude;RobotStatusCallback status;float yaw=0,pitch=10;
    std::mutex mutex;std::vector<Packet> packets;fire_control::Diagnostics diagnostic;
    bool init()override{return true;}void close()override{}void start()override{attitude(Attitude(yaw,pitch,0));}
    void set_attitude_callback(AttitudeCallback f)override{attitude=std::move(f);}
    void set_robot_status_callback(RobotStatusCallback f)override{status=std::move(f);}
    void set_fire_diagnostics(const fire_control::Diagnostics& d)override{diagnostic=d;}
    void transmit_cmd(float,float,float,float,float,float,float,uint8_t fire,uint8_t)override {
        std::lock_guard<std::mutex> lock(mutex);packets.push_back({fire,diagnostic});
    }
    bool completed(){std::lock_guard<std::mutex> lock(mutex);bool single=false;
        for(const auto& p:packets){if(p.fire==2)single=true;else if(single&&p.fire==0)return true;}return false;}
};
void pipeline_checks(const std::string& root) {
    mathutils::CoordTransformer::Init(root+"/asset/camParam/cam_param_hero.yml",false);
    pipeline::bridge::PlannerToSerialBridge bridge;
    pipeline::bridge::SensorFromSerialAttitudeBridge attitudes;
    pipeline::bridge::SensorFromSerialRobotStatusBridge statuses;
    plan::PlannerConfig config;config.debug.show_image=false;config.debug.log_text=false;
    auto fake=std::make_unique<Driver>();auto* driver=fake.get();
    hardware::TimedSerial serial({},std::move(fake),bridge,attitudes,statuses);
    plan::PlannerSubModule planner(config,bridge,root+"/asset/plannerParam/planner_param_hero.yml");
    predict::Tracker tracker;Motion motion{4,false};
    if(target_model::config().fixed())motion.radius=target_model::config().radius_m;
    const auto start=predict::Tracker::TP::clock::now()-6100ms;
    auto data=std::make_shared<ThreadDataPack>();
    data->has_fixed_target=true;data->attitude_valid=true;data->robotstatus.robot_speed_mps=18.5;
    data->attitude=Attitude(0,10,0);
    for(int k=0;k<=300;++k){double t=.02*k;auto z=motion.observation(t);auto tp=stamp(start,t);
        if(k)tracker.track(z,z,1,2,tp,0);else tracker.reset_target(z,tp);}
    require(tracker.get_target().vehicle_model_trust,"pipeline input trust comes from actual tracked rotations");
    struct Run{hardware::TimedSerial& serial;std::thread worker;
        explicit Run(hardware::TimedSerial& s):serial(s),worker(std::ref(s)){serial.start();}
        ~Run(){serial.terminate();worker.join();}} run(serial);
    const auto wall=std::chrono::steady_clock::now();
    int observations=0;bool finished=false;
    while(std::chrono::steady_clock::now()-wall<1500ms){
        double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-wall).count();
        if(elapsed>=observations*.02){
            double t=6+elapsed;auto z=motion.observation(t);auto tp=stamp(start,t);
            tracker.track(z,z,1,2,tp,0);data->target=tracker.get_target();
            data->target.tracked_armor.tag_id=2;data->time=tp;
            planner.process(data,nullptr);++observations;
        }
        // The in-memory gimbal follows the real planner's current final angles.
        driver->attitude(Attitude(data->attitude.yaw()+data->robotcommand.yaw_angle,
                                 data->attitude.pitch()+data->robotcommand.pitch_angle,0));
        if(driver->completed()){finished=true;break;}
        std::this_thread::sleep_for(5ms);
    }
    {std::lock_guard<std::mutex> lock(driver->mutex);
        for(const auto& p:driver->packets)if(p.fire==2){
            require(p.diagnostic.model_trusted&&p.diagnostic.model_rotation_count==3&&
                    p.diagnostic.aim_mode==4&&p.diagnostic.decision.window.hit.inside,
                    "code 2 carries real three-switch trust and an actual center intersection");
        }
    }
    require(finished,"real Tracker -> Planner -> timed TX enters center mode and sends 2 then 0");
    std::cout<<"PIPELINE real_observations="<<observations<<" pulse_and_reset="<<finished<<"\n";
    auto tp=data->time+20ms;auto z=motion.observation(6);z[0]=NAN;
    tracker.track(z,z,1,2,tp,0);data->target=tracker.get_target();data->target.tracked_armor.tag_id=2;data->time=tp;
    planner.process(data,nullptr);
    require(data->robotcommand.fire_enable==0,"invalid/rejected observations still inhibit planner fire");
}
int main(int argc,char** argv){try{
    Log console;screen=&console;
    if(argc>2 && std::string(argv[2])=="--pipeline-only")pipeline_checks(argv[1]);
    else {
        for(double omega:{1.,-1.,4.,-4.})rotating_checks(omega,.02);
        rotating_checks(4,.1);negative_checks();pipeline_checks(argv[1]);
    }
    std::cout<<"PASS "<<checks<<" vehicle-model and end-to-end checks; no camera/UART opened\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<"\n";return 1;}}
