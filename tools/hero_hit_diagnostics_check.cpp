#include "hit_diagnostics.hpp"
#include <cstring>
#include <limits>

using namespace std::chrono_literals;
namespace {
int checks=0;
void require(bool value,const char *message) {++checks;if(!value)throw std::runtime_error(message);}
fire_control::Input fixture() {
    fire_control::Input in;in.now=hit_diagnostics::time(500000000000000001LL);in.rx_time=in.now;
    in.eligible=true;in.target_id=2;in.distance=3;in.aim_mode=4;in.requested=2;
    in.rx_pitch=in.desired_pitch=in.sent_pitch=10;
    auto &s=in.center_window;s.valid=true;s.state_time=in.now;s.bullet_speed=18.5;s.release_delay_s=.23;
    s.air_resistance=true;s.shot_interval_s=.2;
    const double pitch=10*center_fire::pi/180;
    const double flight=std::expm1(center_fire::drag_k*3)/center_fire::drag_k/(s.bullet_speed*std::cos(pitch));
    s.state={3.26,0,0,0,s.bullet_speed*std::sin(pitch)*flight-.5*center_fire::gravity*flight*flight,0,0,0,.26,0,0};
    return in;
}
fire_control::Diagnostics record(fire_control::Gate &gate,const fire_control::Input &in) {
    fire_control::Diagnostics d;d.valid=d.synchronized=true;d.input=in;d.decision=gate.evaluate(in);
    d.planned_yaw=in.desired_yaw;d.planned_pitch=in.desired_pitch;d.requested=in.requested;
    d.fire_config=gate.config();d.center_config=gate.center_config();d.model_trusted=true;
    d.model_rotation_count=3;d.tracker_model=1;
    return d;
}
void scenario(hit_diagnostics::Writer &writer,int index) {
    fire_control::Gate gate;auto in=fixture();
    for(int i=0;i<75;++i) {
        in.now+=5ms;in.rx_time=in.now;
        in.center_window.state_time=in.now;in.rx_yaw=i%20<10?0:5;
        in.requested=(i%17==0)?0:2;
        in.eligible=i%19!=0;
        in.first_shot_ready=i%23!=0;
        if(index==1)in.center_window.state[7]=1.2345678901234567;
        if(index==2)in.center_window.state[7]=-1.2345678901234567;
        if(i==20)in.rx_time-=51ms;
        if(i==30)in.rx_pitch=44;else in.rx_pitch=10;
        if(i==40)in.center_window.state[9]=NAN;else in.center_window.state[9]=0;
        const auto d=record(gate,in);
        const auto line=hit_diagnostics::encode(d,{0,10,0,0,true,true,true,"dry_run"},i+1);
        hit_diagnostics::Row row(line);const auto copy=row.input();
        require(hit_diagnostics::ns(copy.now)==hit_diagnostics::ns(in.now),"nanosecond timestamps round-trip without a double conversion");
        require(copy.rx_yaw==in.rx_yaw && copy.center_window.state[7]==in.center_window.state[7],"actual pose and signed rotation speed round-trip exactly");
        require(copy.center_window.state[0]==in.center_window.state[0] && copy.center_window.release_delay_s==in.center_window.release_delay_s,"model state and release timing are preserved");
        require(row.get("gate_reason")==fire_control::name(d.decision.reason),"decision provenance is retained");
        writer.write(d,{0,10,0,0,true,true,true,"dry_run"});
    }
}
}
int main(int argc,char **argv) {
    try {
        if(argc!=2)throw std::runtime_error("usage: check OUTPUT.csv");
        std::filesystem::remove(argv[1]);
        for(int i=0;i<3;++i) {hit_diagnostics::Writer writer(argv[1],"test-project");scenario(writer,i);}
        auto in=fixture();fire_control::Gate gate;const auto ready=record(gate,in);
        require(ready.decision.advice==2,"fixture contains a real open production window");
        auto held=ready;held.shot_hold=true;held.planned_yaw=8.;
        hit_diagnostics::Row row(hit_diagnostics::encode(held,{0,10,0,0,true,true,true,"dry_run"},1));
        require(row.number("planned_yaw")==8 && row.number("wire_yaw_mcu")==0 && row.number("rx_yaw")==0,
                "pre-hold plan, wire angle and actual attitude remain distinct");
        hit_diagnostics::Writer bad("/proc/hero-hit-check/no-file.csv","test");
        require(!bad.enabled(),"unwritable optional diagnostics do not open a file");
        bad.write(ready,{});
        bool rejected=false;try{hit_diagnostics::Row broken("1,2,3");}catch(const std::runtime_error&){rejected=true;}
        require(rejected,"incomplete rows are rejected instead of silently replayed");
        std::cout<<"PASS "<<checks<<" synchronized snapshot/hold/config/precision checks; no hardware opened\n";
        return 0;
    }catch(const std::exception &e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
