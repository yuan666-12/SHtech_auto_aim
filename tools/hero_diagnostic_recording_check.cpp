#include "diagnostic_capture.hpp"
#include "hit_diagnostics.hpp"
#include <cassert>
#include <thread>
using namespace std::chrono_literals;
static void require(bool ok,const char* what) { if(!ok)throw std::runtime_error(what); }
static fire_control::Diagnostics fixture() {
    fire_control::Diagnostics d;d.valid=d.synchronized=true;
    auto& in=d.input;in.now=fire_control::Clock::now();in.rx_time=in.now;
    in.eligible=true;in.target_id=2;in.distance=3;in.aim_mode=4;in.requested=2;in.rx_pitch=10;
    auto& s=in.center_window;s.valid=true;s.state_time=in.now-20ms;
    s.bullet_speed=18.5;s.release_delay_s=.23;s.air_resistance=true;
    s.state={3.26,.01,.4,.02,.35,0,.1,1.5,.26,.03,.01};
    d.decision_started_at=fire_control::Clock::now();
    fire_control::Gate gate;d.decision=gate.evaluate(in);
    d.decision_finished_at=fire_control::Clock::now();
    d.fire_config=gate.config();d.center_config=gate.center_config();
    return d;
}
int main(int argc,char** argv) {
  try {
    require(argc==3,"mode project");
    const std::string mode=argv[1];const char* project=argv[2];
    auto d=fixture();
    if(mode=="geometry") {
      unsigned crossed=0,odd=0;
      for(int i=0;i<500;++i) {
        auto& s=d.input.center_window;s.state[6]=i*.071;s.state[7]=(i%2?-1:1)*2.1;
        s.state[2]=.4*std::sin(i*.07);d.input.rx_yaw=(i%21-10)*.2;
        s.yaw_comp_rad=.06;s.pitch_comp_rad=.075;
        d.decision.window.hit=center_fire::intersect(s,d.input.now,d.input.rx_yaw,d.input.rx_pitch,d.center_config);
        const auto g=hit_diagnostics::geometry(d);const auto& h=d.decision.window.hit;
        require(g.trajectory_valid,"trajectory valid");
        if(h.crossed) {
          ++crossed;if(h.armor%2)++odd;
          require(std::abs(g.side-h.lateral_m)<1e-12,"same lateral miss");
          require(std::abs(g.height-h.vertical_m)<1e-12,"same vertical miss");
          require(std::abs(g.nx*(g.ray[0]-g.armor[0])+g.ny*(g.ray[1]-g.armor[1]))<1e-7,"same plane");
          require(std::abs(g.horizon_ms-(20+230+h.flight_s*1000))<1e-9,"correct horizon");
        }
        auto changed=d;changed.input.center_window.state[2]+=.4;
        require(hit_diagnostics::geometry(changed).probe==g.probe,"fixed-time curve independent of plate");
      }
      require(crossed>400 && odd>0,"moving even/odd planes covered");
      const auto row=hit_diagnostics::split(hit_diagnostics::encode_geometry(d,{},1,fire_control::Clock::now()));
      require(row.size()==hit_diagnostics::split(hit_diagnostics::geometry_columns).size(),"geometry column count");
      d.input.rx_yaw=NAN;require(!hit_diagnostics::geometry(d).trajectory_valid,"invalid pose");
      std::cout<<"PASS geometry crossed="<<crossed<<" odd="<<odd<<"\n";
    } else if(mode=="capture") {
      ThreadDataPack data;data.frame=cv::Mat(1080,1440,CV_8UC3);cv::randu(data.frame,0,255);
      const cv::Mat original=data.frame.clone();data.attitude_valid=true;
      {diagnostic_capture::Recorder recorder(project);
       for(int i=0;i<140;++i) {
        data.time=std::chrono::high_resolution_clock::now();data.camera_frame_number=i;
        recorder.submit(data);std::this_thread::sleep_for(1ms);
       }}
      require(cv::norm(data.frame,original,cv::NORM_INF)==0,"source image unchanged");
      std::cout<<"PASS capture\n";
    } else if(mode=="disabled" || mode=="invalid") {
      require(!diagnostic_session::config().enabled,"optional recording disabled");
      ThreadDataPack data;data.frame=cv::Mat(480,640,CV_8UC3);
      const auto start=fire_control::Clock::now();
      for(int i=0;i<20000;++i) {hit_diagnostics::append(d,{},project);diagnostic_capture::sample(data,project);}
      const auto us=std::chrono::duration<double,std::micro>(fire_control::Clock::now()-start).count()/20000;
      require(!std::filesystem::exists(std::filesystem::path(project)/"log"),"no default diagnostic files");
      std::cout<<"PASS disabled mean_us="<<us<<"\n";
    } else if(mode=="duration") {
      auto& s=diagnostic_session::Session::get(project);require(s.reserve(20),"start active");
      std::this_thread::sleep_for(1100ms);require(!s.active()&&!s.reserve(20),"duration stops recording");
      std::cout<<"PASS duration\n";
    } else if(mode=="capacity" || mode=="writer") {
      const auto start=fire_control::Clock::now();
      for(int i=0;i<6000;++i) {d.input.now=fire_control::Clock::now();hit_diagnostics::append(d,{},project);}
      std::cout<<"PASS writer mean_us="<<std::chrono::duration<double,std::micro>(fire_control::Clock::now()-start).count()/6000<<"\n";
      if(mode=="capacity") require(!diagnostic_session::Session::get(project).active(),"capacity stops recording");
    } else throw std::runtime_error("unknown mode");
    return 0;
  } catch(const std::exception& e) {std::cerr<<"FAIL "<<e.what()<<"\n";return 1;}
}
