#include "rotation_speed_prior.hpp"
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
// Real Tracker with synthetic observations; no camera, UART or firing device.
#include "common.hpp"
#include "Kalman.hpp"
#include "tracking_policy.hpp"
#include "math_tools.hpp"
#include "IESEKF_Double_Armor.hpp"
#include <iostream>
#include <ctime>
#include <array>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <cfloat>
#include <random>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#define private public
#include "Tracker.hpp"
#undef private

namespace {
int checks=0;
void require(bool ok,const char *message){++checks;if(!ok)throw std::runtime_error(message);}
using Tracker=predict::Tracker;
struct Pair {
    std::unique_ptr<Tracker> base,soft;
    Pair() {
        setenv("AUTOAIM_OMEGA_PRIOR","0",1);base=std::make_unique<Tracker>();
        setenv("AUTOAIM_OMEGA_PRIOR","1",1);soft=std::make_unique<Tracker>();
    }
    void step(Eigen::Vector4d z,Tracker::TP tp,int count,bool reset=false) {
        if(reset){base->reset_target(z,tp);soft->reset_target(z,tp);}
        else {base->track(z,z,count,2,tp,0);soft->track(z,z,count,2,tp,0);}
    }
};
Eigen::Vector4d observation(double phase,double radius,std::mt19937 &random,bool noisy=true) {
    const double cy=3.5,cx=.15,cz=.12;
    const double view=std::atan2(cx,cy);int id=0;double best=-2;
    for(int i=0;i<4;++i){double score=std::cos(phase+i*M_PI_2-view);if(score>best){best=score;id=i;}}
    const double a=mathutils::limit_rad(phase+id*M_PI_2);
    std::normal_distribution<double> pos(0,noisy?.003:0),yaw(0,noisy?.08:0);
    return {cy-radius*std::cos(a)+pos(random),cx-radius*std::sin(a)+pos(random),cz+pos(random),a+yaw(random)};
}
void synthetic(double omega,int seed) {
    Pair pair;std::mt19937 random(seed);auto start=Tracker::TP::clock::now();
    double wb=0,ws=0,pb=0,ps=0,mean=0;int n=0;bool ever=false;
    for(int k=0;k<=1500;++k) {
        const double t=k*.02,phase=.1+omega*t;
        auto tp=start+std::chrono::milliseconds(k*20);
        auto z=observation(phase,.265,random);
        pair.step(z,tp,k>10 && k%101==0?0:1,k==0);
        const auto &b=pair.base->get_target(),&s=pair.soft->get_target();
        const auto &prior=pair.soft->rotation_prior_;
        require(!prior.active() || (s.vehicle_model_trust && prior.direction()*omega>0),
                "confirmed speed follows trusted, correct observed direction");
        if(prior.active()) {
            ever=true;require(pair.soft->whole_state_ekf.get_P()(7,7)>0,"speed uncertainty is not hard zero");
            for(int i:target_model::Config::fixed_axes)
                require(pair.soft->whole_state_ekf.get_P().row(i).isZero(0),"fixed geometry covariance stays constrained");
        }
        if(t>3) {
            const auto &xb=b.tracked_state,&xs=s.tracked_state;
            wb+=std::pow(xb(7)-omega,2);ws+=std::pow(xs(7)-omega,2);mean+=xs(7);
            const double truth=phase+omega*.48;
            pb+=std::pow(std::remainder(xb(6)+xb(7)*.48-truth,M_PI_2),2);
            ps+=std::pow(std::remainder(xs(6)+xs(7)*.48-truth,M_PI_2),2);++n;
        }
    }
    std::cout<<"SYNTH omega="<<omega<<" base_omega_rmse="<<std::sqrt(wb/n)
             <<" prior_omega_rmse="<<std::sqrt(ws/n)<<" base_phase_rmse="<<std::sqrt(pb/n)
             <<" prior_phase_rmse="<<std::sqrt(ps/n)<<" mean_omega="<<mean/n<<"\n";
    require(ever && ws<wb && ps<pb,"independent noisy synthetic motion improves speed and future phase estimates");
    require(std::abs(mean/n-omega)<.04,"small nominal-period error remains correctable by observations");
    auto tp=start+std::chrono::seconds(31);auto z=observation(.1,.265,random,false);
    pair.soft->reset_target(z,tp);
    require(!pair.soft->rotation_prior_.active() && pair.soft->whole_state_ekf.get_P()(7,7)==1,
            "reacquisition clears direction and restores initial speed uncertainty");
    for(int k=1;k<=400;++k) {
        auto tpk=tp+std::chrono::milliseconds(k*20);auto zk=observation(.1-omega*k*.02,.265,random,false);
        pair.soft->track(zk,zk,1,2,tpk,0);
    }
    require(pair.soft->rotation_prior_.active() && pair.soft->rotation_prior_.direction()*omega<0,
            "another startup can confirm the opposite direction");
}
std::vector<std::string> split(const std::string &line) {
    std::vector<std::string> out;std::istringstream in(line);std::string v;
    while(std::getline(in,v,','))out.push_back(v);return out;
}
struct Record {long long stamp;int observed;double measured,base_phase,base_speed,phase,speed;bool active;int direction;};
double quantile(std::vector<double> values,double p) {
    std::sort(values.begin(),values.end());double x=(values.size()-1)*p;size_t i=size_t(x);
    return values[i]+(values[std::min(i+1,values.size()-1)]-values[i])*(x-i);
}
void replay(const char *input,const char *output) {
    Pair pair;std::ifstream file(input);require(bool(file),"recorded observations open");
    std::ofstream out(output);require(bool(out),"replay output opens");out<<std::setprecision(17);
    out<<"image_ns,observed,measured_yaw,base_phase,base_omega,prior_phase,prior_omega,active,direction\n";
    std::string line;std::getline(file,line);std::vector<Record> records;double max_error=0;
    while(std::getline(file,line)) {
        auto v=split(line);require(v.size()==12,"complete measurement replay row");
        long long stamp=std::stoll(v[0]);auto tp=Tracker::TP(std::chrono::nanoseconds(stamp));
        int count=std::stoi(v[2]);Eigen::Vector4d z;for(int i=0;i<4;++i)z(i)=std::stod(v[i+3]);
        pair.step(z,tp,count,std::stoi(v[1]));
        const auto &b=pair.base->get_target().tracked_state,&s=pair.soft->get_target().tracked_state;
        for(int j=0;j<5;++j) {
            const int index[]={0,2,4,6,7};
            double error=b(index[j])-std::stod(v[j+7]);if(j==3)error=mathutils::limit_rad(error);
            max_error=std::max(max_error,std::abs(error));
        }
        const auto &p=pair.soft->rotation_prior_;
        records.push_back({stamp,count>0,z(3),b(6),b(7),s(6),s(7),p.active(),p.direction()});
        out<<stamp<<','<<(count>0)<<','<<z(3)<<','<<b(6)<<','<<b(7)<<','<<s(6)<<','<<s(7)<<','<<p.active()<<','<<p.direction()<<'\n';
    }
    require(max_error<2e-4,"prior-disabled replay reproduces recorded baseline states within logging precision");
    std::vector<double> wb,ws,eb,es;int active=0;
    for(const auto &r:records) {
        if(r.stamp-records.front().stamp<2000000000LL)continue;
        wb.push_back(r.base_speed);ws.push_back(r.speed);active+=r.active;
        const long long future=r.stamp+480000000;
        auto it=std::lower_bound(records.begin(),records.end(),future,[](const Record &x,long long t){return x.stamp<t;});
        if(it==records.end() || !it->observed || std::abs(it->stamp-future)>40000000)continue;
        const double h=(it->stamp-r.stamp)/1e9;
        eb.push_back(std::abs(std::remainder(r.base_phase+r.base_speed*h-it->measured,M_PI_2)));
        es.push_back(std::abs(std::remainder(r.phase+r.speed*h-it->measured,M_PI_2)));
    }
    require(!wb.empty() && !eb.empty() && active>0,"recorded comparison has observations and active prior");
    std::cout<<std::setprecision(12)<<"SPEED_REPLAY {\"rows\":"<<records.size()<<",\"baseline_max_state_error\":"<<max_error
        <<",\"stable_rows\":"<<wb.size()<<",\"active_stable_rows\":"<<active
        <<",\"base_omega_p05\":"<<quantile(wb,.05)<<",\"base_omega_median\":"<<quantile(wb,.5)
        <<",\"base_omega_p95\":"<<quantile(wb,.95)<<",\"prior_omega_p05\":"<<quantile(ws,.05)
        <<",\"prior_omega_median\":"<<quantile(ws,.5)<<",\"prior_omega_p95\":"<<quantile(ws,.95)
        <<",\"base_future_observation_abs_p50_rad\":"<<quantile(eb,.5)<<",\"base_future_observation_abs_p90_rad\":"<<quantile(eb,.9)
        <<",\"prior_future_observation_abs_p50_rad\":"<<quantile(es,.5)<<",\"prior_future_observation_abs_p90_rad\":"<<quantile(es,.9)<<"}\n";
}
void free_check() {
    Pair pair;auto start=Tracker::TP::clock::now();std::mt19937 random(99);
    for(int k=0;k<500;++k) {
        auto tp=start+std::chrono::milliseconds(k*20);auto z=observation(.1+4*k*.02,.265,random);
        pair.step(z,tp,1,k==0);
        require(!pair.soft->rotation_prior_.config().enabled,"free mode ignores omega-prior opt-in");
        require((pair.base->get_target().tracked_state-pair.soft->get_target().tracked_state).norm()==0,
                "free-model numeric state is unchanged");
    }
}
}
int main(int argc,char **argv){try{
    Log console;screen=&console;
    if(argc==4 && std::string(argv[1])=="--replay")replay(argv[2],argv[3]);
    else if(argc==2 && std::string(argv[1])=="--free")free_check();
    else {
        const double nominal=2*rotation_speed::pi/1.4;
        synthetic(nominal,17);synthetic(-nominal,19);synthetic(4.451,23);synthetic(-4.451,29);
    }
    std::cout<<"PASS "<<checks<<" rotation-speed checks; no hardware opened\n";return 0;
}catch(const std::exception &e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
