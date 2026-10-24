#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace center_fire {
using Clock = std::chrono::steady_clock;
constexpr double pi = 3.14159265358979323846;
constexpr double gravity = 9.7946;
constexpr double drag_k = 0.5 * 1.225 * (2.0*pi*.021*.021) / 2.0 / .041;

// State is at the accepted image time, not the planner publication time.
// [y,vy,x,vx,z,vz,phase,omega,r,delta_r,delta_z], metres/radians/seconds.
struct Snapshot {
    std::array<double,11> state{};
    Clock::time_point state_time{};
    double bullet_speed = 0, release_delay_s = 0, shot_interval_s = .2;
    double yaw_comp_rad = 0, pitch_comp_rad = 0;
    bool air_resistance = false, valid = false;
};
struct Config {
    double width_m=.135, height_m=.056, margin_x_m=.010, margin_z_m=.005;
    double rearm_ms=30;
    bool valid=true;
    static Config from_environment() {
        Config c;
        const auto read=[&](const char* name,double& value,bool zero) {
            const char* text=std::getenv(name); if(!text) return;
            char* end=nullptr; double n=std::strtod(text,&end);
            if(end==text || !end || *end || !std::isfinite(n) ||
               (zero?n<0:n<=0)) { c.valid=false;return; }
            value=n;
        };
        read("AUTOAIM_CENTER_WIDTH_M",c.width_m,false);
        read("AUTOAIM_CENTER_HEIGHT_M",c.height_m,false);
        read("AUTOAIM_CENTER_MARGIN_X_M",c.margin_x_m,true);
        read("AUTOAIM_CENTER_MARGIN_Z_M",c.margin_z_m,true);
        read("AUTOAIM_CENTER_REARM_MS",c.rearm_ms,false);
        c.valid=c.valid && c.width_m<=1 && c.height_m<=1 &&
            2*c.margin_x_m<c.width_m && 2*c.margin_z_m<c.height_m &&
            c.rearm_ms<=1000;
        return c;
    }
};
struct Hit {
    bool inside=false;
    int armor=-1;
    double flight_s=NAN, lateral_m=NAN, vertical_m=NAN;
    bool crossed=false, front=false;
};
struct Evaluation {
    Hit hit;
    const char* reason="not_evaluated";
    double half_width_m=NAN, half_height_m=NAN;
};
inline bool valid(const Snapshot& s,Clock::time_point now) {
    if(!s.valid || s.state_time==Clock::time_point{} ||
       !std::all_of(s.state.begin(),s.state.end(),[](double v){return std::isfinite(v);}) ||
       !std::isfinite(s.bullet_speed) || s.bullet_speed<=0 || s.bullet_speed>100 ||
       !std::isfinite(s.release_delay_s) || s.release_delay_s<0 || s.release_delay_s>2 ||
       !std::isfinite(s.shot_interval_s) || s.shot_interval_s<0 || s.shot_interval_s>10 ||
       !std::isfinite(s.yaw_comp_rad) || !std::isfinite(s.pitch_comp_rad)) return false;
    const double age=std::chrono::duration<double>(now-s.state_time).count();
    return age>=0 && age<=2 && s.state[8]>.01 && s.state[8]<=1 &&
           s.state[8]+s.state[9]>.01 && s.state[8]+s.state[9]<=1;
}

// Same horizontal-drag/vertical-gravity convention as solve_ballistic_trajectory.
// Find intersections with the four moving vertical armor rectangles, rejecting
// their back faces. No ideal preaim angle is used in this calculation.
inline Hit intersect(const Snapshot& s,Clock::time_point now,
                     double rx_yaw_deg,double rx_pitch_deg,const Config& c) {
    Hit out;
    if(!c.valid || !valid(s,now) || !std::isfinite(rx_yaw_deg) ||
       !std::isfinite(rx_pitch_deg)) return out;
    const double yaw=rx_yaw_deg*pi/180-s.yaw_comp_rad;
    const double pitch=rx_pitch_deg*pi/180-s.pitch_comp_rad;
    const double dx=-std::sin(yaw),dy=std::cos(yaw);
    const double vh=s.bullet_speed*std::cos(pitch),vz=s.bullet_speed*std::sin(pitch);
    if(vh<=1e-6) return out;
    const double age=std::chrono::duration<double>(now-s.state_time).count()+s.release_delay_s;
    const double half_width=c.width_m/2-c.margin_x_m;
    const double half_height=c.height_m/2-c.margin_z_m;
    double closest_score=INFINITY;
    struct Sample { double plane,side,height,facing; };
    const auto at=[&](double t,int i) {
        const double tau=age+t;
        const double phase=s.state[6]+s.state[7]*tau+i*pi/2;
        const double nx=std::sin(phase),ny=std::cos(phase);
        const double radius=s.state[8]+(i%2?s.state[9]:0);
        const double ax=s.state[2]+s.state[3]*tau-radius*nx;
        const double ay=s.state[0]+s.state[1]*tau-radius*ny;
        const double az=s.state[4]+s.state[5]*tau+(i%2?s.state[10]:0);
        const double range=s.air_resistance?std::log1p(drag_k*vh*t)/drag_k:vh*t;
        const double ex=range*dx-ax,ey=range*dy-ay;
        return Sample{nx*ex+ny*ey,ny*ex-nx*ey,
                      vz*t-.5*gravity*t*t-az,nx*dx+ny*dy};
    };
    // Bound work at the 5 ms TX boundary. A two-second flight horizon exceeds
    // the Hero's normal low trajectory at the competition range.
    constexpr double step=.005;
    for(int i=0;i<4;++i) {
        double left=0;auto a=at(0,i);
        for(int j=1;j<=400;++j) {
            double right=j*step;auto b=at(right,i);
            if((a.plane<=0 && b.plane>=0)||(a.plane>=0 && b.plane<=0)) {
                double lo=left,hi=right,fl=a.plane;
                for(int k=0;k<24;++k) {
                    const double mid=(lo+hi)/2, fm=at(mid,i).plane;
                    if((fl<=0 && fm>=0)||(fl>=0 && fm<=0)) hi=mid;
                    else { lo=mid;fl=fm; }
                }
                const double t=(lo+hi)/2;const auto hit=at(t,i);
                if(t>1e-6) {
                    const bool front=hit.facing>1e-6;
                    const bool inside=front && std::abs(hit.side)<=half_width &&
                        std::abs(hit.height)<=half_height;
                    const double score=std::max(std::abs(hit.side)/half_width,
                                                std::abs(hit.height)/half_height);
                    // Preserve earliest valid hit. On a miss retain the closest
                    // front-face plane crossing for diagnostics only.
                    if((inside && (!out.inside || t<out.flight_s)) ||
                       (!inside && !out.inside &&
                        (!out.crossed || (front && !out.front) ||
                         (front==out.front && score<closest_score)))) {
                        out={inside,i,t,hit.side,hit.height,true,front};
                        closest_score=score;
                    }
                }
            }
            left=right;a=b;
        }
    }
    return out;
}

class Window {
    Config config_;
    bool latched_=false;
    int target_id_=0;
    Clock::time_point outside_since_{},last_shot_{};
public:
    explicit Window(Config c=Config::from_environment()):config_(c) {}
    const Config& config() const { return config_; }
    void reset_entry() { latched_=false;outside_since_={};target_id_=0; } // Keep shot cooldown across rearming.
    bool evaluate(const Snapshot& s,Clock::time_point now,double yaw,double pitch,int id,
                  Evaluation* diagnostic=nullptr) {
        if(diagnostic) *diagnostic={};
        if(diagnostic && config_.valid) {
            diagnostic->half_width_m=config_.width_m/2-config_.margin_x_m;
            diagnostic->half_height_m=config_.height_m/2-config_.margin_z_m;
        }
        const auto result=[&](const char* reason,bool fire=false) {
            if(diagnostic) diagnostic->reason=reason;
            return fire;
        };
        if(!config_.valid) return result("invalid_config");
        if(!valid(s,now)) return result("invalid_model");
        if(id==0) return result("no_target");
        if(id!=target_id_) { target_id_=id;latched_=false;outside_since_={}; }
        const auto hit=intersect(s,now,yaw,pitch,config_);
        if(diagnostic) diagnostic->hit=hit;
        if(!hit.inside) {
            if(outside_since_==Clock::time_point{}) outside_since_=now;
            if(std::chrono::duration<double,std::milli>(now-outside_since_).count()>=config_.rearm_ms)
                latched_=false;
            return result(!hit.crossed?"no_plane_crossing":
                          !hit.front?"back_face":"outside_rectangle");
        }
        outside_since_={};
        if(latched_) return result("window_consumed");
        latched_=true; // Even a cooldown-blocked entry cannot queue a late shot.
        if(last_shot_!=Clock::time_point{} &&
           std::chrono::duration<double>(now-last_shot_).count()<s.shot_interval_s)
            return result("shot_interval");
        last_shot_=now;
        return result("ready",true);
    }
};
} // namespace center_fire
