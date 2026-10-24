#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <limits>

// Observe actual, fresh RX motion before the first center shot. No aim-error gate.
namespace first_shot {
using Clock = std::chrono::steady_clock;
struct Config {
    double stable_ms=100, max_rate_dps=3, yaw_span_deg=.4, pitch_span_deg=.25, reposition_deg=5;
    bool valid=true;
    static Config from_environment() {
        Config c;
        const auto read=[&](const char *name,double &value,bool zero=false) {
            const char *s=std::getenv(name);if(!s)return;
            char *end=nullptr;const double v=std::strtod(s,&end);
            if(end==s || *end || !std::isfinite(v) || (zero?v<0:v<=0))c.valid=false;
            else value=v;
        };
        read("AUTOAIM_FIRST_SHOT_STABLE_MS",c.stable_ms,true);
        read("AUTOAIM_FIRST_SHOT_RATE_DPS",c.max_rate_dps);
        read("AUTOAIM_FIRST_SHOT_YAW_SPAN_DEG",c.yaw_span_deg);
        read("AUTOAIM_FIRST_SHOT_PITCH_SPAN_DEG",c.pitch_span_deg);
        read("AUTOAIM_FIRST_SHOT_REPOSITION_DEG",c.reposition_deg);
        if(c.stable_ms>1000 || c.max_rate_dps>1000 || c.reposition_deg>=90 ||
           c.yaw_span_deg>=c.reposition_deg || c.pitch_span_deg>=c.reposition_deg)c.valid=false;
        return c;
    }
};
struct Input {
    Clock::time_point now{},rx_time{};
    bool active=false;
    int target_id=0;
    double yaw=0,pitch=0,planned_yaw=0,planned_pitch=0,rx_max_age_ms=50;
};
struct Result {
    bool ready=false,dispatched=false;
    const char *state="inactive";
    double span_ms=0,yaw_rate_dps=NAN,pitch_rate_dps=NAN,yaw_span_deg=NAN,pitch_span_deg=NAN;
};
class Gate {
    struct Sample {Clock::time_point time;double yaw,pitch;};
    Config config_;
    std::deque<Sample> samples_;
    int target_id_=0;
    bool dispatched_=false;
    double planned_yaw_=0,planned_pitch_=0,shot_yaw_=0,shot_pitch_=0;
    void begin(const Input &in) {
        samples_.clear();dispatched_=false;target_id_=in.target_id;
        planned_yaw_=in.planned_yaw;planned_pitch_=in.planned_pitch;
    }
    static double angle(double a,double b) {return std::remainder(a-b,360.);}
public:
    explicit Gate(Config config=Config::from_environment()):config_(config) {}
    const Config &config() const {return config_;}
    void reset() {samples_.clear();target_id_=0;dispatched_=false;}
    void note_dispatched(const Input &in) {
        dispatched_=true;shot_yaw_=in.yaw;shot_pitch_=in.pitch;
        planned_yaw_=in.planned_yaw;planned_pitch_=in.planned_pitch;
    }
    Result update(const Input &in) {
        Result out;
        if(!config_.valid) {reset();out.state="invalid_config";return out;}
        if(config_.stable_ms==0) {reset();out.ready=true;out.state="disabled";return out;}
        if(!in.active || in.target_id==0) {reset();return out;}
        if(!std::isfinite(in.yaw) || !std::isfinite(in.pitch) ||
           !std::isfinite(in.planned_yaw) || !std::isfinite(in.planned_pitch) ||
           in.rx_time==Clock::time_point{} || in.rx_time>in.now ||
           std::chrono::duration<double,std::milli>(in.now-in.rx_time).count()>in.rx_max_age_ms) {
            reset();out.state="invalid_rx";return out;
        }
        if(target_id_!=in.target_id ||
           std::abs(angle(in.planned_yaw,planned_yaw_))>=config_.reposition_deg ||
           std::abs(in.planned_pitch-planned_pitch_)>=config_.reposition_deg ||
           (dispatched_ && (std::abs(angle(in.yaw,shot_yaw_))>=config_.reposition_deg ||
                            std::abs(in.pitch-shot_pitch_)>=config_.reposition_deg)))
            begin(in);
        if(!samples_.empty() && (in.rx_time<samples_.back().time ||
           std::chrono::duration<double,std::milli>(in.rx_time-samples_.back().time).count()>in.rx_max_age_ms))
            begin(in);
        if(samples_.empty() || in.rx_time>samples_.back().time) {
            const double yaw=samples_.empty()?in.yaw:samples_.back().yaw+angle(in.yaw,samples_.back().yaw);
            samples_.push_back({in.rx_time,yaw,in.pitch});
        }
        const auto cutoff=in.rx_time-std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double,std::milli>(config_.stable_ms));
        while(samples_.size()>2 && samples_[1].time<=cutoff)samples_.pop_front();
        while(samples_.size()>512)samples_.pop_front();
        out.dispatched=dispatched_;
        out.span_ms=std::chrono::duration<double,std::milli>(samples_.back().time-samples_.front().time).count();
        double sy=0,sp=0,st=0,loy=INFINITY,hiy=-INFINITY,lop=INFINITY,hip=-INFINITY;
        for(const auto &s:samples_) {
            sy+=s.yaw;sp+=s.pitch;st+=std::chrono::duration<double>(s.time-samples_.front().time).count();
            loy=std::min(loy,s.yaw);hiy=std::max(hiy,s.yaw);lop=std::min(lop,s.pitch);hip=std::max(hip,s.pitch);
        }
        const double n=samples_.size(),my=sy/n,mp=sp/n,mt=st/n;
        double variance=0,cy=0,cp=0;
        for(const auto &s:samples_) {
            const double dt=std::chrono::duration<double>(s.time-samples_.front().time).count()-mt;
            variance+=dt*dt;cy+=dt*(s.yaw-my);cp+=dt*(s.pitch-mp);
        }
        out.yaw_span_deg=hiy-loy;out.pitch_span_deg=hip-lop;
        if(variance>0) {out.yaw_rate_dps=cy/variance;out.pitch_rate_dps=cp/variance;}
        if(dispatched_) {out.ready=true;out.state="released";return out;}
        if(out.span_ms+1e-6<config_.stable_ms || samples_.size()<2) {out.state="warming";return out;}
        out.ready=std::abs(out.yaw_rate_dps)<=config_.max_rate_dps &&
                  std::abs(out.pitch_rate_dps)<=config_.max_rate_dps &&
                  out.yaw_span_deg<=config_.yaw_span_deg && out.pitch_span_deg<=config_.pitch_span_deg;
        out.state=out.ready?"stable":"moving";
        return out;
    }
};
} // namespace first_shot
