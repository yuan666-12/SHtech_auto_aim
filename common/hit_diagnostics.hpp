#pragma once
#include "fire_control.hpp"
#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <sstream>
#include <string>
#include <stdexcept>
#include <vector>
#include <unordered_map>
#include <unistd.h>

// Opt-in, full-TX-rate diagnostics. Never supplies values to the control path.
namespace hit_diagnostics {
using Clock=fire_control::Clock;
inline long long ns(Clock::time_point time) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}
inline Clock::time_point time(long long value) {
    return Clock::time_point(std::chrono::duration_cast<Clock::duration>(std::chrono::nanoseconds(value)));
}
struct Transport {
    double wire_yaw_mcu=NAN,wire_pitch=NAN;
    int fire_byte=0,written=0;
    bool link_enabled=false,dry_run=false,track_only=false;
    const char *status="not_written";
};
inline constexpr const char *columns=
    "sequence,wall_ns,now_ns,rx_ns,state_ns,planned_yaw,planned_pitch,desired_yaw,desired_pitch,"
    "slew_yaw,slew_pitch,wire_yaw_mcu,wire_pitch,rx_yaw,rx_pitch,distance,requested_raw,requested_gate,"
    "target_id,aim_mode,eligible,trusted,rotation_count,tracker_model,single_shot_id,shot_hold,"
    "link_enabled,dry_run,track_only,written,fire_byte,transport_status,advice,gate_reason,window_reason,"
    "snapshot_valid,bullet_speed,release_delay,shot_interval,yaw_comp,pitch_comp,air_resistance,"
    "s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,width,height,margin_x,margin_z,rearm_ms,center_config_valid,"
    "yaw_tol,pitch_tol,ref_distance,settle_ms,rx_max_age_ms,fire_config_valid,"
    "hit_inside,hit_crossed,hit_front,hit_armor,flight_s,lateral_m,vertical_m,half_width,half_height,"
    "observation_fresh,observation_remaining_ms";

inline std::string encode(const fire_control::Diagnostics &d,const Transport &tx,unsigned long long seq) {
    const auto &in=d.input;const auto &s=in.center_window;
    const auto &c=d.center_config;const auto &f=d.fire_config;const auto &hit=d.decision.window.hit;
    std::ostringstream out;out.imbue(std::locale::classic());out<<std::setprecision(17);
    out<<seq;
    const auto field=[&](auto value){out<<','<<value;};
    field(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    field(ns(in.now));field(ns(in.rx_time));field(ns(s.state_time));
    field(d.planned_yaw);field(d.planned_pitch);field(in.desired_yaw);field(in.desired_pitch);
    field(in.sent_yaw);field(in.sent_pitch);field(tx.wire_yaw_mcu);field(tx.wire_pitch);
    field(in.rx_yaw);field(in.rx_pitch);field(in.distance);field(d.requested);field(in.requested);
    field(in.target_id);field(in.aim_mode);field(in.eligible);field(d.model_trusted);
    field(d.model_rotation_count);field(d.tracker_model);field(in.single_shot_id);field(d.shot_hold);
    field(tx.link_enabled);field(tx.dry_run);field(tx.track_only);field(tx.written);field(tx.fire_byte);field(tx.status);
    field(unsigned(d.decision.advice));field(fire_control::name(d.decision.reason));field(d.decision.window.reason);
    field(s.valid);field(s.bullet_speed);field(s.release_delay_s);field(s.shot_interval_s);
    field(s.yaw_comp_rad);field(s.pitch_comp_rad);field(s.air_resistance);
    for(double v:s.state)field(v);
    field(c.width_m);field(c.height_m);field(c.margin_x_m);field(c.margin_z_m);field(c.rearm_ms);field(c.valid);
    field(f.yaw_tolerance_deg);field(f.pitch_tolerance_deg);field(f.reference_distance_m);
    field(f.settle_ms);field(f.rx_max_age_ms);field(f.valid);
    field(hit.inside);field(hit.crossed);field(hit.front);field(hit.armor);
    field(hit.flight_s);field(hit.lateral_m);field(hit.vertical_m);
    field(d.decision.window.half_width_m);field(d.decision.window.half_height_m);
    field(d.observation_fresh);field(d.observation_remaining_ms);
    return out.str();
}

inline std::vector<std::string> split(const std::string &line) {
    std::vector<std::string> result;std::istringstream in(line);std::string value;
    while(std::getline(in,value,','))result.push_back(value);
    return result;
}
inline const std::unordered_map<std::string, size_t> &column_indexes() {
    static const auto indexes = [] {
        const auto names = split(columns);
        std::unordered_map<std::string, size_t> result;
        result.reserve(names.size());
        for (size_t i = 0; i < names.size(); ++i) result.emplace(names[i], i);
        return result;
    }();
    return indexes;
}
// Shared decoder: replay uses precisely the recorded doubles and monotonic times.
class Row {
    std::vector<std::string> values_;
public:
    explicit Row(const std::string &line):values_(split(line)) {
        if(values_.size()!=column_indexes().size())throw std::runtime_error("incomplete hit diagnostic row");
    }
    const std::string &get(const std::string &name) const {
        const auto &indexes=column_indexes();
        const auto it=indexes.find(name);
        if(it==indexes.end())throw std::runtime_error("unknown hit diagnostic column: "+name);
        return values_[it->second];
    }
    double number(const std::string &name) const { return std::stod(get(name)); }
    long long integer(const std::string &name) const { return std::stoll(get(name)); }
    fire_control::Input input() const {
        fire_control::Input in;
        in.now=time(integer("now_ns"));in.rx_time=time(integer("rx_ns"));
        in.desired_yaw=number("desired_yaw");in.desired_pitch=number("desired_pitch");
        in.sent_yaw=number("slew_yaw");in.sent_pitch=number("slew_pitch");
        in.rx_yaw=number("rx_yaw");in.rx_pitch=number("rx_pitch");in.distance=number("distance");
        in.requested=integer("requested_gate");in.target_id=integer("target_id");
        in.aim_mode=integer("aim_mode");in.eligible=integer("eligible");
        in.single_shot_id=std::stoull(get("single_shot_id"));
        auto &s=in.center_window;s.state_time=time(integer("state_ns"));s.valid=integer("snapshot_valid");
        s.bullet_speed=number("bullet_speed");s.release_delay_s=number("release_delay");
        s.shot_interval_s=number("shot_interval");s.yaw_comp_rad=number("yaw_comp");
        s.pitch_comp_rad=number("pitch_comp");s.air_resistance=integer("air_resistance");
        for(size_t i=0;i<s.state.size();++i)s.state[i]=number("s"+std::to_string(i));
        return in;
    }
    center_fire::Config center_config() const {
        center_fire::Config c;c.width_m=number("width");c.height_m=number("height");
        c.margin_x_m=number("margin_x");c.margin_z_m=number("margin_z");
        c.rearm_ms=number("rearm_ms");c.valid=integer("center_config_valid");return c;
    }
    fire_control::Config fire_config() const {
        fire_control::Config f;f.yaw_tolerance_deg=number("yaw_tol");f.pitch_tolerance_deg=number("pitch_tol");
        f.reference_distance_m=number("ref_distance");f.settle_ms=number("settle_ms");
        f.rx_max_age_ms=number("rx_max_age_ms");f.valid=integer("fire_config_valid");return f;
    }
};

class Writer {
    std::array<char,65536> buffer_{};
    std::ofstream file_;
    unsigned long long sequence_=0;
    Clock::time_point flushed_{};
public:
    Writer(const char *path,const char *project) {
        if(!path || !*path)return;
        file_.rdbuf()->pubsetbuf(buffer_.data(),buffer_.size());
        file_.imbue(std::locale::classic());file_.open(path,std::ios::app);
        if(!file_) { std::cerr<<"[HIT_DIAG] cannot open diagnostic file: "<<path<<'\n';return; }
        std::error_code error;const auto exe=std::filesystem::read_symlink("/proc/self/exe",error);
        const char *config=std::getenv("AUTOAIM_CONFIG");
        file_<<"# schema=hero-hit-v1 pid="<<getpid()<<" project="<<std::quoted(project)
             <<" executable="<<std::quoted(exe.string())<<" build="<<std::quoted(std::string(__DATE__)+" "+__TIME__)
             <<" config="<<std::quoted(config?config:"")<<'\n'<<columns<<'\n';
    }
    bool enabled() const { return file_.is_open() && file_.good(); }
    void write(const fire_control::Diagnostics &d,const Transport &tx) {
        if(!enabled() || !d.synchronized)return;
        file_<<encode(d,tx,++sequence_)<<'\n';
        if(d.input.now-flushed_>=std::chrono::seconds(1)) {file_.flush();flushed_=d.input.now;}
    }
};
inline void append(const fire_control::Diagnostics &d,const Transport &tx,const char *project) noexcept {
    const char *path=std::getenv("AUTOAIM_HIT_LOG_FILE");
    if(!path || !*path)return;
    try {
        static thread_local Writer writer(path,project);
        writer.write(d,tx);
    }catch(...) { /* Diagnostic failures never change a control decision. */ }
}
}
