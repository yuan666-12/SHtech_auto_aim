#pragma once
#include "fire_control.hpp"
#include "target_model.hpp"
#include "diagnostic_session.hpp"
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


// Companion schema: original hero-hit-v1 CSV remains replay-compatible.
inline constexpr const char* geometry_columns=
    "sequence,wall_ns,now_ns,gate_begin_ns,gate_end_ns,writer_entry_ns,gate_eval_us,"
    "now_to_writer_ms,model_age_ms,prediction_horizon_ms,impact_ns,fire_byte,advice,"
    "gate_reason,window_reason,hit_armor,hit_inside,hit_front,flight_s,"
    "trajectory_valid,ray_x_m,ray_y_m,ray_z_m,armor_x_m,armor_y_m,armor_z_m,"
    "normal_x,normal_y,side_recomputed_m,height_recomputed_m,side_recorded_m,"
    "height_recorded_m,probe_flight_s,probe_x_m,probe_y_m,probe_z_m";
struct Geometry {
    bool trajectory_valid=false;
    double model_age_ms=NAN,horizon_ms=NAN,probe_flight_s=.2;
    long long impact_ns=0;
    std::array<double,3> ray{NAN,NAN,NAN},armor{NAN,NAN,NAN},probe{NAN,NAN,NAN};
    double nx=NAN,ny=NAN,side=NAN,height=NAN;
};
inline Geometry geometry(const fire_control::Diagnostics& d) {
    Geometry g;const auto& in=d.input;const auto& s=in.center_window;
    const auto& hit=d.decision.window.hit;
    if(s.state_time!=Clock::time_point{})
        g.model_age_ms=std::chrono::duration<double,std::milli>(in.now-s.state_time).count();
    if(!std::isfinite(in.rx_yaw) || !std::isfinite(in.rx_pitch) ||
       !std::isfinite(s.bullet_speed) || s.bullet_speed<=0 || s.bullet_speed>100 ||
       !std::isfinite(s.yaw_comp_rad) || !std::isfinite(s.pitch_comp_rad))return g;
    const double yaw=in.rx_yaw*center_fire::pi/180-s.yaw_comp_rad;
    const double pitch=in.rx_pitch*center_fire::pi/180-s.pitch_comp_rad;
    const double vh=s.bullet_speed*std::cos(pitch),vz=s.bullet_speed*std::sin(pitch);
    if(vh<=1e-6)return g;
    const auto point=[&](double t) {
        const double range=s.air_resistance ?
            std::log1p(center_fire::drag_k*vh*t)/center_fire::drag_k : vh*t;
        return std::array<double,3>{-range*std::sin(yaw),range*std::cos(yaw),
                                   vz*t-.5*center_fire::gravity*t*t};
    };
    g.trajectory_valid=true;g.probe=point(g.probe_flight_s);
    if(!hit.crossed || hit.armor<0 || hit.armor>3 || !std::isfinite(hit.flight_s) ||
       hit.flight_s<=0 || !std::isfinite(g.model_age_ms) ||
       !std::isfinite(s.release_delay_s) ||
       !std::all_of(s.state.begin(),s.state.end(),[](double v){return std::isfinite(v);}))return g;
    const double tau=g.model_age_ms/1000+s.release_delay_s+hit.flight_s;
    const double phase=s.state[6]+s.state[7]*tau+hit.armor*center_fire::pi/2;
    const double radius=s.state[8]+(hit.armor%2?s.state[9]:0);
    g.nx=std::sin(phase);g.ny=std::cos(phase);
    g.ray=point(hit.flight_s);
    g.armor={s.state[2]+s.state[3]*tau-radius*g.nx,
             s.state[0]+s.state[1]*tau-radius*g.ny,
             s.state[4]+s.state[5]*tau+(hit.armor%2?s.state[10]:0)};
    g.side=g.ny*(g.ray[0]-g.armor[0])-g.nx*(g.ray[1]-g.armor[1]);
    g.height=g.ray[2]-g.armor[2];g.horizon_ms=tau*1000;
    g.impact_ns=ns(in.now)+static_cast<long long>((s.release_delay_s+hit.flight_s)*1e9);
    return g;
}
inline std::string encode_geometry(const fire_control::Diagnostics& d,const Transport& tx,
                                   unsigned long long seq,Clock::time_point writer_entry) {
    const auto g=geometry(d);const auto& hit=d.decision.window.hit;
    const bool timed=d.decision_started_at!=Clock::time_point{} &&
        d.decision_finished_at>=d.decision_started_at;
    std::ostringstream out;out.imbue(std::locale::classic());out<<std::setprecision(17)<<seq;
    const auto field=[&](auto value){out<<','<<value;};
    field(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    field(ns(d.input.now));field(ns(d.decision_started_at));field(ns(d.decision_finished_at));
    field(ns(writer_entry));
    field(timed?std::chrono::duration<double,std::micro>(
        d.decision_finished_at-d.decision_started_at).count():NAN);
    field(std::chrono::duration<double,std::milli>(writer_entry-d.input.now).count());
    field(g.model_age_ms);field(g.horizon_ms);field(g.impact_ns);
    field(tx.fire_byte);field(unsigned(d.decision.advice));
    field(fire_control::name(d.decision.reason));field(d.decision.window.reason);
    field(hit.armor);field(hit.inside);field(hit.front);field(hit.flight_s);
    field(g.trajectory_valid);
    for(double v:g.ray)field(v);
    for(double v:g.armor)field(v);
    field(g.nx);field(g.ny);field(g.side);field(g.height);
    field(hit.lateral_m);field(hit.vertical_m);field(g.probe_flight_s);
    for(double v:g.probe)field(v);
    return out.str();
}

class Writer {
    std::array<char,65536> buffer_{},geometry_buffer_{};
    std::ofstream file_,geometry_file_;
    unsigned long long sequence_=0;
    Clock::time_point flushed_{};
    diagnostic_session::Session* session_=nullptr;
public:
    Writer(const char *path,const char *project) {
        if(!path || !*path)return;
        if(diagnostic_session::config().enabled)
            session_=&diagnostic_session::Session::get(project);
        file_.rdbuf()->pubsetbuf(buffer_.data(),buffer_.size());
        file_.imbue(std::locale::classic());file_.open(path,std::ios::app);
        if(!file_) { std::cerr<<"[HIT_DIAG] cannot open diagnostic file: "<<path<<'\n';return; }
        const auto geometry_path=std::filesystem::path(path).string()+".geometry.csv";
        geometry_file_.rdbuf()->pubsetbuf(geometry_buffer_.data(),geometry_buffer_.size());
        geometry_file_.imbue(std::locale::classic());geometry_file_.open(geometry_path,std::ios::app);
        if(!geometry_file_)std::cerr<<"[HIT_DIAG] cannot open geometry file: "<<geometry_path<<'\n';
        std::error_code error;const auto exe=std::filesystem::read_symlink("/proc/self/exe",error);
        const char *config=std::getenv("AUTOAIM_CONFIG");
        std::ostringstream header;
        header<<"# schema=hero-hit-v1 pid="<<getpid()<<" project="<<std::quoted(project)
              <<" executable="<<std::quoted(exe.string())<<" build="
              <<std::quoted(std::string(__DATE__)+" "+__TIME__)
              <<" target_model="<<target_model::config().name()
              <<" radius_m="<<target_model::config().initial_radius_m()
              <<" stationary_center="<<target_model::config().fixed()
              <<" config="<<std::quoted(config?config:"")<<'\n'<<columns<<'\n';
        const std::string detail_header=std::string(
            "# schema=hero-hit-geometry-v1 units=m,rad,s coords=world_xyz origin=gimbal_axis_intersection "
            "timing=steady_ns probe_flight_s=0.2 projectile_direction=actual_rx_minus_compensation\n")+
            geometry_columns+"\n";
        if(session_ && !session_->reserve(header.str().size()+
                (geometry_file_?detail_header.size():0))) {
            file_.close();geometry_file_.close();return;
        }
        file_<<header.str();if(geometry_file_)geometry_file_<<detail_header;
    }
    bool enabled() const { return file_.is_open() && file_.good(); }
    void write(const fire_control::Diagnostics &d,const Transport &tx) {
        if(!enabled() || !d.synchronized || (session_ && !session_->active()))return;
        const auto entry=Clock::now();
        const auto raw=encode(d,tx,sequence_+1);
        const auto detail=geometry_file_?encode_geometry(d,tx,sequence_+1,entry):std::string{};
        if(session_ && !session_->reserve(raw.size()+1+(detail.empty()?0:detail.size()+1)))return;
        ++sequence_;file_<<raw<<'\n';if(!detail.empty())geometry_file_<<detail<<'\n';
        if(entry-flushed_>=std::chrono::seconds(1)) {
            file_.flush();if(geometry_file_)geometry_file_.flush();flushed_=entry;
        }
    }
};
inline void append(const fire_control::Diagnostics &d,const Transport &tx,const char *project) noexcept {
    const char *path=std::getenv("AUTOAIM_HIT_LOG_FILE");
    if((path && !*path) || (!path && !diagnostic_session::config().enabled) || !d.synchronized)return;
    try {
        static thread_local const std::string selected_path=[&] {
            if(path)return std::string(path);
            const auto& session=diagnostic_session::Session::get(project);
            return session.directory().empty()?std::string{}:(session.directory()/"tx.csv").string();
        }();
        static thread_local Writer writer(selected_path.c_str(),project);
        writer.write(d,tx);
    }catch(...) { /* Optional recording never changes a control decision. */ }
}

}
