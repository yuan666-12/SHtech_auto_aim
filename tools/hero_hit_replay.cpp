// Standalone: c++ -O2 -std=c++20 -Icommon tools/hero_hit_replay.cpp -o hero_hit_replay
#include "hit_diagnostics.hpp"
#include <map>
#include <memory>
#include <limits>

namespace {
std::string configuration_key(const hit_diagnostics::Row &row) {
    static constexpr const char *keys[] = {
        "width", "height", "margin_x", "margin_z", "rearm_ms", "center_config_valid",
        "yaw_tol", "pitch_tol", "ref_distance", "settle_ms", "rx_max_age_ms", "fire_config_valid"};
    std::string result;
    for (const auto *key : keys) result += row.get(key) + ',';
    return result;
}
bool same(double a,double b) {
    return (std::isnan(a)&&std::isnan(b)) || a==b ||
        (std::isfinite(a)&&std::isfinite(b)&&std::abs(a-b)<=1e-10*std::max({1.,std::abs(a),std::abs(b)}));
}
bool geometry_matches(const fire_control::Decision &d,const hit_diagnostics::Row &r) {
    const auto &h=d.window.hit;
    return h.inside==bool(r.integer("hit_inside")) && h.crossed==bool(r.integer("hit_crossed")) &&
        h.front==bool(r.integer("hit_front")) && h.armor==r.integer("hit_armor") &&
        same(h.flight_s,r.number("flight_s")) && same(h.lateral_m,r.number("lateral_m")) &&
        same(h.vertical_m,r.number("vertical_m")) && same(d.window.half_width_m,r.number("half_width")) &&
        same(d.window.half_height_m,r.number("half_height"));
}
struct Summary {
    long rows=0,sessions=0,decision_mismatches=0,geometry_mismatches=0,sequence_gaps=0;
    long evaluated=0,actual_inside=0,ideal_inside=0,ideal_only=0,both_outside=0,holding=0;
    std::map<std::string,long> reasons;
    double abs_yaw_error_sum=0,abs_pitch_error_sum=0;
    long first_ns=0,last_ns=0;
    std::string json() const {
        std::ostringstream out;out<<std::setprecision(12);
        out<<"{\n  \"rows\":"<<rows<<",\n  \"sessions\":"<<sessions
           <<",\n  \"decision_mismatches\":"<<decision_mismatches
           <<",\n  \"geometry_mismatches\":"<<geometry_mismatches
           <<",\n  \"sequence_gaps\":"<<sequence_gaps
           <<",\n  \"span_s\":"<<double(last_ns-first_ns)/1e9
           <<",\n  \"evaluated_windows\":"<<evaluated<<",\n  \"actual_inside\":"<<actual_inside
           <<",\n  \"ideal_inside\":"<<ideal_inside<<",\n  \"ideal_only_inside\":"<<ideal_only
           <<",\n  \"both_outside\":"<<both_outside<<",\n  \"holding_rows\":"<<holding
           <<",\n  \"mean_abs_planned_rx_yaw_deg\":"<<(evaluated?abs_yaw_error_sum/evaluated:0)
           <<",\n  \"mean_abs_planned_rx_pitch_deg\":"<<(evaluated?abs_pitch_error_sum/evaluated:0)
           <<",\n  \"reasons\": {";
        bool comma=false;for(const auto &[key,value]:reasons) {if(comma)out<<',';comma=true;out<<'"'<<key<<"\":"<<value;}
        out<<"}\n}\n";return out.str();
    }
};
void analyze_geometry(const hit_diagnostics::Row &r,const fire_control::Input &in,const fire_control::Decision &d,
                      Summary &s,std::ostream &details) {
    const bool evaluated=std::string(d.window.reason)!="not_evaluated";
    if(!evaluated)return;
    const auto ideal=center_fire::intersect(in.center_window,in.now,r.number("planned_yaw"),r.number("planned_pitch"),r.center_config());
    ++s.evaluated;s.actual_inside+=d.window.hit.inside;s.ideal_inside+=ideal.inside;
    s.ideal_only+=ideal.inside&&!d.window.hit.inside;s.both_outside+=!ideal.inside&&!d.window.hit.inside;
    const double yaw=std::remainder(r.number("planned_yaw")-in.rx_yaw,360.);
    const double pitch=r.number("planned_pitch")-in.rx_pitch;
    s.abs_yaw_error_sum+=std::abs(yaw);s.abs_pitch_error_sum+=std::abs(pitch);
    details<<r.get("sequence")<<','<<r.get("now_ns")<<','<<d.window.reason<<','<<d.window.hit.inside
           <<','<<ideal.inside<<','<<d.window.hit.lateral_m<<','<<d.window.hit.vertical_m
           <<','<<ideal.lateral_m<<','<<ideal.vertical_m<<','<<yaw<<','<<pitch<<','<<r.get("shot_hold")<<'\n';
}
}
int main(int argc,char **argv) {
    try {
        if(argc<2 || argc>3)throw std::runtime_error("usage: hero_hit_replay LOG.csv [SUMMARY.json]");
        std::ifstream input(argv[1]);if(!input)throw std::runtime_error("cannot read diagnostic CSV");
        const std::string output=argc==3?argv[2]:std::string(argv[1])+".replay.json";
        std::ofstream details(output+".windows.csv");details.imbue(std::locale::classic());details<<std::setprecision(17);
        if(!details)throw std::runtime_error("cannot create replay output");
        details<<"sequence,now_ns,window_reason,actual_inside,ideal_inside,actual_x_m,actual_z_m,ideal_x_m,ideal_z_m,planned_rx_yaw_deg,planned_rx_pitch_deg,shot_hold\n";
        Summary s;std::unique_ptr<fire_control::Gate> gate;std::string line,configuration;
        unsigned long long last_sequence=0;
        while(std::getline(input,line)) {
            if(!line.empty()&&line.back()=='\r')line.pop_back();
            if(line.empty() || line[0]=='#')continue;
            if(line==hit_diagnostics::columns || line==hit_diagnostics::legacy_columns) {gate.reset();configuration.clear();last_sequence=0;++s.sessions;continue;}
            hit_diagnostics::Row r(line);const auto in=r.input();
            const auto config=configuration_key(r);
            if(!gate) {gate=std::make_unique<fire_control::Gate>(r.fire_config(),r.center_config());configuration=config;}
            if(config!=configuration)throw std::runtime_error("configuration changed inside one recorded session");
            const auto sequence=std::stoull(r.get("sequence"));if(sequence!=last_sequence+1)++s.sequence_gaps;last_sequence=sequence;
            if(!s.rows)s.first_ns=hit_diagnostics::ns(in.now);s.last_ns=hit_diagnostics::ns(in.now);
            const auto d=gate->evaluate(in);++s.rows;s.holding+=r.integer("shot_hold");++s.reasons[r.get("gate_reason")];
            const bool matched=fire_control::name(d.reason)==r.get("gate_reason") &&
                unsigned(d.advice)==unsigned(r.integer("advice")) && d.window.reason==r.get("window_reason");
            if(!matched)++s.decision_mismatches;
            if(!geometry_matches(d,r))++s.geometry_mismatches;
            analyze_geometry(r,in,d,s,details);
        }
        if(!s.rows)throw std::runtime_error("no synchronized diagnostic rows");
        std::ofstream summary(output);if(!summary)throw std::runtime_error("cannot create summary");
        summary<<s.json();std::cout<<s.json();
        return s.decision_mismatches||s.geometry_mismatches||s.sequence_gaps?2:0;
    }catch(const std::exception &e){std::cerr<<"Replay error: "<<e.what()<<'\n';return 1;}
}
