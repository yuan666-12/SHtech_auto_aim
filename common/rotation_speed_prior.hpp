#pragma once
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string_view>

namespace rotation_speed {
inline constexpr double pi=3.14159265358979323846;
struct Config {
    bool enabled=false;
    double period_s=1.4, sigma_rad_s=.15, drift_std=.05, confirm_ms=500;
    double magnitude() const {return 2*pi/period_s;}
    static Config from_environment(bool fixed_target) {
        Config c;c.enabled=fixed_target;
        if(!fixed_target)return c;
        if(const char *s=std::getenv("AUTOAIM_OMEGA_PRIOR")) {
            if(std::string_view(s)!="0" && std::string_view(s)!="1")
                throw std::invalid_argument("AUTOAIM_OMEGA_PRIOR must be 0 or 1");
            c.enabled=std::string_view(s)=="1";
        }
        const auto read=[](const char *name,double &value,double low,double high) {
            const char *s=std::getenv(name);if(!s)return;
            char *end=nullptr;const double v=std::strtod(s,&end);
            if(end==s || *end || !std::isfinite(v) || v<low || v>high)
                throw std::invalid_argument(name);
            value=v;
        };
        read("AUTOAIM_TARGET_PERIOD_S",c.period_s,.2,60);
        read("AUTOAIM_OMEGA_PRIOR_STD_RAD_S",c.sigma_rad_s,.01,5);
        read("AUTOAIM_OMEGA_DRIFT_STD",c.drift_std,.001,10);
        read("AUTOAIM_OMEGA_DIRECTION_CONFIRM_MS",c.confirm_ms,100,5000);
        return c;
    }
};
struct Noise {double phase=0,cross=0,speed=0;};
// Direction is learned by the existing EKF. Confirm it before applying one
// independent speed prior; never count the same known period as new evidence each frame.
class Prior {
    Config config_;
    int direction_=0,candidate_=0,samples_=0;
    double confirmed_ms_=0;
public:
    explicit Prior(Config c):config_(c) {}
    const Config &config() const {return config_;}
    bool active() const {return config_.enabled && direction_!=0;}
    int direction() const {return direction_;}
    int candidate() const {return candidate_;}
    double confirmed_ms() const {return confirmed_ms_;}
    double target() const {return direction_*config_.magnitude();}
    const char *state() const {
        return !config_.enabled?"disabled":active()?"active":candidate_?"confirming":"waiting";
    }
    void reset() {direction_=candidate_=samples_=0;confirmed_ms_=0;}
    bool observe(double omega,bool observed,bool trusted,double dt) {
        if(!config_.enabled || !trusted) {reset();return false;}
        if(active()) {
            if(observed && std::isfinite(omega) && omega*direction_<-.25*config_.magnitude())reset();
            return false;
        }
        const double magnitude=config_.magnitude();
        if(!observed || !std::isfinite(omega) || dt<=0 ||
           dt>std::min(.2,config_.period_s/8) ||
           std::abs(omega)<.5*magnitude || std::abs(omega)>1.5*magnitude) {
            candidate_=samples_=0;confirmed_ms_=0;return false;
        }
        const int sign=omega>0?1:-1;
        if(candidate_!=sign) {candidate_=sign;samples_=1;confirmed_ms_=0;return false;}
        ++samples_;confirmed_ms_+=dt*1000;
        if(samples_>=8 && confirmed_ms_+1e-6>=config_.confirm_ms) {
            direction_=candidate_;return true;
        }
        return false;
    }
    // Continuous-time white angular acceleration: frame-rate-independent Q.
    Noise noise(double dt) const {
        const double q=config_.drift_std*config_.drift_std;
        return {q*dt*dt*dt/3,q*dt*dt/2,q*dt};
    }
};
} // namespace rotation_speed
