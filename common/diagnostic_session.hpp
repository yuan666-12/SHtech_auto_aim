#pragma once
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>

// Per-process diagnostic controls only; no control-path configuration is changed.
namespace diagnostic_session {
using Clock = std::chrono::steady_clock;
inline long long ns(Clock::time_point t) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
}
struct Config {
    bool enabled=false, capture=false, valid=true;
    double seconds=120, fps=5;
    int width=800, jpeg_quality=80;
    unsigned long long max_bytes=128ULL*1024*1024;
    static Config from_environment() {
        Config c;
        const auto yes=[](const char* key) {
            const char* s=std::getenv(key); return s && std::string(s)=="1";
        };
        c.enabled=yes("AUTOAIM_DIAG");c.capture=yes("AUTOAIM_DIAG_CAPTURE");
        if(!c.enabled)return c;
        const auto read=[&](const char* key,double fallback,double lo,double hi) {
            const char* s=std::getenv(key);if(!s)return fallback;
            char* end=nullptr;const double v=std::strtod(s,&end);
            if(end==s || !end || *end || !std::isfinite(v) || v<lo || v>hi) {
                c.valid=false;return fallback;
            }
            return v;
        };
        c.seconds=read("AUTOAIM_DIAG_SECONDS",120,1,3600);
        c.fps=read("AUTOAIM_DIAG_CAPTURE_FPS",5,.2,30);
        const double width=read("AUTOAIM_DIAG_CAPTURE_WIDTH",800,160,1920);
        const double quality=read("AUTOAIM_DIAG_JPEG_QUALITY",80,40,95);
        const double mb=read("AUTOAIM_DIAG_MAX_MB",128,1,4096);
        if(width!=std::floor(width) || quality!=std::floor(quality))c.valid=false;
        c.width=static_cast<int>(width);c.jpeg_quality=static_cast<int>(quality);
        c.max_bytes=static_cast<unsigned long long>(mb*1024*1024);
        if(!c.valid) {
            std::cerr<<"[DIAG] invalid diagnostic parameters; optional recording disabled\n";
            c.enabled=false;c.capture=false;
        }
        return c;
    }
};
inline const Config& config() { static const Config c=Config::from_environment();return c; }
class Session {
    Clock::time_point started_=Clock::now();
    std::filesystem::path directory_;
    std::atomic<unsigned long long> reserved_{0};
    std::atomic<bool> full_{false};
    explicit Session(const char* project) {
        if(!config().enabled)return;
        const auto wall=std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm local{};localtime_r(&wall,&local);
        std::ostringstream name;
        name<<std::put_time(&local,"%Y%m%d-%H%M%S")<<'-'<<getpid();
        directory_=std::filesystem::path(project)/"log"/"diagnostic"/name.str();
        std::error_code error;std::filesystem::create_directories(directory_,error);
        if(error) {
            std::cerr<<"[DIAG] cannot create directory: "<<error.message()<<'\n';
            directory_.clear();return;
        }
        std::cerr<<"[DIAG] directory="<<directory_<<" seconds="<<config().seconds
                 <<" max_bytes="<<config().max_bytes<<" capture="<<config().capture<<'\n';
    }
public:
    static Session& get(const char* project) { static Session value(project);return value; }
    const std::filesystem::path& directory() const { return directory_; }
    bool active() const {
        return !directory_.empty() && !full_.load(std::memory_order_relaxed) &&
            std::chrono::duration<double>(Clock::now()-started_).count()<config().seconds;
    }
    bool reserve(size_t bytes) {
        if(!active())return false;
        auto used=reserved_.load(std::memory_order_relaxed);
        for(;;) {
            if(bytes>config().max_bytes-used) {
                if(!full_.exchange(true))std::cerr<<"[DIAG] byte limit reached; optional recording stopped\n";
                return false;
            }
            if(reserved_.compare_exchange_weak(used,used+bytes,std::memory_order_relaxed))return true;
        }
    }
};
} // namespace diagnostic_session
