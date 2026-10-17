#pragma once
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
namespace hero_log {
using Clock = std::chrono::steady_clock;
enum class Stream { Rx, Tx, Vision, Pnp };
struct Permit {
    bool emit = false, event = false;
    uint64_t sequence = 0, skipped = 0;
};
class RateGate {
    Clock::time_point last_{};
    uint64_t key_ = 0, sequence_ = 0, skipped_ = 0;
    bool initialized_ = false;
public:
    Permit sample(Clock::time_point now, uint64_t key,
                  std::chrono::milliseconds interval) {
        ++sequence_;
        const bool event = !initialized_ || key != key_;
        if (event || now - last_ >= interval) {
            Permit p{true,event,sequence_,skipped_};
            initialized_ = true; key_ = key; last_ = now; skipped_ = 0;
            return p;
        }
        ++skipped_; return {};
    }
};
inline std::chrono::milliseconds interval() {
    static const auto value = [] {
        const char* p = std::getenv("AUTOAIM_XUC_LOG_INTERVAL_MS");
        if (!p || !*p) return 100L;
        char* end = nullptr; long n = std::strtol(p,&end,10);
        return end && !*end && n >= 0 && n <= 5000 ? n : 100L;
    }();
    return std::chrono::milliseconds(value);
}
inline bool raw_enabled() {
    const char* p = std::getenv("AUTOAIM_XUC_LOG_RAW");
    return p && std::string(p) == "1";
}
inline Permit permit(Stream stream, uint64_t key) {
    static std::mutex mutex;
    static std::array<RateGate,4> gates;
    std::lock_guard<std::mutex> lock(mutex);
    return gates[static_cast<size_t>(stream)].sample(Clock::now(),key,interval());
}
inline void append(const std::string& text, bool critical = false) {
    struct Writer {
        std::mutex mutex;
        std::ofstream file;
        Clock::time_point flushed{};
        bool initialized = false;
    };
    static Writer writer;
    std::lock_guard<std::mutex> lock(writer.mutex);
    if (!writer.initialized) {
        writer.initialized = true;
        const auto now = std::chrono::system_clock::now();
        const std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm{}; localtime_r(&t,&tm);
        std::ostringstream path;
        path << PROJECT_DIR << "/log/xuc/xuc_" << std::put_time(&tm,"%Y-%m-%d-%H-%M-%S") << ".log";
        const char* custom = std::getenv("AUTOAIM_XUC_LOG_FILE");
        const std::filesystem::path target = custom && *custom ? custom : path.str();
        std::error_code error;
        if (!target.parent_path().empty()) std::filesystem::create_directories(target.parent_path(),error);
        writer.file.open(target,std::ios::app);
        if (writer.file) writer.file << "[LOG] interval_ms=" << interval().count()
            << " raw=" << raw_enabled() << " events=immediate flush_ms=1000\n";
    }
    if (!writer.file) return;
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{}; localtime_r(&t,&tm);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count()%1000;
    writer.file << std::put_time(&tm,"%Y-%m-%d %H:%M:%S") << '.'
        << std::setfill('0') << std::setw(3) << ms << std::setfill(' ')
        << ' ' << text << '\n';
    const auto steady = Clock::now();
    if (critical || steady-writer.flushed >= std::chrono::seconds(1)) {
        writer.file.flush(); writer.flushed = steady;
    }
}
}
