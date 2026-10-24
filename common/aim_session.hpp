#pragma once
#include <chrono>
#include <cstdint>
#include <mutex>
#include <utility>

namespace aim_session {
using Clock=std::chrono::steady_clock;
struct State {
    bool managed=false,enabled=false;
    uint64_t generation=0;
    Clock::time_point started_at{};
};
struct Token {
    State session;
    Clock::time_point captured_at{};
};
inline bool same(const State &a,const State &b) {
    return a.managed==b.managed && a.enabled==b.enabled &&
           a.generation==b.generation && a.started_at==b.started_at;
}
inline bool fresh(const State &s,Clock::time_point captured) {
    return s.generation!=0 && s.started_at!=Clock::time_point{} &&
           captured!=Clock::time_point{} && captured>=s.started_at;
}
inline bool accepts(const State &live,const Token &token) {
    return !live.managed ||
        (live.enabled && token.session.managed && token.session.enabled &&
         live.generation==token.session.generation &&
         live.started_at==token.session.started_at && fresh(live,token.captured_at));
}
// Serial RX owns transitions; TX can validate and write under the same lock.
class Source {
    mutable std::mutex mutex_;
    State state_;
public:
    explicit Source(bool managed=false) {state_.managed=managed;}
    State observe(bool enabled,Clock::time_point now=Clock::now()) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(enabled && !state_.enabled) {++state_.generation;state_.started_at=now;}
        state_.enabled=enabled;
        return state_;
    }
    State snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);return state_;
    }
    template<class F> decltype(auto) with_current(F &&f) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return std::forward<F>(f)(state_);
    }
};
enum class Action { Continue, Reset, Reject };
// Each serial stage keeps its own guard. Old frames cannot undo a newer reset.
class StageGuard {
    uint64_t newest_=0,reset_=0;
    Clock::time_point last_frame_{};
public:
    Action inspect(const State &s,Clock::time_point captured) {
        if(!s.managed)return Action::Continue;
        if(s.generation<newest_)return Action::Reject;
        if(s.generation>newest_) {newest_=s.generation;last_frame_={};}
        if(captured==Clock::time_point{} ||
           (s.enabled && !fresh(s,captured)) ||
           (last_frame_!=Clock::time_point{} && captured<=last_frame_))
            return Action::Reject;
        last_frame_=captured;
        if(s.enabled && s.generation!=reset_) {reset_=s.generation;return Action::Reset;}
        return Action::Continue;
    }
};
} // namespace aim_session
