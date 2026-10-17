#pragma once
#include "datatype.hpp"
#include <deque>
#include <algorithm>
struct HistoricalAttitude {
    Attitude attitude;
    std::chrono::steady_clock::time_point received_at{};
    bool valid = false;
    double error_ms = INFINITY;
};
class AttitudeHistory {
    using Clock = std::chrono::steady_clock;
    struct Entry { Clock::time_point stamp; Attitude attitude; };
    std::deque<Entry> entries_;
public:
    void push(Clock::time_point stamp, const Attitude& attitude) {
        if (!entries_.empty() && stamp <= entries_.back().stamp) return;
        entries_.push_back({stamp,attitude});
        while (entries_.size()>512 ||
               (!entries_.empty() && stamp-entries_.front().stamp>std::chrono::seconds(2)))
            entries_.pop_front();
    }
    HistoricalAttitude sample(Clock::time_point stamp) const {
        if (entries_.empty()) return {};
        auto hi=std::lower_bound(entries_.begin(),entries_.end(),stamp,
            [](const Entry& e,Clock::time_point t){return e.stamp<t;});
        if (hi==entries_.begin() || hi==entries_.end()) {
            const auto& e=hi==entries_.end()?entries_.back():*hi;
            const double error=std::abs(std::chrono::duration<double,std::milli>(stamp-e.stamp).count());
            return {e.attitude,e.stamp,error<=30.0,error};
        }
        const auto& a=*std::prev(hi);const auto& b=*hi;
        const double gap=std::chrono::duration<double,std::milli>(b.stamp-a.stamp).count();
        if (gap>60.0) return {};
        const double weight=std::chrono::duration<double>(stamp-a.stamp).count()/
            std::chrono::duration<double>(b.stamp-a.stamp).count();
        auto angle=[weight](float x,float y){return float(x+weight*std::remainder(double(y-x),360.0));};
        Attitude result(angle(a.attitude.yaw(),b.attitude.yaw()),
                        angle(a.attitude.pitch(),b.attitude.pitch()),
                        angle(a.attitude.roll(),b.attitude.roll()));
        return {result,b.stamp,true,0.0};
    }
};
