#pragma once

#include "center_fire_control.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace fire_control {
using Clock = std::chrono::steady_clock;
constexpr double pi = 3.14159265358979323846;

// Wire and internal requests use the same meanings: 0 stop, 1 continuous,
// 2 one shot. Each mode is checked at the 200 Hz serial boundary using live RX.
struct Config {
    double yaw_tolerance_deg = 0.5;
    double pitch_tolerance_deg = 0.2;
    double reference_distance_m = 3.0;
    double settle_ms = 40.0;
    double rx_max_age_ms = 50.0;
    bool valid = true;

    static Config from_environment() {
        Config c;
        const auto read = [&c](const char* name, double& value, bool allow_zero) {
            const char* text = std::getenv(name);
            if (!text) return;
            char* end = nullptr;
            const double n = std::strtod(text, &end);
            if (end == text || !end || *end || !std::isfinite(n) ||
                (allow_zero ? n < 0.0 : n <= 0.0)) {
                c.valid = false; // Invalid tuning must not silently permit fire.
                return;
            }
            value = n;
        };
        read("AUTOAIM_FIRE_YAW_TOL_DEG", c.yaw_tolerance_deg, false);
        read("AUTOAIM_FIRE_PITCH_TOL_DEG", c.pitch_tolerance_deg, false);
        read("AUTOAIM_FIRE_REF_DISTANCE_M", c.reference_distance_m, false);
        read("AUTOAIM_FIRE_SETTLE_MS", c.settle_ms, true);
        read("AUTOAIM_FIRE_RX_MAX_AGE_MS", c.rx_max_age_ms, false);
        if (c.yaw_tolerance_deg >= 90.0 || c.pitch_tolerance_deg >= 90.0)
            c.valid = false;
        return c;
    }

    double tolerance(double base_deg, double distance_m) const {
        // Keep the same lateral error budget beyond the 3 m reference range;
        // do not loosen the gate for observations nearer than that reference.
        return std::atan(std::tan(base_deg * pi / 180.0) *
            reference_distance_m / std::max(reference_distance_m, distance_m)) *
            180.0 / pi;
    }
};

enum class Reason {
    InvalidConfig, InvalidTarget, InvalidRx, PitchLimited, Slewing,
    Misaligned, Settling, NoRequest, SingleConsumed, CenterWindowClosed, Ready
};
inline const char* name(Reason reason) {
    switch (reason) {
        case Reason::InvalidConfig: return "invalid_config";
        case Reason::InvalidTarget: return "invalid_target";
        case Reason::InvalidRx: return "stale_or_invalid_rx";
        case Reason::PitchLimited: return "pitch_limited";
        case Reason::Slewing: return "slewing";
        case Reason::Misaligned: return "misaligned";
        case Reason::Settling: return "settling";
        case Reason::NoRequest: return "no_request";
        case Reason::SingleConsumed: return "single_consumed";
        case Reason::CenterWindowClosed: return "center_window_closed";
        case Reason::Ready: return "ready";
    }
    return "unknown";
}
struct Input {
    Clock::time_point now{}, rx_time{};
    double desired_yaw = 0, desired_pitch = 0;
    double sent_yaw = 0, sent_pitch = 0;
    double rx_yaw = 0, rx_pitch = 0, distance = 0;
    int requested = 0, target_id = 0, aim_mode = 0;
    bool eligible = false;
    uint64_t single_shot_id = 0;
    center_fire::Snapshot center_window;
};
struct Decision {
    uint8_t advice = 0;
    Reason reason = Reason::InvalidTarget;
    double yaw_error_deg = INFINITY, pitch_error_deg = INFINITY;
    double yaw_limit_deg = 0, pitch_limit_deg = 0;
    double rx_age_ms = INFINITY, settled_ms = 0;
    center_fire::Evaluation window;
};
// Per-TX-thread metadata; never changes the wire protocol or fire decision.
struct Diagnostics {
    Decision decision;
    bool valid=false, eligible=false, model_trusted=false;
    bool observation_fresh=false, shot_hold=false;
    int requested=0, gate_requested=0, aim_mode=0;
    int model_rotation_count=0, tracker_model=0;
    double observation_remaining_ms=NAN, model_speed_mps=NAN, rx_age_ms=NAN;
};

class Gate {
    Config config_;
    center_fire::Window center_window_;
    Clock::time_point aligned_since_{}, first_aligned_rx_{}, last_check_{};
    int target_id_ = 0, aim_mode_ = 0;
    uint64_t last_single_shot_id_ = 0;
public:
    explicit Gate(Config config = Config::from_environment()) : config_(config) {}
    const Config& config() const { return config_; }

    Decision evaluate(const Input& in) {
        Decision out;
        // A single request is evaluated only on its first serial cycle.
        // A blocked request is dropped, never queued until the window is gone.
        const bool new_single = in.requested == 2 && in.single_shot_id != 0 &&
            in.single_shot_id > last_single_shot_id_;
        if (new_single) last_single_shot_id_ = in.single_shot_id;
        const auto fail = [&](Reason reason) {
            aligned_since_ = {}; first_aligned_rx_ = {};
            out.reason = reason;
            return out;
        };
        if (target_id_ != in.target_id || aim_mode_ != in.aim_mode ||
            (last_check_ != Clock::time_point{} &&
             std::chrono::duration<double, std::milli>(in.now-last_check_).count() >
                config_.rx_max_age_ms)) {
            aligned_since_ = {}; first_aligned_rx_ = {};
        }
        target_id_ = in.target_id; aim_mode_ = in.aim_mode; last_check_ = in.now;
        if (!config_.valid) return fail(Reason::InvalidConfig);
        if (!in.eligible || in.target_id == 0 || !std::isfinite(in.distance) ||
            in.distance <= 0.0 || in.requested < 0 || in.requested > 2 ||
            !std::isfinite(in.desired_yaw) || !std::isfinite(in.desired_pitch) ||
            !std::isfinite(in.sent_yaw) || !std::isfinite(in.sent_pitch))
            return fail(Reason::InvalidTarget);
        out.rx_age_ms = std::chrono::duration<double, std::milli>(in.now-in.rx_time).count();
        if (in.rx_time == Clock::time_point{} || out.rx_age_ms < 0.0 ||
            out.rx_age_ms > config_.rx_max_age_ms ||
            !std::isfinite(in.rx_yaw) || !std::isfinite(in.rx_pitch))
            return fail(Reason::InvalidRx);
        if (in.desired_pitch < 0.0 || in.desired_pitch > 43.0 ||
            in.sent_pitch < 0.0 || in.sent_pitch > 43.0)
            return fail(Reason::PitchLimited);
        if (in.aim_mode == 4) {
            // A live ballistic intersection replaces arrival at the ideal preaim.
            // No 40 ms dwell is imposed on a moving armor's short hit window.
            aligned_since_ = {}; first_aligned_rx_ = {};
            if (in.rx_pitch < 0.0 || in.rx_pitch > 43.0)
                return fail(Reason::PitchLimited);
            if (!center_window_.config().valid) return fail(Reason::InvalidConfig);
            if (in.requested != 2) { out.reason=Reason::NoRequest; return out; }
            if (center_window_.evaluate(in.center_window,in.now,in.rx_yaw,in.rx_pitch,in.target_id,
                                        &out.window)) {
                out.advice=2;out.reason=Reason::Ready;
            } else out.reason=Reason::CenterWindowClosed;
            return out;
        }
        out.yaw_limit_deg = config_.tolerance(config_.yaw_tolerance_deg, in.distance);
        out.pitch_limit_deg = config_.tolerance(config_.pitch_tolerance_deg, in.distance);
        out.yaw_error_deg = std::abs(std::remainder(in.desired_yaw-in.rx_yaw, 360.0));
        out.pitch_error_deg = std::abs(in.desired_pitch-in.rx_pitch);
        if (std::abs(std::remainder(in.desired_yaw-in.sent_yaw, 360.0)) > out.yaw_limit_deg ||
            std::abs(in.desired_pitch-in.sent_pitch) > out.pitch_limit_deg)
            return fail(Reason::Slewing);
        if (out.yaw_error_deg > out.yaw_limit_deg || out.pitch_error_deg > out.pitch_limit_deg ||
            std::abs(std::remainder(in.sent_yaw-in.rx_yaw, 360.0)) > out.yaw_limit_deg ||
            std::abs(in.sent_pitch-in.rx_pitch) > out.pitch_limit_deg)
            return fail(Reason::Misaligned);
        if (aligned_since_ == Clock::time_point{}) {
            aligned_since_ = in.now; first_aligned_rx_ = in.rx_time;
        }
        out.settled_ms = std::chrono::duration<double, std::milli>(in.now-aligned_since_).count();
        if (out.settled_ms < config_.settle_ms ||
            (config_.settle_ms > 0.0 && in.rx_time <= first_aligned_rx_)) {
            out.reason = Reason::Settling;
        } else if (in.requested == 0) {
            out.reason = Reason::NoRequest;
        } else if (in.requested == 2 && !new_single) {
            out.reason = Reason::SingleConsumed;
        } else {
            out.advice = static_cast<uint8_t>(in.requested);
            out.reason = Reason::Ready;
        }
        return out;
    }
};
} // namespace fire_control
