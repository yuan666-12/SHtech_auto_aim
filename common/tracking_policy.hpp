#pragma once
#include "datatype.hpp"
#include <cmath>
namespace tracking_policy {
constexpr double observation_timeout_s = 0.200;
using TP = std::chrono::high_resolution_clock::time_point;
inline double observation_age(TP stamp, TP now) {
    if (stamp == TP{}) return INFINITY;
    return std::chrono::duration<double>(now - stamp).count();
}
// Isolated position jumps require a second spatially coherent observation.
// Repeated new positions AND consistent fast motion can both confirm a jump.
class ObservationGate {
    Eigen::Vector4d accepted_ = Eigen::Vector4d::Zero(), pending_ = accepted_;
    TP accepted_at_{}, pending_at_{};
    bool pending_valid_ = false, velocity_valid_ = false;
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
public:
    void reset(const Eigen::Vector4d& z, TP t) {
        accepted_ = z; accepted_at_ = t; pending_valid_ = false; velocity_valid_ = false;
        velocity_.setZero();
    }
    bool consider(const Eigen::Vector4d& z, TP t) {
        if (!z.allFinite()) return false;
        const double dt = observation_age(accepted_at_, t);
        if (dt <= 0.0) return false;
        const double pending_dt = pending_valid_ ? observation_age(pending_at_,t) : INFINITY;
        const bool fresh_candidate = pending_valid_ && pending_dt > 0.0 &&
                                     pending_dt <= observation_timeout_s;
        bool accept = !std::isfinite(dt) || (dt > observation_timeout_s && !fresh_candidate) ||
            (z.head<3>() - accepted_.head<3>()).norm() <= 0.2;
        if (!accept && velocity_valid_)
            accept = (z.head<3>() - accepted_.head<3>() - velocity_ * dt).norm() <= 0.2;
        if (!accept && pending_valid_) {
            const double first = observation_age(accepted_at_, pending_at_);
            const double extra = observation_age(pending_at_, t);
            if (first > 0.0 && extra >= 0.0) {
                Eigen::Vector3d expected = pending_.head<3>() +
                    (pending_.head<3>() - accepted_.head<3>()) * extra / first;
                accept = (z.head<3>() - pending_.head<3>()).norm() <= 0.2 ||
                         (z.head<3>() - expected).norm() <= 0.2;
            }
        }
        if (accept) {
            velocity_valid_ = std::isfinite(dt) && (dt <= observation_timeout_s || fresh_candidate);
            if (dt > observation_timeout_s && fresh_candidate)
                velocity_ = (z.head<3>()-pending_.head<3>()) / pending_dt;
            else if (velocity_valid_) velocity_ = (z.head<3>()-accepted_.head<3>()) / dt;
            else velocity_.setZero();
            accepted_=z;accepted_at_=t;pending_valid_=false;
            return true;
        }
        pending_ = z; pending_at_ = t; pending_valid_ = true;
        return false;
    }
};
}
