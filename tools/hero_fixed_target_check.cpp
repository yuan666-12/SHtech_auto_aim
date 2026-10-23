// Real Tracker with synthetic observations; no camera, UART or firing device.
#include "common.hpp"
#include "Kalman.hpp"
#include "tracking_policy.hpp"
#include "math_tools.hpp"
#include "IESEKF_Double_Armor.hpp"
#include <iostream>
#include <ctime>
#include <array>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <cfloat>
#include <random>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#define private public
#include "Tracker.hpp"
#undef private

namespace {
int checks = 0;
void require(bool ok, const char *message) {
    ++checks;
    if (!ok) throw std::runtime_error(message);
}
void config_checks() {
    require(!target_model::parse(nullptr, nullptr).fixed(), "CLI defaults to free model");
    require(!target_model::parse("free", nullptr).fixed(), "explicit free mode");
    require(target_model::parse("province_fixed", nullptr).radius_m == .265, "measured default radius");
    require(target_model::parse("province_fixed", "0.3").radius_m == .3, "radius override");
    for (const char *mode : {"fixed", "Province_fixed"}) {
        bool rejected = false;
        try { target_model::parse(mode, nullptr); } catch (const std::invalid_argument &) { rejected = true; }
        require(rejected, "unknown mode rejected");
    }
    for (const char *radius : {"nan", "inf", "0", "-1", "0.119", "0.401", "26.5", "0.265m", " "}) {
        bool rejected = false;
        try { target_model::parse("province_fixed", radius); } catch (const std::invalid_argument &) { rejected = true; }
        require(rejected, "invalid fixed radius rejected");
    }
}
void invariant(predict::Tracker &tracker) {
    const auto &x = tracker.get_target().tracked_state;
    const auto internal = tracker.whole_state_ekf.get_X();
    const auto covariance = tracker.whole_state_ekf.get_P();
    require(x.allFinite() && covariance.allFinite(), "finite state and covariance");
    require(x.isApprox(internal, 1e-13), "public and internal states agree");
    for (int index : target_model::Config::fixed_axes) {
        const double expected = index == 8 ? target_model::config().radius_m : 0.;
        require(x(index) == expected, "physical dimensions and center velocity remain exact");
        require(covariance.row(index).isZero(0) && covariance.col(index).isZero(0),
                "constrained states cannot gain uncertainty or measurement corrections");
    }
    require(tracker.get_target().updating_model_type == UpdatingModelType::VEHICLE_MODEL,
            "fixed target uses vehicle model even at low speed");
}
void rotating(double omega, double dt) {
    predict::Tracker tracker;
    const double radius = target_model::config().radius_m;
    const double cy = 4.5, cx = -.2, cz = .3;
    std::mt19937 random(20261024);
    std::normal_distribution<double> noise(0, .002);
    auto tp = predict::Tracker::TP::clock::now();
    const auto observation = [&](double t) {
        int id = 0; double best = -2;
        for (int candidate = 0; candidate < 4; ++candidate) {
            const double score = std::cos(.1 + omega * t + candidate * M_PI_2 - std::atan2(cx, cy));
            if (score > best) { best = score; id = candidate; }
        }
        const double angle = mathutils::limit_rad(.1 + omega * t + id * M_PI_2);
        return Eigen::Vector4d(cy - radius * std::cos(angle) + noise(random),
                               cx - radius * std::sin(angle) + noise(random),
                               cz + noise(random), angle + noise(random));
    };
    auto z = observation(0);
    // Initial position/normal errors must be corrected, without inventing translation.
    z(0) += .02; z(3) += .04;
    tracker.reset_target(z, tp);
    invariant(tracker);
    require(!tracker.get_target().vehicle_model_trust, "known dimensions do not grant trust");
    int resets = 0, min_count = 0;
    const int frames = int(35 / dt);
    for (int k = 1; k <= frames; ++k) {
        tp += std::chrono::duration_cast<predict::Tracker::TP::duration>(std::chrono::duration<double>(dt));
        z = observation(k * dt);
        const bool missing = k > 10 && k % 83 < 3;
        const int tag = k % 3 == 0 ? 8 : (k % 3 == 1 ? 0 : 2);
        const auto &target = tracker.track(z, z, missing ? 0 : 1, tag, tp, 0);
        invariant(tracker);
        resets += target.filter_reset;
        require(target.vehicle_model_rotation_count >= min_count, "short losses preserve switch progress");
        min_count = target.vehicle_model_rotation_count;
        require(!target.vehicle_model_trust || min_count >= 3, "three switches still required");
    }
    const auto &x = tracker.get_target().tracked_state;
    const double center_error = Eigen::Vector3d(x(0) - cy, x(2) - cx, x(4) - cz).norm();
    const double phase_error = std::abs(mathutils::limit_rad(x(6) + x(7) * .45 - (.1 + omega * (frames * dt + .45))));
    // Plate index is arbitrary; compare phase modulo a quarter turn.
    const double equivalent_phase_error = std::abs(std::remainder(phase_error, M_PI_2));
    std::cout << "FIXED omega=" << omega << " dt=" << dt << " center_error_m=" << center_error
              << " future_phase_error_rad=" << equivalent_phase_error
              << " omega_error=" << std::abs(x(7) - omega) << " resets=" << resets << '\n';
    require(resets == 0, "continuous rotations and short dropouts do not reset");
    require(center_error < .015, "stationary center converges despite initial bias and observation noise");
    require(std::abs(x(7) - omega) < .08 && equivalent_phase_error < .06,
            "rotation and future phase remain responsive");
    require(tracker.get_target().vehicle_model_trust == (omega != 0), "only observed rotation earns trust");
    for (int k = 0; k < 35; ++k) {
        tp += std::chrono::milliseconds(20);
        tracker.track(z, z, 0, 2, tp, 0);
        invariant(tracker);
    }
    require(!tracker.get_target().vehicle_model_trust, "expired tracking clears trust");
    z = observation(0); tp += std::chrono::milliseconds(20); tracker.reset_target(z, tp);
    invariant(tracker);
    const auto covariance = tracker.whole_state_ekf.get_P();
    require(covariance(0, 0) == 1 && covariance(2, 2) == 1 && covariance(4, 4) == 1,
            "reacquisition restores free center uncertainty");
    require(!tracker.get_target().vehicle_model_trust, "reacquisition cannot inherit trust");
}
void free_checks() {
    predict::Tracker tracker;
    auto tp = predict::Tracker::TP::clock::now();
    Eigen::Vector4d z(3, 0, .1, 0);
    const auto &target = tracker.reset_target(z, tp);
    require(target.tracked_state(8) == .26, "free mode preserves original initial radius");
    require(target.updating_model_type == UpdatingModelType::ARMOR_MODEL, "free model selection unchanged");
    require(tracker.whole_state_ekf.get_P().isIdentity(), "free covariance remains unconstrained");
}
}
int main() {
    try {
        Log console; screen = &console;
        config_checks();
        if (target_model::config().fixed()) {
            for (double omega : {0., .35, 1., 4., -4.}) rotating(omega, .02);
            rotating(4, .1);
        } else free_checks();
        std::cout << "PASS " << checks << " target-model checks; no hardware opened\n";
        return 0;
    } catch (const std::exception &error) { std::cerr << "FAIL " << error.what() << '\n'; return 1; }
}
