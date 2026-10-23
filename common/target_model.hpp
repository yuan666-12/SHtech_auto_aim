#pragma once
#include <array>
#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include <stdexcept>
#include <string_view>

// State order: cy, vy, cx, vx, cz, vz, phase, omega, r, dr, dz.
// Fixed geometry constrains the estimator itself, including its uncertainty.
namespace target_model {
enum class Mode { Free, ProvinceFixed };
struct Config {
    Mode mode = Mode::Free;
    double radius_m = 0.265;
    bool fixed() const { return mode == Mode::ProvinceFixed; }
    const char *name() const { return fixed() ? "province_fixed" : "free"; }
    double initial_radius_m() const { return fixed() ? radius_m : 0.26; }
    inline static constexpr std::array<int, 6> fixed_axes{1, 3, 5, 8, 9, 10};

    template <class State> void constrain_state(State &state) const {
        if (!fixed()) return;
        for (int index : fixed_axes) state(index) = 0.0;
        state(8) = radius_m;
    }
    template <class Matrix> void constrain_covariance(Matrix &matrix) const {
        if (!fixed()) return;
        for (int index : fixed_axes) {
            matrix.row(index).setZero();
            matrix.col(index).setZero();
        }
    }
    template <class Matrix> void constrain_process_noise(Matrix &matrix) const {
        if (!fixed()) return;
        constrain_covariance(matrix);
        // Stationary center: observations refine only its initial uncertainty.
        for (int index : {0, 2, 4}) {
            matrix.row(index).setZero();
            matrix.col(index).setZero();
        }
    }
    template <class Matrix> void constrain_observation(Matrix &matrix) const {
        if (!fixed()) return;
        for (int index : fixed_axes) matrix.col(index).setZero();
    }
};
inline Config parse(const char *mode, const char *radius) {
    Config result;
    if (!mode || !*mode || std::string_view(mode) == "free") return result;
    if (std::string_view(mode) != "province_fixed")
        throw std::invalid_argument("AUTOAIM_TARGET_MODEL must be free or province_fixed");
    result.mode = Mode::ProvinceFixed;
    if (radius && *radius) {
        char *end = nullptr;
        result.radius_m = std::strtod(radius, &end);
        if (end == radius || *end || !std::isfinite(result.radius_m) ||
            result.radius_m < 0.12 || result.radius_m > 0.4)
            throw std::invalid_argument("AUTOAIM_TARGET_RADIUS_M must be finite and within [0.12, 0.4] m");
    }
    return result;
}
inline const Config &config() {
    static const Config value = parse(std::getenv("AUTOAIM_TARGET_MODEL"), std::getenv("AUTOAIM_TARGET_RADIUS_M"));
    return value;
}
} // namespace target_model
