#include "common.hpp"
#include "planner_submodule.hpp"
#include <cstdlib>
#include <iostream>

using namespace std::chrono_literals;
int checks = 0;
void require(bool ok, const char *message) {
    ++checks;
    if (!ok) throw std::runtime_error(message);
}

void check_interval(const std::string &root, const char *setting, double expected) {
    if (setting) setenv("AUTOAIM_SHOOT_INTERVAL_S", setting, 1);
    else unsetenv("AUTOAIM_SHOOT_INTERVAL_S");
    pipeline::bridge::PlannerToSerialBridge bridge;
    pipeline::bridge::PlannerToSerialMessage captured{};
    bridge.set_receiver([&](const auto &message) { captured = message; });
    plan::PlannerConfig config;
    config.debug.show_image = false;
    config.debug.log_text = false;
    plan::PlannerSubModule planner(config, bridge, root + "/tools/fixtures/planner_raw_rx.yml");
    auto data = std::make_shared<ThreadDataPack>();
    data->has_fixed_target = true;
    data->attitude_valid = true;
    data->robotstatus.robot_speed_mps = 15;
    data->attitude = Attitude(0, 10, 0);
    data->time = std::chrono::high_resolution_clock::now();
    auto &target = data->target;
    target.predictor_state = TrackingState::TRACKING;
    target.vehicle_model_trust = true;
    target.observation_updated = true;
    target.last_observation_time = data->time;
    target.last_observation_update_time = center_fire::Clock::now();
    target.tracked_armor.tag_id = 2;
    target.tracked_measurement << 3, 0, .1, 0;
    target.armor_y_state << 3, 0;
    target.armor_z_state << .1, 0;
    target.tracked_state << 3.26, 0, 0, 0, .1, 0, 0, 0, .26, 0, 0;
    planner.process(data, nullptr);
    require(captured.center_window.valid && captured.fire_control_eligible,
            "real planner bridge publishes a valid firing snapshot");
    require(std::abs(captured.center_window.shot_interval_s - expected) < 1e-12,
            "startup interval reaches the TX snapshot");
    require(std::abs(captured.center_window.release_delay_s - .23) < 1e-8,
            "shot interval does not change release delay");
    if (expected > 1) return;
    const auto ray = mathutils::solve_ballistic_trajectory(Eigen::Vector3d(0, 3, .1), 15, true);
    const double pitch = ray.pitch * 180 / center_fire::pi;
    const auto now = center_fire::Clock::now() + 1s;
    auto snapshot = captured.center_window;
    snapshot.state_time = now;
    auto outside = snapshot;
    outside.state[2] = .5;
    const auto reentry = [&](double elapsed, const char *reason) {
        center_fire::Window window(center_fire::Config{});
        center_fire::Evaluation diagnostic;
        require(window.evaluate(snapshot, now, 0, pitch, 2), "first valid encounter fires");
        window.evaluate(outside, now + 5ms, 0, pitch, 2);
        window.evaluate(outside, now + 40ms, 0, pitch, 2);
        const auto at = now + std::chrono::duration_cast<center_fire::Clock::duration>(
            std::chrono::duration<double>(elapsed));
        const bool fired = window.evaluate(snapshot, at, 0, pitch, 2, &diagnostic);
        require(std::string(diagnostic.reason) == reason, "TX cooldown obeys the configured boundary");
        return fired;
    };
    require(!reentry(expected - .001, "shot_interval"), "reentry one millisecond early is blocked");
    require(reentry(expected + .001, "ready"), "new encounter after the interval can fire");
}

int main(int argc, char **argv) {
    try {
        require(argc > 1, "repository root is required");
        const std::string root = argv[1];
        Log console;
        screen = &console;
        mathutils::CoordTransformer::Init(root + "/asset/camParam/cam_param_hero.yml", false);
        check_interval(root, nullptr, .5);
        check_interval(root, "0.3", .3);
        check_interval(root, "0.8", .8);
        check_interval(root, "10", 10);
        for (const char *invalid : {"", "0", "-0.1", "nan", "inf", "10.001", "0.3s", "1e9999", "1e-9999"}) {
            setenv("AUTOAIM_SHOOT_INTERVAL_S", invalid, 1);
            bool rejected = false;
            try {
                predict::Planner planner(root + "/tools/fixtures/planner_raw_rx.yml", false);
            } catch (const std::invalid_argument &error) {
                rejected = std::string(error.what()).find("AUTOAIM_SHOOT_INTERVAL_S") != std::string::npos;
            }
            require(rejected, "invalid startup interval is explicitly rejected");
        }
        unsetenv("AUTOAIM_SHOOT_INTERVAL_S");
        std::cout << "PASS " << checks << " shot-interval checks; no camera/UART opened\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
