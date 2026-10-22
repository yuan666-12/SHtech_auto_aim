// Short detection gaps must preserve a learned vehicle model; no hardware is opened.
#include "Tracker.hpp"
#include "MultiPolicyPredictor_submodule.hpp"
#include "planner_submodule.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

using TP = predict::Tracker::TP;
int checks = 0;
void require(bool ok, const char *message) {
    ++checks;
    if (!ok) throw std::runtime_error(message);
}
struct Motion {
    double omega = 1.0;
    double radius = .26;
    double vx = 0;
    int armor(double t) const {
        int best = 0;
        double score = -2;
        for (int j = 0; j < 4; ++j) {
            double candidate = std::cos(omega * t + j * M_PI_2);
            if (candidate > score) { score = candidate; best = j; }
        }
        return best;
    }
    Eigen::Vector4d observation(double t) const {
        double a = mathutils::limit_rad(omega * t + armor(t) * M_PI_2);
        return {3.26 - radius * std::cos(a), vx * t - radius * std::sin(a), .1, a};
    }
};
TP stamp(TP start, double seconds) {
    return start + std::chrono::duration_cast<TP::duration>(std::chrono::duration<double>(seconds));
}
const char *state(TrackingState value) {
    switch (value) {
    case TrackingState::IDLE: return "IDLE";
    case TrackingState::DETECTING: return "DETECTING";
    case TrackingState::TRACKING: return "TRACKING";
    case TrackingState::TEMP_LOST: return "TEMP_LOST";
    }
    return "unknown";
}
void mark(const std::string &name) {
    std::fprintf(logfile->out, "CASE %s\n", name.c_str());
}
void warm(predict::Tracker &tracker, const Motion &motion, TP start, double dt) {
    for (int k = 0; k <= int(std::round(8 / dt)); ++k) {
        auto tp = stamp(start, k * dt);
        auto z = motion.observation(k * dt);
        if (k) tracker.track(z, z, 1, 2, tp, 0);
        else tracker.reset_target(z, tp);
    }
    require(tracker.get_target().vehicle_model_trust, "warm-up must earn real rotation trust");
}
void single_miss(int fps) {
    mark("single_miss_" + std::to_string(fps));
    predict::Tracker tracker;
    Motion motion;
    const double dt = 1.0 / fps;
    auto start = TP::clock::now() - std::chrono::seconds(20);
    warm(tracker, motion, start, dt);
    auto before = tracker.get_target();
    auto z = motion.observation(8);
    tracker.track(z, z, 0, 2, stamp(start, 8 + dt), 0);
    auto lost = tracker.get_target();
    z = motion.observation(8 + 2 * dt);
    tracker.track(z, z, 1, 2, stamp(start, 8 + 2 * dt), 0);
    auto recovered = tracker.get_target();
    require(lost.predictor_state == TrackingState::TEMP_LOST && lost.vehicle_model_trust,
            "missing frame itself temporarily retains a trusted model");
    require(!recovered.filter_reset && recovered.vehicle_model_trust &&
                recovered.vehicle_model_rotation_count == before.vehicle_model_rotation_count &&
                recovered.predictor_state == TrackingState::TRACKING,
            "accepted same-target reacquisition must preserve the learned rotation model");
    require(std::abs(recovered.tracked_state[7] - motion.omega) < .15,
            "angular velocity survives the missed frame");
    std::cout << "RESULT {\"case\":\"single_miss\",\"fps\":" << fps
              << ",\"accepted_observation_gap_ms\":" << 2000 * dt
              << ",\"before_count\":" << before.vehicle_model_rotation_count
              << ",\"before_trust\":" << before.vehicle_model_trust
              << ",\"lost_state\":\"" << state(lost.predictor_state)
              << "\",\"lost_trust\":" << lost.vehicle_model_trust
              << ",\"recovered_state\":\"" << state(recovered.predictor_state)
              << "\",\"recovered_count\":" << recovered.vehicle_model_rotation_count
              << ",\"recovered_trust\":" << recovered.vehicle_model_trust
              << ",\"filter_reset\":" << recovered.filter_reset
              << ",\"omega_before\":" << before.tracked_state[7]
              << ",\"omega_after\":" << recovered.tracked_state[7] << "}\n";
}
void skipped_packet_and_timeout() {
    predict::Tracker tracker;
    Motion motion;
    auto start = TP::clock::now() - std::chrono::seconds(20);
    mark("skipped_packet_50fps");
    warm(tracker, motion, start, .02);
    auto z = motion.observation(8.04);
    tracker.track(z, z, 1, 2, stamp(start, 8.04), 0);
    auto skipped = tracker.get_target();
    require(skipped.vehicle_model_trust && !skipped.filter_reset,
            "a 40 ms interval with no zero-detection call does not reset the model");
    std::cout << "RESULT {\"case\":\"skipped_packet\",\"gap_ms\":40,\"trust\":"
              << skipped.vehicle_model_trust << ",\"count\":" << skipped.vehicle_model_rotation_count
              << ",\"filter_reset\":" << skipped.filter_reset << "}\n";
    mark("actual_timeout");
    tracker.track(z, z, 0, 2, stamp(start, 8.241), 0);
    auto expired = tracker.get_target();
    require(expired.predictor_state == TrackingState::IDLE && !expired.vehicle_model_trust,
            "a genuine 201 ms visual gap also expires the model");
    std::cout << "RESULT {\"case\":\"actual_timeout\",\"gap_ms\":201,\"state\":\""
              << state(expired.predictor_state) << "\",\"trust\":" << expired.vehicle_model_trust << "}\n";
}
void center_reset() {
    mark("learned_center_reset");
    predict::Tracker tracker;
    Motion motion{1, .32, .03};
    auto start = TP::clock::now() - std::chrono::seconds(20);
    warm(tracker, motion, start, .02);
    auto before = tracker.get_target();
    auto z = motion.observation(8);
    tracker.track(z, z, 0, 2, stamp(start, 8.02), 0);
    z = motion.observation(8.04);
    tracker.track(z, z, 1, 2, stamp(start, 8.04), 0);
    auto after = tracker.get_target();
    double before_error = std::hypot(before.tracked_state[0] - 3.26, before.tracked_state[2] - .03 * 8);
    double after_error = std::hypot(after.tracked_state[0] - 3.26, after.tracked_state[2] - .03 * 8.04);
    require(before_error < .02, "pre-loss center must actually converge");
    require(!after.filter_reset && after.vehicle_model_trust,
            "the learned center remains trusted after a short gap");
    require(after_error < .02 && std::abs(after.tracked_state[8] - .32) < .015 &&
                std::abs(after.tracked_state[3] - .03) < .02,
            "center, radius and translation velocity must remain learned");
    std::cout << "RESULT {\"case\":\"learned_center_reset\",\"true_radius_m\":0.32,\"radius_before\":"
              << before.tracked_state[8] << ",\"radius_after\":" << after.tracked_state[8]
              << ",\"center_error_before_mm\":" << before_error * 1000
              << ",\"center_error_after_mm\":" << after_error * 1000
              << ",\"vx_before\":" << before.tracked_state[3]
              << ",\"vx_after\":" << after.tracked_state[3] << "}\n";
}
void recurring_misses(int every, double omega) {
    mark("recurring_misses_" + std::to_string(every) + "_omega_" + std::to_string(omega));
    predict::Tracker tracker;
    Motion motion{omega};
    auto start = TP::clock::now() - std::chrono::seconds(20);
    int resets = 0, trusted = 0, maximum_count = 0, lost_frames = 0;
    Eigen::Vector4d previous = motion.observation(0);
    for (int k = 0; k <= 600; ++k) {
        double t = .02 * k;
        auto tp = stamp(start, t);
        auto z = motion.observation(t);
        bool missing = every && k && k % every == 0;
        bool called = true;
        if (!k || (tracker.get_tracker_state() == TrackingState::IDLE && !missing)) {
            // Mirror MultiPolicyPredictor::update_tracker: IDLE reacquires
            // with reset_target; track() alone does not leave IDLE.
            tracker.reset_target(z, tp);
        } else if (tracker.get_tracker_state() != TrackingState::IDLE) {
            tracker.track(missing ? previous : z, z, missing ? 0 : 1, 2, tp, 0);
        } else {
            called = false;
        }
        if (!missing) previous = z;
        const auto &target = tracker.get_target();
        resets += called && target.filter_reset;
        trusted += target.vehicle_model_trust && target.predictor_state == TrackingState::TRACKING;
        lost_frames += missing;
        maximum_count = std::max(maximum_count, target.vehicle_model_rotation_count);
    }
    require(trusted > 0 && maximum_count == 3 && resets == 1,
            "recurring isolated losses must still build trust without full reinitialization");
    std::cout << "RESULT {\"case\":\"recurring_misses\",\"omega\":" << omega
              << ",\"fps\":50,\"duration_s\":12,\"drop_every\":"
              << every << ",\"lost_frames\":" << lost_frames << ",\"resets\":" << resets
              << ",\"trusted_tracking_frames\":" << trusted << ",\"maximum_switch_count\":" << maximum_count << "}\n";
}
bbox_t projected_armor(const std::string &root, const Motion &motion, double t) {
    auto z = motion.observation(t);
    cv::FileStorage file(root + "/asset/camParam/cam_param_hero.yml", cv::FileStorage::READ);
    cv::Mat rotation_cv, intrinsic, distortion;
    file["R_c2i"] >> rotation_cv;
    file["K"] >> intrinsic;
    file["D"] >> distortion;
    Eigen::Matrix3d camera_to_imu, base;
    cv::cv2eigen(rotation_cv, camera_to_imu);
    base << 1, 0, 0, 0, 0, 1, 0, -1, 0;
    Eigen::Matrix3d world_armor = Eigen::AngleAxisd(-z[3], Eigen::Vector3d::UnitZ()).toRotationMatrix() * base;
    const Attitude attitude;
    Eigen::Matrix3d rotation = camera_to_imu.transpose() * attitude.R_world2imu() * world_armor;
    Eigen::Vector3d center(z[1], z[0], z[2]);
    auto camera = mathutils::CoordTransformer::Get().pw_to_pc(center, attitude.R_world2imu());
    cv::Mat matrix, rvec, tvec;
    cv::eigen2cv(rotation, matrix);
    cv::Rodrigues(matrix, rvec);
    cv::eigen2cv(camera, tvec);
    std::vector<cv::Point2d> pixels;
    cv::projectPoints(mathutils::pw_blue_small, rvec, tvec, intrinsic, distortion, pixels);
    bbox_t box{};
    for (int i = 0; i < 4; ++i) box.pts[i] = pixels[i];
    box.tag_id = 2;
    box.color_id = 1;
    box.confidence = .9;
    box.source = DetectionSource::NEURAL_NETWORK;
    return box;
}
void predictor_miss(const std::string &root) {
    mark("projected_corners_predictor_single_miss");
    mathutils::CoordTransformer::Init(root + "/asset/camParam/cam_param_hero.yml", false);
    predict::MultiPolicyPredictorSubModule predictor(predict::MultiPolicyPredictorConfig{});
    Motion motion;
    auto start = TP::clock::now() - std::chrono::seconds(20);
    Target before, missing, recovered;
    for (int k = 0; k <= 402; ++k) {
        auto data = std::make_shared<ThreadDataPack>();
        data->frame = cv::Mat::zeros(1080, 1440, CV_8UC3);
        data->attitude_valid = true;
        data->time = stamp(start, k * .02);
        data->robotstatus.enemy_color = EnemyColor::BLUE;
        data->robotstatus.program_mode = ProgramMode::AUTO_AIM;
        if (k != 401) data->bboxes.push_back(projected_armor(root, motion, k * .02));
        predictor.process(data, nullptr);
        if (k == 400) before = data->target;
        if (k == 401) missing = data->target;
        if (k == 402) recovered = data->target;
    }
    std::cout << "RESULT {\"case\":\"predictor_projected_corners\",\"fps\":50,\"gap_ms\":40,\"before_trust\":"
              << before.vehicle_model_trust << ",\"before_count\":" << before.vehicle_model_rotation_count
              << ",\"lost_state\":\"" << state(missing.predictor_state)
              << "\",\"recovered_state\":\"" << state(recovered.predictor_state)
              << "\",\"recovered_trust\":" << recovered.vehicle_model_trust
              << ",\"recovered_count\":" << recovered.vehicle_model_rotation_count
              << ",\"filter_reset\":" << recovered.filter_reset << "}\n";
    require(before.vehicle_model_trust, "projected pixels must build real predictor model before omission");
    require(missing.predictor_state == TrackingState::TEMP_LOST && !recovered.filter_reset &&
                recovered.vehicle_model_trust && recovered.vehicle_model_rotation_count == 3 &&
                recovered.predictor_state == TrackingState::TRACKING,
            "PnP and predictor association preserve the same model across an empty detection list");
    mathutils::CoordTransformer::Destroy();
}

void bounded_gaps_and_outliers() {
    for (int missed : {3, 8}) {
        predict::Tracker tracker;
        Motion motion;
        auto start = TP::clock::now() - std::chrono::seconds(20);
        warm(tracker, motion, start, .02);
        const auto seen = tracker.get_target().last_observation_time;
        const auto accepted = tracker.get_target().last_observation_update_time;
        auto z = motion.observation(8);
        for (int k = 1; k <= missed; ++k) {
            tracker.track(z, z, 0, 2, stamp(start, 8 + k * .02), 0);
            require(tracker.get_target().vehicle_model_trust &&
                        tracker.get_target().last_observation_time == seen &&
                        tracker.get_target().last_observation_update_time == accepted,
                    "predictions preserve the model without refreshing either observation clock");
        }
        double t = 8 + (missed + 1) * .02;
        z = motion.observation(t);
        tracker.track(z, z, 1, 2, stamp(start, t), 0);
        const auto &target = tracker.get_target();
        require(target.predictor_state == TrackingState::TRACKING && target.vehicle_model_trust &&
                    target.vehicle_model_rotation_count == 3 && !target.filter_reset,
                "multiple missed frames inside 200 ms do not erase rotation history");
    }
    predict::Tracker tracker;
    Motion motion;
    auto start = TP::clock::now() - std::chrono::seconds(20);
    warm(tracker, motion, start, .02);
    auto z = motion.observation(8);
    const auto seen = tracker.get_target().last_observation_time;
    tracker.track(z, z, 0, 2, stamp(start, 8.02), 0);
    auto spike = motion.observation(8.04);
    spike[0] += 2;
    tracker.track(spike, spike, 1, 2, stamp(start, 8.04), 0);
    require(tracker.get_target().measurement_rejected && !tracker.get_target().observation_updated &&
                tracker.get_target().last_observation_time == seen &&
                tracker.get_target().vehicle_model_rotation_count == 3,
            "one unrelated outlier cannot refresh freshness or erase the old model");
    z = motion.observation(8.06);
    tracker.track(z, z, 1, 2, stamp(start, 8.06), 0);
    require(tracker.get_target().vehicle_model_trust && !tracker.get_target().filter_reset &&
                tracker.get_target().predictor_state == TrackingState::TRACKING,
            "the original plate recovers after one rejected outlier");
    tracker.track(z, z, 0, 2, stamp(start, 8.261), 0);
    require(tracker.get_target().predictor_state == TrackingState::IDLE &&
                !tracker.get_target().vehicle_model_trust && tracker.get_target().vehicle_model_rotation_count == 0,
            "a genuine 201 ms visual gap still expires trust and history");
    auto tp = stamp(start, 8.281);
    z = motion.observation(8.281);
    tracker.reset_target(z, tp);
    require(!tracker.get_target().vehicle_model_trust && tracker.get_target().vehicle_model_rotation_count == 0,
            "a new acquisition after expiry cannot inherit the previous vehicle model");
}


void planner_loss_inhibits_fire(const std::string &root) {
    mathutils::CoordTransformer::Init(root + "/asset/camParam/cam_param_hero.yml", false);
    predict::Tracker tracker;
    Motion motion;
    auto start = TP::clock::now() - std::chrono::milliseconds(8100);
    warm(tracker, motion, start, .02);
    pipeline::bridge::PlannerToSerialBridge bridge;
    bool eligible = false;
    bridge.set_receiver([&](const pipeline::bridge::PlannerToSerialMessage &message) {
        eligible = message.fire_control_eligible;
    });
    plan::PlannerConfig config;
    config.debug.show_image = config.debug.log_text = false;
    plan::PlannerSubModule planner(config, bridge, root + "/asset/plannerParam/planner_param_hero.yml");
    auto data = std::make_shared<ThreadDataPack>();
    data->attitude_valid = data->has_fixed_target = true;
    data->robotstatus.robot_speed_mps = 18.5;
    data->attitude = Attitude(0, 10, 0);
    auto publish = [&](double t) {
        data->target = tracker.get_target();
        data->target.tracked_armor.tag_id = 2;
        data->time = stamp(start, t);
        planner.process(data, nullptr);
    };
    publish(8);
    require(eligible, "fresh trusted observations reach the production planner gate");
    auto z = motion.observation(8);
    tracker.track(z, z, 0, 2, stamp(start, 8.02), 0);
    publish(8.02);
    require(data->target.vehicle_model_trust && !eligible && data->robotcommand.fire_enable == 0,
            "retaining a trusted model during loss must not retain permission to fire");
    z = motion.observation(8.04);
    tracker.track(z, z, 1, 2, stamp(start, 8.04), 0);
    publish(8.04);
    require(eligible && !data->target.filter_reset && data->target.vehicle_model_rotation_count == 3,
            "accepted reacquisition restores eligibility through the normal planner checks");
    mathutils::CoordTransformer::Destroy();
}

int main(int argc, char **argv) {
    try {
        require(argc == 2, "project root required");
        setenv("AUTOAIM_HEADLESS", "1", 1);
        logfile->reopen("tracker_diagnostics.log");
        std::cout << std::fixed << std::setprecision(6);
        for (int fps : {100, 50, 30, 20}) single_miss(fps);
        skipped_packet_and_timeout();
        bounded_gaps_and_outliers();
        center_reset();
        recurring_misses(0, 1);
        recurring_misses(10, 1);
        recurring_misses(0, 4);
        recurring_misses(10, 4);
        predictor_miss(argv[1]);
        planner_loss_inhibits_fire(argv[1]);
        std::cout << "PASS " << checks << " dropout/model checks; no camera/UART opened\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
