/**
 * @file MultiPolicyPredictor.cpp
 * @brief 多策略预测器实现 - 整合完整预测系统的主要实现
 * @author Cao Jingyan
 * @date 2025/11/21
 *
 * 实现功能：
 * 1. 预测流程的完整执行逻辑
 * 2. 装甲板筛选和目标匹配算法
 * 3. 多种可视化显示实现
 * 4. 机器人控制指令生成
 */

#include "gui_control.hpp"
#include "MultiPolicyPredictor_submodule.hpp"
#include <cstdlib>
#include "armor_policy.hpp"
#include <cstdio>
#include <cstring>
#include <cmath>

namespace predict {
    struct FrameTrackerDiagnostic {
        const char *action = "not_run";
        int matching_labels = 0, pnp_failed = 0, selected_count = 0;
    };
    // Scope-bound so invalid-attitude and initial-PnP early returns are also recorded.
    struct PredictorFrameDiagnostic {
        const ThreadDataPack &data;
        predict::Tracker *trackers;
        std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
        const char *outcome = "complete";
        bool filter_ran = false;
        int color_pass = 0, pnp_ok = 0, pnp_failed = 0, pnp_nonfinite = 0, geometry_pass = 0;
        int distance_rejected = 0, yaw_rejected = 0, height_rejected = 0;
        size_t new_candidates = 0, tracking_candidates = 0;
        std::array<FrameTrackerDiagnostic, NUM_TRACKER> slots{};
        PredictorFrameDiagnostic(const ThreadDataPack &frame, predict::Tracker *owner) : data(frame), trackers(owner) {}
        double stage_ms(SubModuleName name) const {
            const auto &times = data.submodule_timestamps[static_cast<size_t>(name)];
            if (times.first == std::chrono::steady_clock::time_point{} ||
                times.second == std::chrono::steady_clock::time_point{} || times.second < times.first)
                return NAN;
            return std::chrono::duration<double, std::milli>(times.second - times.first).count();
        }
        ~PredictorFrameDiagnostic() noexcept {
            if (!logfile || !logfile->out || logfile->out == stdout || logfile->out == stderr) return;
            const auto now = std::chrono::steady_clock::now();
            const auto image_ns = static_cast<long long>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(data.time.time_since_epoch()).count());
            const double image_age =
                data.time == std::chrono::high_resolution_clock::time_point{}
                    ? NAN
                    : std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - data.time)
                          .count();
            std::fprintf(
                logfile->out,
                "[DIAG_FRAME] wall_ms=%.3f image_ns=%lld frame_index=%d camera_frame=%llu "
                "camera_host_stamp=%lld camera_device_stamp=%llu outcome=%s detections=%zu filter_ran=%d "
                "color_pass=%d pnp_ok=%d pnp_failed=%d pnp_nonfinite=%d geometry_pass=%d "
                "distance_rejected=%d yaw_rejected=%d height_rejected=%d new_candidates=%zu "
                "tracking_candidates=%zu attitude_valid=%d attitude_age_ms=%.3f alignment_error_ms=%.3f "
                "capture_ready_wait_ms=%.3f image_age_ms=%.3f sensor_ms=%.3f preprocess_ms=%.3f "
                "detect_ms=%.3f corner_ms=%.3f predict_ms=%.3f has_fixed_target=%d "
                "state=%d trusted=%d count=%d\n",
                std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count(),
                image_ns, data.index, static_cast<unsigned long long>(data.camera_frame_number),
                static_cast<long long>(data.camera_host_timestamp),
                static_cast<unsigned long long>(data.camera_device_timestamp), outcome, data.bboxes.size(),
                int(filter_ran), color_pass, pnp_ok, pnp_failed, pnp_nonfinite, geometry_pass, distance_rejected,
                yaw_rejected, height_rejected, new_candidates, tracking_candidates, int(data.attitude_valid),
                data.attitude_age_ms, data.attitude_alignment_error_ms, data.capture_ready_wait_ms, image_age,
                stage_ms(SubModuleName::SENSOR), stage_ms(SubModuleName::PREPROCESS), stage_ms(SubModuleName::DETECT),
                stage_ms(SubModuleName::CORNER_REFINE),
                std::chrono::duration<double, std::milli>(now - started).count(), int(data.has_fixed_target),
                static_cast<int>(data.target.predictor_state), int(data.target.vehicle_model_trust),
                data.target.vehicle_model_rotation_count);
            for (size_t i = 0; i < slots.size(); ++i) {
                const auto &slot = slots[i];
                if (std::strcmp(slot.action, "not_run") == 0) continue;
                const auto &t = trackers[i].get_target();
                std::fprintf(logfile->out,
                             "[DIAG_ASSOC] image_ns=%lld slot=%zu tracker=%p action=%s matching_labels=%d "
                             "pnp_failed=%d selected_count=%d state=%d trusted=%d count=%d\n",
                             image_ns, i, static_cast<const void *>(&trackers[i]), slot.action, slot.matching_labels,
                             slot.pnp_failed, slot.selected_count, static_cast<int>(t.predictor_state),
                             int(t.vehicle_model_trust), t.vehicle_model_rotation_count);
            }
        }
    };
} // namespace predict

namespace predict {
    /**
     * @brief 构造函数 - 初始化多策略预测器的所有组件
     * @param comm_latency_ 通信延迟时间 (毫秒)
     * @param shoot_latency_ 发射延迟时间 (毫秒)
     * @param debug_ 调试模式标志
     * @param show_ 显示模式标志
     * @param plot_ 绘图模式标志
     * @param adjust_ 参数调整模式标志
     * @details 初始化跟踪器和规划器，设置各种显示和调试选项
     *          CoordTransformer 已在 main 中初始化，直接使用单例
     */
    MultiPolicyPredictorSubModule::MultiPolicyPredictorSubModule(const MultiPolicyPredictorConfig &config)
        : SubModule(SubModuleName::MULTI_POLICY_PREDICTOR), config_(config), coord_transformer(CoordTransformer::Get()),
          autoaim_mode_counter(0), in_autoaim_mode(false), fixed_target_id(0) {
        for (int i = 0; i < NUM_TRACKER; ++i) {
            trackers[i].set_debug(config_.debug.log_text);
            trackers[i].set_adjust(config_.adjust_tracker_noise);
        }

        if (config_.adjust_mode) {
            autoaim_gui::namedWindow("predictor trackbar", cv::WINDOW_AUTOSIZE);

            autoaim_gui::createTrackbar("in_autoaim_mode", "predictor trackbar", &in_autoaim_mode_adjust, 1, 0);
        }

        LOGM_S("[MultiPolicyPredictorSubModule] ignore armor id: %s (enemy color retained)",
               armor_policy::ignore_armor_id() ? "ON, 135x56 mm small armor" : "OFF, original digit policy");
        LOGM_S("[MultiPolicyPredictorSubModule] construction completed");
    }

    SubModuleResult MultiPolicyPredictorSubModule::process(std::shared_ptr<ThreadDataPack> data,
                                                           const pipeline::BasicTask *parent) {
        auto t1 = std::chrono::steady_clock::now();

        // LOGM_S("[MultiPolicyPredictorSubModule] ready");

        // 执行预测算法
        predict(data);

        auto t2 = std::chrono::steady_clock::now();

        // 调试信息
        if (config_.debug.log_text) {
            // auto &send = data->robotcommand;
            // LOGM_S("[MultiPolicyPredictorSubModule] pitch %6.2f, yaw %6.2f, dist %4.1f",
            //        send.pitch_angle, send.yaw_angle,
            //        (float)send.distance / 10);
        }

        // 显示结果（如果需要）
        if (config_.debug.show_image) {
            // 预测模块的显示逻辑（如果需要的话）
        }

        auto t3 = std::chrono::steady_clock::now();
        // LOGM_S(
        //     "LinearPredictorSubModule Predict %.2lfms Show %.2lfms",
        //     std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count()*1000,
        //     std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count()*1000
        // );

        // 预测总是成功的，返回 true
        return SubModuleResult::SUCCESS;
    }

    /**
     * @brief 主预测函数 - 执行完整的预测流程
     * @param data 线程数据包，包含检测结果、传感器数据、图像等
     * @details 完整的预测流程：
     *          1. 更新坐标变换矩阵
     *          2. 根据跟踪状态执行不同的处理逻辑
     *          3. 装甲板筛选和目标匹配
     *          4. 执行跟踪和预测计算
     *          5. 生成控制指令
     *          6. 可视化显示（可选）
     */

    void MultiPolicyPredictorSubModule::update_autoaim_mode(const RobotStatus &status) {
        if (in_autoaim_mode) {
            autoaim_mode_counter =
                status.program_mode == ProgramMode::AUTO_AIM ? max_autoaim_mode_counter : autoaim_mode_counter - 1;
            if (autoaim_mode_counter <= 0) {
                in_autoaim_mode = false;
                autoaim_mode_counter = 0;
            }
        } else {
            autoaim_mode_counter = status.program_mode == ProgramMode::AUTO_AIM ? autoaim_mode_counter + 1 : 0;
            if (autoaim_mode_counter >= max_autoaim_mode_counter) {
                in_autoaim_mode = true;
                autoaim_mode_counter = max_autoaim_mode_counter;
            }
        }
        if (config_.adjust_mode) in_autoaim_mode = in_autoaim_mode_adjust > 0;
    }

    double MultiPolicyPredictorSubModule::armor_score(const bbox_t &armor, double distance) const {
        const auto center = points_center(armor.pts);
        const double screen_distance = std::hypot(center.x - cx, center.y - cy);
        return (1 - dist_weight) * screen_distance / maxSD + dist_weight * distance / max_distance_accept;
    }

    std::vector<MultiPolicyPredictorSubModule::ArmorObservation>
    MultiPolicyPredictorSubModule::collect_observations(const ThreadDataPack &data, const RobotStatus &status,
                                                        bool ignore_ids, PredictorFrameDiagnostic &diagnostic) {
        std::vector<ArmorObservation> observations;
        observations.reserve(data.bboxes.size());
        diagnostic.filter_ran = true;
        const auto rotation = data.attitude.R_world2imu();
        const float yaw = data.attitude.yaw() * M_PI / 180;
        for (auto armor : data.bboxes) {
            armor.tag_id = armor_policy::tracking_id(armor.tag_id, ignore_ids);
            if (armor.color_id != (status.enemy_color == EnemyColor::BLUE)) continue;
            ++diagnostic.color_pass;
            ArmorObservation observation;
            observation.armor = armor;
            float camera_yaw = 0;
            if (!coord_transformer.pnp_get_measurement(armor.pts, armor.tag_id, armor.color_id, yaw, rotation,
                                                       camera_yaw, observation.measurement)) {
                ++diagnostic.pnp_failed;
                continue;
            }
            if (!observation.measurement.allFinite() || !std::isfinite(camera_yaw)) {
                ++diagnostic.pnp_nonfinite;
                continue;
            }
            ++diagnostic.pnp_ok;
            observation.distance = observation.measurement.head<3>().norm();
            const bool distance_ok = observation.distance < max_distance_accept;
            const bool yaw_ok = std::abs(camera_yaw) < max_yaw_accept;
            const bool height_ok = std::abs(observation.measurement(2)) < max_height_accept;
            diagnostic.distance_rejected += !distance_ok;
            diagnostic.yaw_rejected += !yaw_ok;
            diagnostic.height_rejected += !height_ok;
            if (!distance_ok || !yaw_ok || !height_ok) continue;
            ++diagnostic.geometry_pass;
            observations.push_back(observation);
        }
        return observations;
    }

    void MultiPolicyPredictorSubModule::partition_observations(const std::vector<ArmorObservation> &observations,
                                                               std::vector<ArmorObservation> &fresh,
                                                               std::vector<ArmorObservation> &tracked) {
        const bool need_new = std::any_of(trackers.begin(), trackers.end(), [](auto &tracker) {
            return tracker.get_tracker_state() == TrackingState::IDLE;
        });
        for (const auto &observation : observations) {
            bool already_tracked = false;
            for (int i = 0; i < NUM_TRACKER; ++i) {
                if (trackers[i].get_tracker_state() != TrackingState::IDLE &&
                    observation.armor.tag_id == tracked_armors[i].tag_id) {
                    already_tracked = true;
                    break;
                }
            }
            if (already_tracked)
                tracked.push_back(observation);
            else if (need_new && (observation.armor.source == DetectionSource::TRADITIONAL ||
                                  observation.armor.source == DetectionSource::NEURAL_NETWORK))
                fresh.push_back(observation);
        }
        std::stable_sort(fresh.begin(), fresh.end(), [this](const auto &a, const auto &b) {
            return armor_score(a.armor, a.distance) < armor_score(b.armor, b.distance);
        });
    }

    void MultiPolicyPredictorSubModule::update_tracker(int index, std::vector<ArmorObservation> &fresh,
                                                       const std::vector<ArmorObservation> &observations,
                                                       bool ignore_ids, TP time, double attitude_yaw,
                                                       FrameTrackerDiagnostic &diagnostic) {
        auto &tracker = trackers[index];
        auto &armor = tracked_armors[index];
        auto &measurement = tracked_measurements[index];
        auto &secondary = secondary_tracked_measurements[index];
        diagnostic.action = tracker.get_tracker_state() == TrackingState::IDLE ? "idle_no_candidate" : "track";
        if (tracker.get_tracker_state() == TrackingState::IDLE) {
            if (fresh.empty()) return;
            armor = fresh.front().armor;
            measurement = fresh.front().measurement;
            fresh.erase(std::remove_if(fresh.begin(), fresh.end(),
                                       [&](const auto &candidate) { return candidate.armor.tag_id == armor.tag_id; }),
                        fresh.end());
            diagnostic.action = "acquire";
            diagnostic.selected_count = 1;
            tracker.reset_target(measurement, time);
            return;
        }
        const ArmorObservation *selected = nullptr;
        const ArmorObservation *alternate = nullptr;
        double nearest = DBL_MAX;
        int count = 0;
        for (const auto &candidate : observations) {
            if (candidate.armor.tag_id != armor.tag_id || candidate.armor.color_id != armor.color_id) continue;
            ++diagnostic.matching_labels;
            const double distance = (measurement.head<3>() - candidate.measurement.head<3>()).norm();
            if (ignore_ids) {
                if (distance < nearest) {
                    selected = &candidate;
                    nearest = distance;
                    count = 1;
                }
                continue;
            }
            ++count;
            if (distance < nearest) {
                alternate = selected;
                selected = &candidate;
                nearest = distance;
            } else
                alternate = &candidate;
            if (count == 2) break;
        }
        diagnostic.selected_count = count;
        if (count == 2 && selected->armor.source != DetectionSource::TRADITIONAL &&
            alternate->armor.source == DetectionSource::TRADITIONAL)
            std::swap(selected, alternate);
        if (selected) {
            armor = selected->armor;
            measurement = selected->measurement;
            secondary = alternate ? alternate->measurement : Eigen::Vector4d::Zero();
        }
        tracker.track(measurement, secondary, count, armor.tag_id, time, attitude_yaw);
    }

    void MultiPolicyPredictorSubModule::copy_target(Target &output, int index) {
        auto &tracker = trackers[index];
        output = tracker.get_target();
        output.tracked_armor = tracked_armors[index];
        const int plate = tracker.match_armor_id(output.tracked_measurement);
        const auto position = tracker.h_armor_xyz(output.tracked_state, plate);
        output.estimated_armor_m << position, mathutils::limit_rad(output.tracked_state(6) + plate * M_PI_2);
    }

    void MultiPolicyPredictorSubModule::publish_target(ThreadDataPack &data, double attitude_yaw,
                                                       const Eigen::Matrix3d &rotation) {
        if (in_autoaim_mode && target_locked_ && trackers[fixed_target_id].get_tracker_state() != TrackingState::IDLE) {
            copy_target(data.target, fixed_target_id);
            data.has_fixed_target = true;
            return;
        }
        const int selected = reset_target_for_plan(data.target, attitude_yaw, rotation);
        fixed_target_id = selected < 0 ? 0 : selected;
        target_locked_ = in_autoaim_mode && selected >= 0;
        data.has_fixed_target = target_locked_;
    }

    void MultiPolicyPredictorSubModule::predict(std::shared_ptr<ThreadDataPack> data) {
        PredictorFrameDiagnostic diagnostic(*data, trackers.data());
        if (!data->attitude_valid) {
            diagnostic.outcome = "invalid_attitude";
            data->target = Target{};
            data->has_fixed_target = target_locked_ = false;
            fixed_target_id = 0;
            return;
        }
        cx = data->frame.cols * 0.5;
        cy = data->frame.rows * 0.5;
        maxSD = std::max(1.0, std::hypot(cx, cy));
        auto status = data->robotstatus;
        const char *test_blue = std::getenv("AUTOAIM_TEST_ENEMY_BLUE");
        if (test_blue && test_blue[0] == '1') status.enemy_color = EnemyColor::BLUE;
        update_autoaim_mode(status);
        const bool ignore_ids = armor_policy::ignore_armor_id();
        const auto observations = collect_observations(*data, status, ignore_ids, diagnostic);
        std::vector<ArmorObservation> fresh, tracked;
        fresh.reserve(observations.size());
        tracked.reserve(observations.size());
        partition_observations(observations, fresh, tracked);
        diagnostic.new_candidates = fresh.size();
        diagnostic.tracking_candidates = tracked.size();
        const double yaw = data->attitude.yaw() * M_PI / 180;
        for (int i = 0; i < NUM_TRACKER; ++i)
            update_tracker(i, fresh, tracked, ignore_ids, data->time, yaw, diagnostic.slots[i]);
        publish_target(*data, yaw, data->attitude.R_world2imu());
    }

    int MultiPolicyPredictorSubModule::select_target_id(double, const Eigen::Matrix3d &) {
        int selected = -1;
        double best_score = DBL_MAX;
        for (int i = 0; i < NUM_TRACKER; ++i) {
            if (trackers[i].get_tracker_state() == TrackingState::IDLE) continue;
            const double distance = trackers[i].get_target().tracked_measurement.head<3>().norm();
            const double score = armor_score(tracked_armors[i], distance);
            if (score < best_score) {
                selected = i;
                best_score = score;
            }
        }
        return selected;
    }

    int MultiPolicyPredictorSubModule::reset_target_for_plan(Target &target, double attitude_yaw,
                                                             const Eigen::Matrix3d &rotation) {
        const int selected = select_target_id(attitude_yaw, rotation);
        if (selected >= 0)
            copy_target(target, selected);
        else
            target = trackers[0].get_target();
        return selected;
    }
} // namespace predict
