/**
 * @file Tracker.cpp
 * @brief 目标跟踪器实现 - 多模型自适应跟踪算法的具体实现
 * @author Cao Jingyan
 * @date 2025/11/21
 *
 * 实现功能：
 * 1. 多滤波器初始化和参数设置
 * 2. 跟踪状态机逻辑
 * 3. 模型自适应切换策略
 * 4. 异常检测和恢复机制
 */

#include "gui_control.hpp"
#include "Tracker.hpp"
#include <cstdio>

namespace predict {
    using DiagnosticTP = std::chrono::high_resolution_clock::time_point;
    // File-only, buffered diagnostics. No device access or control-state writes.
    struct TrackerDiagnostic {
        const void *tracker;
        const Target &target;
        DiagnosticTP image_time;
        std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
        Eigen::Vector4d input = Eigen::Vector4d::Zero();
        int input_count, state_before, count_before, trust_before, model_before;
        double observation_gap_ms, update_age_before_ms, frame_dt_ms;
        const char *kind;
        const char *gate = "not_called";
        const char *reject_reason = "none";
        const char *reset_reason = "none";
        bool armor_switched = false;
        TrackerDiagnostic(const void *owner, const Target &t, const Eigen::Vector4d &z, DiagnosticTP tp,
                          DiagnosticTP previous_tp, int count, const char *operation)
            : tracker(owner), target(t), image_time(tp), input_count(count),
              state_before(static_cast<int>(t.predictor_state)), count_before(t.vehicle_model_rotation_count),
              trust_before(t.vehicle_model_trust), model_before(static_cast<int>(t.updating_model_type)),
              observation_gap_ms(tracking_policy::observation_age(t.last_observation_time, tp) * 1000.0),
              update_age_before_ms(
                  t.last_observation_update_time == std::chrono::steady_clock::time_point{}
                      ? INFINITY
                      : std::chrono::duration<double, std::milli>(started - t.last_observation_update_time).count()),
              frame_dt_ms(std::chrono::duration<double, std::milli>(tp - previous_tp).count()), kind(operation) {
            if (count > 0) input = z;
        }
        ~TrackerDiagnostic() noexcept {
            if (!logfile || !logfile->out || logfile->out == stdout || logfile->out == stderr) return;
            const auto now = std::chrono::steady_clock::now();
            const double update_age =
                target.last_observation_update_time == std::chrono::steady_clock::time_point{}
                    ? INFINITY
                    : std::chrono::duration<double, std::milli>(now - target.last_observation_update_time).count();
            const bool finite = target.tracked_state.allFinite() && target.yaw_state.allFinite() &&
                                target.armor_x_state.allFinite() && target.armor_y_state.allFinite() &&
                                target.armor_z_state.allFinite();
            std::fprintf(
                logfile->out,
                "[DIAG_TRACK] wall_ms=%.3f image_ns=%lld tracker=%p kind=%s input_count=%d "
                "gate=%s reject_reason=%s reset_reason=%s state_before=%d state=%d "
                "count_before=%d count=%d trust_before=%d trusted=%d model_before=%d model=%d "
                "observed=%d rejected=%d filter_reset=%d armor_switched=%d finite=%d "
                "frame_dt_ms=%.3f obs_gap_ms=%.3f update_age_before_ms=%.3f update_age_ms=%.3f "
                "track_ms=%.3f input_y=%.6f input_x=%.6f input_z=%.6f input_yaw=%.6f\n",
                std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count(),
                static_cast<long long>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(image_time.time_since_epoch()).count()),
                tracker, kind, input_count, gate, reject_reason, reset_reason, state_before,
                static_cast<int>(target.predictor_state), count_before, target.vehicle_model_rotation_count,
                trust_before, int(target.vehicle_model_trust), model_before,
                static_cast<int>(target.updating_model_type), int(target.observation_updated),
                int(target.measurement_rejected), int(target.filter_reset), int(armor_switched), int(finite),
                frame_dt_ms, observation_gap_ms, update_age_before_ms, update_age,
                std::chrono::duration<double, std::milli>(now - started).count(), input[0], input[1], input[2],
                input[3]);
        }
    };
} // namespace predict

namespace predict {
    /**
     * @brief 构造函数 - 初始化跟踪器的所有组件
     * @param debug_ 调试模式标志
     * @param adjust_ 参数调整模式标志
     * @details 设置初始状态，初始化滤波器，配置参数调整界面（如果需要）
     */
    Tracker::Tracker()
        : last_tp(std::chrono::high_resolution_clock::now()), detecting_counter(0), temp_lost_counter(0),
          yaw_speed_diverge_counter(0) {
        // 初始化目标状态
        target_init();

        // 初始化装甲板模型的卡尔曼滤波器组
        armor_state_kf_init();

        // 初始化整车模型的扩展卡尔曼滤波器
        whole_state_ekf_init();
    }

    void Tracker::set_debug(bool debug_) {
        debug = debug_;
    }

    void Tracker::set_adjust(bool adjust_) {
        adjust = adjust_;

        if (adjust) {
            parameter_adjustor_init();
        }
    }

    /**
     * @brief 重置目标跟踪 - 用新检测到的目标重新初始化跟踪器
     * @param measurement 新的观测值 [y, x, z, yaw]
     * @param tp 当前时间戳
     * @return 重置后的目标状态引用
     * @details 重置所有滤波器状态，进入检测阶段
     */
    const Target &Tracker::reset_target(const Eigen::Matrix<double, 4, 1> &measurement, TP &tp) {
        TrackerDiagnostic diagnostic(this, target, measurement, tp, last_tp, 1, "reset_target");
        diagnostic.reset_reason = "explicit_reset_target";
        last_tp = tp;
        target.last_observation_time = tp;
        target.last_observation_update_time = std::chrono::steady_clock::now();
        target.observation_updated = true;
        target.measurement_rejected = false;
        target.filter_reset = true;
        target.raw_measurement = measurement;
        observation_gate.reset(measurement, tp);
        dt = 0.0;
        detecting_counter = 0;
        temp_lost_counter = 0;
        // 保存新的观测值
        target.tracked_measurement = measurement;

        // 重置所有滤波器到初始状态
        reset_whole_state_ekf();
        reset_yaw_kf();
        reset_armor_x_kf();
        reset_armor_y_kf();
        reset_armor_z_kf();

        // 进入检测阶段
        detecting_counter++;
        target.predictor_state = TrackingState::DETECTING;

        // 初始使用装甲板模型
        target.updating_model_type = UpdatingModelType::ARMOR_MODEL;

        target.vehicle_model_rotation_count = 0;
        target.vehicle_model_trust = false;

        return target;
    }

    /**
     * @brief 获取当前目标状态
     * @return 目标状态结构体的常引用
     */
    const Target &Tracker::get_target() {
        return target;
    }

    /**
     * @brief 执行目标跟踪更新 - 核心跟踪算法
     * @param measurement 当前观测值 [y, x, z, yaw]
     * @param secondary_measurement 当前备选观测值 [y, x, z, yaw]
     * @param same_id_armor_count 检测到的同ID装甲板数量
     * @param tp 当前时间戳
     * @param attitude_yaw 机器人当前姿态偏航角
     * @return 更新后的目标状态引用
     * @details 执行完整的跟踪流程：预测、更新、模型选择、异常检测
     */
    bool Tracker::process_observation(const Eigen::Vector4d &measurement, const TP &tp, int &same_id_armor_count,
                                      bool &confirmed_pending_switch, TrackerDiagnostic &diagnostic) {
        const bool previous_rejection = target.measurement_rejected;
        target.observation_updated = false;
        target.measurement_rejected = false;
        target.filter_reset = false;
        if (same_id_armor_count > 0) {
            target.raw_measurement = measurement;
            const double age = tracking_policy::observation_age(target.last_observation_time, tp);
            if (age > tracking_policy::observation_timeout_s && !previous_rejection) {
                diagnostic.reset_reason = "observation_timeout_on_arrival";
                TP reset_time = tp;
                reset_target(measurement, reset_time);
                return false;
            }
            if (!observation_gate.consider(measurement, tp)) {
                diagnostic.gate = "rejected";
                diagnostic.reject_reason = !measurement.allFinite() ? "nonfinite_measurement"
                                                                    : (age <= 0.0 ? "nonincreasing_measurement_time"
                                                                                  : "unconfirmed_position_jump");
                pending_switch_ =
                    measurement.allFinite() &&
                    (measurement.head<3>() - target.tracked_measurement.head<3>()).norm() > same_position_threshold &&
                    std::abs(mathutils::limit_rad(measurement(3) - target.tracked_measurement(3))) >= M_PI_4;
                if (pending_switch_) {
                    pending_switch_measurement_ = measurement;
                    pending_switch_time_ = tp;
                }
                same_id_armor_count = 0;
                target.measurement_rejected = true;
            } else if (target.predictor_state == TrackingState::TEMP_LOST && !previous_rejection) {
                diagnostic.gate = "accepted";
                diagnostic.reset_reason = "reacquired_after_temp_lost";
                TP reset_time = tp;
                reset_target(measurement, reset_time);
                return false;
            } else {
                diagnostic.gate = "accepted";
                const double pending_age = tracking_policy::observation_age(pending_switch_time_, tp);
                confirmed_pending_switch =
                    pending_switch_ && pending_age > 0 && pending_age <= tracking_policy::observation_timeout_s &&
                    (measurement.head<3>() - pending_switch_measurement_.head<3>()).norm() <= same_position_threshold &&
                    std::abs(mathutils::limit_rad(measurement(3) - pending_switch_measurement_(3))) < M_PI_4;
                pending_switch_ = false;
                target.last_observation_time = tp;
                target.last_observation_update_time = std::chrono::steady_clock::now();
                target.observation_updated = true;
            }
        }
        return true;
    }

    void Tracker::update_yaw_filter(bool observed, bool armor_switched) {
        if (adjust) {
            update_parameter();
        }
        if (observed) {
            if (!armor_switched) {
                target.yaw_measurement << target.yaw_state(0, 0) +
                                              mathutils::limit_rad(target.tracked_measurement(3, 0) -
                                                                   target.yaw_state(0, 0));
                target.yaw_state = yaw_kf.update(target.yaw_measurement);
            } else {
                reset_yaw_kf(true);
                if (debug) std::cout << "[predict] yaw kf reset" << std::endl;
            }
        }
    }

    void Tracker::update_armor_filters(bool observed, bool armor_switched) {
        if (observed) {
            if (target.updating_model_type != UpdatingModelType::VEHICLE_MODEL) {
                if (!armor_switched) {
                    target.armor_y_measurement << target.tracked_measurement(0, 0);
                    target.armor_y_state = armor_y_kf.update(target.armor_y_measurement);
                    target.armor_x_measurement << target.tracked_measurement(1, 0);
                    target.armor_x_state = armor_x_kf.update(target.armor_x_measurement);
                    target.armor_z_measurement << target.tracked_measurement(2, 0);
                    target.armor_z_state = armor_z_kf.update(target.armor_z_measurement);
                    if (debug) std::cout << "[predictor] kf update" << std::endl;
                } else {
                    reset_armor_y_kf();
                    reset_armor_x_kf();
                    reset_armor_z_kf();
                    if (debug) std::cout << "[predict] armor state kf reset" << std::endl;
                }
            } else {
                reset_armor_y_kf();
                reset_armor_x_kf();
                reset_armor_z_kf();
                if (debug) std::cout << "[predict] vehicle model update only" << std::endl;
            }
        } else {
            target.armor_x_state = armor_x_kf.predict();
            target.armor_y_state = armor_y_kf.predict();
            target.armor_z_state = armor_z_kf.predict();
            target.yaw_state = yaw_kf.predict();
            if (debug) std::cout << "[predict] no same id armor" << std::endl;
        }
    }

    void Tracker::update_vehicle_filter(int same_id_armor_count, int tag_id, bool armor_switched,
                                        TrackerDiagnostic &diagnostic) {
        if (same_id_armor_count) {
            if (target.updating_model_type != UpdatingModelType::ARMOR_MODEL ||
                target.vehicle_model_rotation_count > 0) {
                if (same_id_armor_count == 1) {
                    int id = match_armor_id(target.tracked_measurement);
                    target.tracked_state = whole_state_ekf.update(target.tracked_measurement, id);
                } else if (same_id_armor_count == 2) {
                    int id = match_armor_id(target.tracked_measurement);
                    target.tracked_state = whole_state_ekf.update(target.tracked_measurement, id);
                }
                target.vehicle_model_trust = target.vehicle_model_rotation_count >= least_rotate_count;
                if (armor_switched) {
                    if (debug) std::cout << "[predict] armor jump" << std::endl;
                }
                if (debug) std::cout << "[predict] ekf update" << std::endl;
                radium_limit();
                if (tag_id == 0) {
                    if (abs(target.tracked_state(7, 0)) > outpost_fix_yaw_speed_threshold) {
                        target.tracked_state(7, 0) =
                            target.tracked_state(7, 0) > 0 ? outpost_yaw_speed : -outpost_yaw_speed;
                    }
                    target.tracked_state(8, 0) = outpost_r;
                    target.tracked_state(9, 0) = 0.0;
                }
            } else {
                diagnostic.reset_reason = "vehicle_model_initialization_before_first_switch";
                reset_whole_state_ekf();
                if (debug) std::cout << "[predict] armor model update only" << std::endl;
            }
        } else {
            if (debug) std::cout << "[predict] no same id armor" << std::endl;
        }
    }

    const Target &Tracker::track(const Eigen::Matrix<double, 4, 1> &measurement,
                                 const Eigen::Matrix<double, 4, 1> &secondary_measurement, int same_id_armor_count,
                                 const int tag_id, const TP &tp, const double attitude_yaw) {
        TrackerDiagnostic diagnostic(this, target, measurement, tp, last_tp, same_id_armor_count, "track");
        bool confirmed_pending_switch = false;
        if (!process_observation(measurement, tp, same_id_armor_count, confirmed_pending_switch, diagnostic))
            return target;
        dt = duration_cast<microseconds>(tp - last_tp).count() / 1e6;
        if (!std::isfinite(dt) || dt <= 0.0 || dt > 1.0) {
            diagnostic.reset_reason = "invalid_frame_dt";
            if (same_id_armor_count == 0) {
                last_tp = tp;
                target.predictor_state = TrackingState::IDLE;
                target.vehicle_model_trust = false;
                target.vehicle_model_rotation_count = 0;
                pending_switch_ = false;
                return target;
            }
            TP reset_time = tp;
            return reset_target(measurement, reset_time);
        }
        Eigen::Matrix<double, 3, 1> measured_pw(target.tracked_measurement(1, 0), target.tracked_measurement(0, 0),
                                                target.tracked_measurement(2, 0));
        Eigen::Matrix<double, 3, 1> tracked_pw(measurement(1, 0), measurement(0, 0), measurement(2, 0));
        double min_position_diff = (measured_pw - tracked_pw).norm();
        const double normal_change = std::abs(mathutils::limit_rad(measurement(3) - target.tracked_measurement(3)));
        if (same_id_armor_count > 0 && normal_change < M_PI_4) min_position_diff = 0.0;
        const bool armor_switched =
            same_id_armor_count > 0 && (min_position_diff > same_position_threshold || confirmed_pending_switch);
        diagnostic.armor_switched = armor_switched;
        if (armor_switched) {
            target.ab_counter = 1 - target.ab_counter;
            if (target.vehicle_model_rotation_count < least_rotate_count) ++target.vehicle_model_rotation_count;
        }
        if (same_id_armor_count) {
            target.tracked_measurement = measurement;
            target.tracked_measurement(3, 0) = mathutils::limit_rad(target.tracked_measurement(3, 0));
        }
        target.tracked_state = whole_state_ekf.predict();
        if (tag_id == 8) {
            if (abs(target.tracked_state(7, 0)) > outpost_fix_yaw_speed_threshold) {
                target.tracked_state(7, 0) = target.tracked_state(7, 0) > 0 ? outpost_yaw_speed : -outpost_yaw_speed;
            }
            target.tracked_state(8, 0) = outpost_r;
            target.tracked_state(9, 0) = 0.0;
            p_coord = 1e2;
            p_yaw = 1e1;
        } else {
            p_coord = 1e2;
            p_yaw = 4e2;
        }
        if (debug) {
            std::cout << "dt: " << dt << std::endl;
            std::cout << "[predict] predict" << std::endl;
        }
        const double diagnostic_age = tracking_policy::observation_age(target.last_observation_time, tp);
        if (diagnostic_age < 0.0 || diagnostic_age >= tracking_policy::observation_timeout_s)
            diagnostic.reset_reason =
                diagnostic_age < 0.0 ? "negative_observation_age" : "observation_timeout_in_state_update";
        update_tracker_state(same_id_armor_count, tp);
        update_yaw_filter(same_id_armor_count > 0, armor_switched);
        tracker_model_select();
        update_armor_filters(same_id_armor_count > 0, armor_switched);
        update_vehicle_filter(same_id_armor_count, tag_id, armor_switched, diagnostic);
        if (!target.tracked_state.allFinite() || !target.yaw_state.allFinite() || !target.armor_x_state.allFinite() ||
            !target.armor_y_state.allFinite() || !target.armor_z_state.allFinite()) {
            diagnostic.reset_reason = "nonfinite_filter_output";
            LOGW_S("[Tracker] non-finite filter output; resetting from measurement");
            TP reset_time = tp;
            const auto image_seen = target.last_observation_time;
            const auto update_seen = target.last_observation_update_time;
            reset_target(target.tracked_measurement, reset_time);
            if (same_id_armor_count == 0) {
                target.last_observation_time = image_seen;
                target.last_observation_update_time = update_seen;
                target.observation_updated = false;
            }
            return target;
        }
        last_tp = tp;
        return target;
    }

    Eigen::Vector3d Tracker::h_armor_xyz(const Eigen::VectorXd &x, int id) {
        auto angle = mathutils::limit_rad(x[6] + id * M_PI_2);
        auto use_l_h = id == 1 || id == 3;

        auto r = (use_l_h) ? x[8] + x[9] : x[8];
        auto armor_x = x[0] - r * std::cos(angle);
        auto armor_y = x[2] - r * std::sin(angle);
        auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

        return {armor_x, armor_y, armor_z};
    }

    std::vector<Eigen::Vector4d> Tracker::armor_xyza_list() {
        std::vector<Eigen::Vector4d> _armor_xyza_list;

        for (int i = 0; i < 4; i++) {
            auto angle = mathutils::limit_rad(target.tracked_state(6, 0) + i * M_PI_2);
            Eigen::Vector3d xyz = h_armor_xyz(target.tracked_state, i);
            _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
        }
        return _armor_xyza_list;
    }

    int Tracker::match_armor_id(const Eigen::Matrix<double, 4, 1> &measurement) {
        std::vector<Eigen::Vector4d> standard_armors = armor_xyza_list();
        Pos3D matched_armor_pos = {measurement(1, 0), measurement(0, 0), measurement(2, 0)};
        Eigen::Matrix<double, 4, 2> negative_score;
        for (int j = 0; j < 4; j++) {
            Pos3D standard_armor_pos = {standard_armors[j][1], standard_armors[j][0], standard_armors[j][2]};
            negative_score(j, 0) = mathutils::distance_3D(matched_armor_pos - standard_armor_pos);
            negative_score(j, 1) = abs(mathutils::limit_rad(measurement(3, 0) - standard_armors[j](3, 0)));
        }
        if (!negative_score.allFinite()) return 0;
        Eigen::Matrix<double, 4, 2> regular_score;
        for (int column = 0; column < 2; ++column) {
            const double maximum = negative_score.col(column).maxCoeff();
            const double range = maximum - negative_score.col(column).minCoeff();
            for (int i = 0; i < regular_score.rows(); ++i)
                regular_score(i, column) = range > 1e-12 ? (maximum - negative_score(i, column)) / range : 1.0;
        }
        Eigen::Matrix<double, 4, 2> score_weight;
        Eigen::VectorXd col_sum = regular_score.colwise().sum();
        for (int i = 0; i < score_weight.rows(); i++) {
            score_weight(i, 0) = regular_score(i, 0) / col_sum(0);
            score_weight(i, 1) = regular_score(i, 1) / col_sum(1);
        }
        Eigen::Vector2d entropy = Eigen::Vector2d::Zero();
        for (int i = 0; i < score_weight.rows(); i++) {
            if (score_weight(i, 0) != 0) entropy(0) -= score_weight(i, 0) * log(score_weight(i, 0));
            if (score_weight(i, 1) != 0) entropy(1) -= score_weight(i, 1) * log(score_weight(i, 1));
        }
        entropy /= log(score_weight.rows());
        const double information = 2.0 - entropy.sum();
        Eigen::Vector2d weight = information > 1e-12 ? ((Eigen::Vector2d::Ones() - entropy) / information).eval()
                                                     : Eigen::Vector2d::Constant(0.5);
        Eigen::Matrix<double, 1, 4> score;
        for (int j = 0; j < 4; j++) {
            score(0, j) = negative_score.row(j) * weight;
        }
        int index;
        score.row(0).minCoeff(&index);
        return index;
    }

    /**
     * @brief 获取当前跟踪状态
     * @return 跟踪器状态枚举值
     */
    TrackingState Tracker::get_tracker_state() {
        return target.predictor_state;
    }

    /**
     * @brief 初始化目标状态参数
     * @details 设置目标的初始状态和默认参数
     */
    void Tracker::target_init() {
        target.predictor_state = TrackingState::IDLE;
        target.updating_model_type = UpdatingModelType::ARMOR_MODEL;
        target.ab_counter = 0;
        target.vehicle_model_trust = false;

        target.tracked_state.setZero();
        target.tracked_measurement.setZero();
        target.estimated_armor_m.setZero();

        target.yaw_state.setZero();
        target.yaw_measurement.setZero();
        target.armor_x_state.setZero();
        target.armor_x_measurement.setZero();
        target.armor_y_state.setZero();
        target.armor_y_measurement.setZero();
        target.armor_z_state.setZero();
        target.armor_z_measurement.setZero();

        target.another_r = 0.26;
        target.dz = 0.0;

        target.tracked_armor = bbox_t{};
        target.tracked_armor.confidence = 0.0f;
        target.tracked_armor.color_id = 0;
        target.tracked_armor.tag_id = 0;
        target.tracked_armor.source = DetectionSource::NEURAL_NETWORK;
        for (auto &pt : target.tracked_armor.pts) {
            pt = cv::Point2f(0.0f, 0.0f);
        }
    }

    /**
     * @brief 初始化装甲板模型卡尔曼滤波器组
     * @details 设置四个独立的2D卡尔曼滤波器：偏航角、X坐标、Y坐标、Z坐标
     *          每个滤波器使用位置-速度模型，状态为 [位置, 速度]
     */

    Tracker::filter_2d Tracker::make_axis_filter(const double *process_noise, const double *measurement_noise) {
        auto transition = [this](const Matx1_2d &) {
            Matxx_2d matrix;
            matrix << 1, dt, 0, 1;
            return matrix;
        };
        auto observation = [](const Matz1_2d &) {
            Matzx_2d matrix;
            matrix << 1, 0;
            return matrix;
        };
        auto process_covariance = [this, process_noise](const Matx1_2d &) {
            Matxx_2d matrix;
            matrix << std::pow(dt, 4) / 4 * *process_noise, std::pow(dt, 3) / 2 * *process_noise,
                std::pow(dt, 3) / 2 * *process_noise, std::pow(dt, 2) * *process_noise;
            return matrix;
        };
        auto observation_covariance = [measurement_noise](const Matz1_2d &) {
            Matzz_2d matrix;
            matrix << *measurement_noise;
            return matrix;
        };
        return filter_2d(transition, observation, process_covariance, observation_covariance, Matxx_2d::Zero(),
                         Matx1_2d::Zero());
    }

    void Tracker::armor_state_kf_init() {
        yaw_kf = make_axis_filter(&q_kf_yaw, &r_yaw);
        armor_y_kf = make_axis_filter(&q_kf_y, &r_ycoord);
        armor_x_kf = make_axis_filter(&q_kf_x, &r_xcoord);
        armor_z_kf = make_axis_filter(&q_kf_z, &r_zcoord);
    }

    /**
     * @brief 初始化整车状态扩展卡尔曼滤波器
     * @details 设置11维状态向量的EKF：[y, vy, x, vx, z, vz, yaw, vyaw, r]
     *          观测向量为4维：[y, x, z, yaw] (装甲板位置和朝向)
     */
    Eigen::Matrix4d Tracker::vehicle_observation_covariance(const Eigen::Vector4d &measurement) const {
        Eigen::Matrix<double, 4, 4> R = Eigen::Matrix<double, 4, 4>::Zero();
        double meas_y = measurement(0, 0);
        double meas_x = measurement(1, 0);
        double meas_z = measurement(2, 0);
        double dist_xy_sq = meas_y * meas_y + meas_x * meas_x;
        double dist_xy = std::sqrt(dist_xy_sq);
        double dist_3d = std::sqrt(dist_xy_sq + meas_z * meas_z);
        if (dist_xy < 0.1) dist_xy = 0.1;
        if (dist_3d < 0.1) dist_3d = 0.1;
        Eigen::Vector2d u_radial(meas_y / dist_xy, meas_x / dist_xy);
        Eigen::Vector2d u_tangent(-meas_x / dist_xy, meas_y / dist_xy);
        double sigma_pos_tangent = dist_3d * std_dev_azi_angle;
        double sigma_pos_z = dist_3d * std_dev_ele_angle;
        double sigma_pos_radial = dist_3d * std_dev_dist_coeff;
        double sigma_yaw = std_dev_tgt_yaw;
        Eigen::Matrix2d R_pos_xy = std::pow(sigma_pos_radial, 2) * (u_radial * u_radial.transpose()) +
                                   std::pow(sigma_pos_tangent, 2) * (u_tangent * u_tangent.transpose());
        R.block<2, 2>(0, 0) = R_pos_xy;
        R(2, 2) = std::pow(sigma_pos_z, 2);
        R(3, 3) = std::pow(sigma_yaw, 2);
        return R;
    }

    void Tracker::whole_state_ekf_init() {
        auto f_ = [this](const Eigen::Matrix<double, 11, 1> &x) {
            Eigen::Matrix<double, 11, 1> x_pri = x;
            x_pri(0, 0) += x(1, 0) * dt;
            x_pri(2, 0) += x(3, 0) * dt;
            x_pri(4, 0) += x(5, 0) * dt;
            x_pri(6, 0) += x(7, 0) * dt;
            x_pri(6, 0) = mathutils::limit_rad(x_pri(6, 0));
            return x_pri;
        };
        auto cal_F_ = [this](const Eigen::Matrix<double, 11, 1> &x) {
            Eigen::Matrix<double, 11, 11> F;
            F << 1, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
            return F;
        };
        auto h_ = [](const Eigen::Matrix<double, 11, 1> &x, const int id) {
            Eigen::Matrix<double, 4, 1> z;
            auto angle = mathutils::limit_rad(x(6, 0) + id * M_PI_2);
            auto is_another_r = id == 1 || id == 3;
            auto r = is_another_r ? x(8, 0) + x(9, 0) : x(8, 0);
            z(0, 0) = x(0, 0) - r * cos(angle);
            z(1, 0) = x(2, 0) - r * sin(angle);
            z(2, 0) = is_another_r ? x(4, 0) + x(10, 0) : x(4, 0);
            z(3, 0) = angle;
            return z;
        };
        auto cal_H_ = [](const Eigen::Matrix<double, 11, 1> &x, const int id) {
            auto angle = mathutils::limit_rad(x(6, 0) + id * M_PI_2);
            auto is_another_r = id == 1 || id == 3;
            auto r = is_another_r ? x(8, 0) + x(9, 0) : x(8, 0);
            auto dx_dl = is_another_r ? -cos(angle) : 0.0;
            auto dy_dl = is_another_r ? -sin(angle) : 0.0;
            auto dz_dh = is_another_r ? 1.0 : 0.0;
            Eigen::Matrix<double, 4, 11> H;
            H << 1, 0, 0, 0, 0, 0, r * sin(angle), 0, -cos(angle), dx_dl, 0, 0, 0, 1, 0, 0, 0, -r * cos(angle), 0,
                -sin(angle), dy_dl, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dz_dh, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
            return H;
        };
        auto update_Q_ = [this](const Eigen::Matrix<double, 11, 1> &x) {
            Eigen::Matrix<double, 11, 11> Q;
            double q_x_x = pow(dt, 4) / 4 * p_coord, q_x_vx = pow(dt, 3) / 2 * p_coord, q_vx_vx = pow(dt, 2) * p_coord;
            double q_y_y = pow(dt, 4) / 4 * p_yaw, q_y_vy = pow(dt, 3) / 2 * p_yaw, q_vy_vy = pow(dt, 2) * p_yaw;
            Q << q_x_x, q_x_vx, 0, 0, 0, 0, 0, 0, 0, 0, 0, q_x_vx, q_vx_vx, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, q_x_x,
                q_x_vx, 0, 0, 0, 0, 0, 0, 0, 0, 0, q_x_vx, q_vx_vx, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, q_x_x, q_x_vx, 0,
                0, 0, 0, 0, 0, 0, 0, 0, q_x_vx, q_vx_vx, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, q_y_y, q_y_vy, 0, 0, 0, 0, 0,
                0, 0, 0, 0, q_y_vy, q_vy_vy, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, p_r, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, p_r,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, p_r;
            return Q;
        };
        auto update_R_ = [this](const Eigen::Vector4d &measurement) {
            return vehicle_observation_covariance(measurement);
        };
        Eigen::Matrix<double, 11, 11> P0;
        P0 = Eigen::Matrix<double, 11, 11>::Identity();
        Eigen::Matrix<double, 11, 1> X0;
        X0.setZero();
        X0(8, 0) = 0.26;
        auto x_add_ = [](const Eigen::Matrix<double, 11, 1> &x1, const Eigen::Matrix<double, 11, 1> &x2) {
            Eigen::Matrix<double, 11, 1> res;
            res = x1 + x2;
            res(6, 0) = mathutils::limit_rad(res(6, 0));
            return res;
        };
        auto x_minus_ = [](const Eigen::Matrix<double, 11, 1> &x1, const Eigen::Matrix<double, 11, 1> &x2) {
            Eigen::Matrix<double, 11, 1> res = x1 - x2;
            while (res(6, 0) > M_PI)
                res(6, 0) -= 2 * M_PI;
            while (res(6, 0) < -M_PI)
                res(6, 0) += 2 * M_PI;
            return res;
        };
        whole_state_ekf =
            IESEKF_Double_Armor<4, 11>(f_, h_, cal_F_, cal_H_, update_Q_, update_R_, P0, X0, x_add_, x_minus_, 5, 1e-4);
    }

    /**
     * @brief 初始化参数调整界面
     * @details 创建OpenCV滑动条窗口，用于实时调整滤波器参数
     */
    void Tracker::parameter_adjustor_init() {
        // 为参数调整创建窗口
        autoaim_gui::namedWindow("predictor trackbar", cv::WINDOW_AUTOSIZE);

        // 过程噪声参数调整滑动条
        autoaim_gui::createTrackbar("p_coord_mant", "predictor trackbar", &p_coord_mant, 99, 0);
        autoaim_gui::createTrackbar("p_coord_exp", "predictor trackbar", &p_coord_exp, 20, 0);
        autoaim_gui::createTrackbar("p_yaw_mant", "predictor trackbar", &p_yaw_mant, 99, 0);
        autoaim_gui::createTrackbar("p_yaw_exp", "predictor trackbar", &p_yaw_exp, 20, 0);
        autoaim_gui::createTrackbar("p_r_mant", "predictor trackbar", &p_r_mant, 99, 0);
        autoaim_gui::createTrackbar("p_r_exp", "predictor trackbar", &p_r_exp, 20, 0);

        // === 观测噪声参数滑动条 (物理单位：角度-度、距离-%) ===
        autoaim_gui::createTrackbar("Azimuth_Deg_Int", "predictor trackbar", &azi_angle_deg_int, 50, 0);
        autoaim_gui::createTrackbar("Azimuth_Deg_Frac", "predictor trackbar", &azi_angle_deg_frac, 9, 0);

        autoaim_gui::createTrackbar("Pitch_Deg_Int", "predictor trackbar", &ele_angle_deg_int, 50, 0);
        autoaim_gui::createTrackbar("Pitch_Deg_Frac", "predictor trackbar", &ele_angle_deg_frac, 9, 0);

        autoaim_gui::createTrackbar("Distance_Coeff_%", "predictor trackbar", &dist_coeff_percent, 100, 0);

        autoaim_gui::createTrackbar("TargetYaw_Deg_Int", "predictor trackbar", &tgt_yaw_deg_int, 50, 0);
        autoaim_gui::createTrackbar("TargetYaw_Deg_Frac", "predictor trackbar", &tgt_yaw_deg_frac, 9, 0);

        // 装甲板模型KF参数调整滑动条
        // cv::createTrackbar("kf_yaw_mant", "predictor trackbar", &kf_yaw_mant, 99, 0);
        // cv::createTrackbar("kf_yaw_exp", "predictor trackbar", &kf_yaw_exp, 20, 0);
        // cv::createTrackbar("kf_y_mant", "predictor trackbar", &kf_y_mant, 99, 0);
        // cv::createTrackbar("kf_y_exp", "predictor trackbar", &kf_y_exp, 20, 0);
        // cv::createTrackbar("kf_x_mant", "predictor trackbar", &kf_x_mant, 99, 0);
        // cv::createTrackbar("kf_x_exp", "predictor trackbar", &kf_x_exp, 20, 0);
        // cv::createTrackbar("kf_z_mant", "predictor trackbar", &kf_z_mant, 99, 0);
        // cv::createTrackbar("kf_z_exp", "predictor trackbar", &kf_z_exp, 20, 0);
    }

    /**
     * @brief 更新滤波器参数
     * @details 从滑动条读取参数值并转换为实际的噪声协方差值
     */
    void Tracker::update_parameter() {
        // 更新观测噪声 (将物理单位转换为内部计算单位)
        // 角度: 度数 -> 弧度
        std_dev_azi_angle = (azi_angle_deg_int + azi_angle_deg_frac / 10.0) * 0.1 * (M_PI / 180.0);
        std_dev_ele_angle = (ele_angle_deg_int + ele_angle_deg_frac / 10.0) * 0.1 * (M_PI / 180.0);
        std_dev_tgt_yaw = (tgt_yaw_deg_int + tgt_yaw_deg_frac / 10.0) * 0.1 * (M_PI / 180.0);

        // 距离系数: 百分比 -> 小数
        std_dev_dist_coeff = dist_coeff_percent / 100.0;

        // 更新过程噪声
        p_yaw = sci_to_float(p_yaw_mant, p_yaw_exp - 10);
        p_coord = sci_to_float(p_coord_mant, p_coord_exp - 10);
        p_r = sci_to_float(p_r_mant, p_r_exp - 10);

        // 更新装甲板KF参数
        q_kf_yaw = sci_to_float(kf_yaw_mant, kf_yaw_exp - 10);
        q_kf_y = sci_to_float(kf_y_mant, kf_y_exp - 10);
        q_kf_x = sci_to_float(kf_x_mant, kf_x_exp - 10);
        q_kf_z = sci_to_float(kf_z_mant, kf_z_exp - 10);
    }

    /**
     * @brief 更新跟踪器状态机
     * @param same_id_armor_count 检测到的同ID装甲板数量
     * @details 管理IDLE、DETECTING、TRACKING、TEMP_LOST四个状态之间的转换
     */
    void Tracker::update_tracker_state(const double same_id_armor_count, const TP &tp) {
        const double age = tracking_policy::observation_age(target.last_observation_time, tp);
        if (age < 0.0 || age >= tracking_policy::observation_timeout_s) {
            target.predictor_state = TrackingState::IDLE;
            target.vehicle_model_trust = false;
            target.vehicle_model_rotation_count = 0;
            pending_switch_ = false;
            detecting_counter = temp_lost_counter = 0;
            return;
        }
        if (target.predictor_state == TrackingState::DETECTING) {
            if (same_id_armor_count > 0) {
                if (++detecting_counter > detecting_counter_threshold) {
                    detecting_counter = 0;
                    target.predictor_state = TrackingState::TRACKING;
                }
            } else {
                detecting_counter = 0;
                target.predictor_state = TrackingState::IDLE;
            }
        } else if (target.predictor_state == TrackingState::TRACKING && same_id_armor_count == 0) {
            target.predictor_state = TrackingState::TEMP_LOST;
            temp_lost_tp = tp;
        }
        if (target.predictor_state == TrackingState::TEMP_LOST && same_id_armor_count > 0)
            target.predictor_state = TrackingState::TRACKING;
        // Actual visual reacquisition resets before update; confirmed candidate jumps do not.
    }

    /**
     * @brief 跟踪模型选择策略
     * @details 根据目标旋转速度在装甲板模型、整车模型和混合模型之间自动切换
     *          装甲板模型在低旋转速度下效果好，
     *          整车模型在高旋转速度下效果好，但在低速度时可能发散
     *          （该模型的Q参数对仅平移运动不敏感）
     */
    void Tracker::tracker_model_select() {
        // 根据旋转速度更新模型
        if (target.updating_model_type == UpdatingModelType::ARMOR_MODEL) {
            if (abs(target.yaw_state(1, 0)) > armor_model_threshold) {
                // 速度超过装甲板模型阈值，切换到混合模型
                target.updating_model_type = UpdatingModelType::BOTH;
            }
        } else if (target.updating_model_type == UpdatingModelType::BOTH) {
            if (abs(target.yaw_state(1, 0)) > vehicle_model_threshold) {
                // 速度超过整车模型阈值，切换到纯整车模型
                target.updating_model_type = UpdatingModelType::VEHICLE_MODEL;
            } else if (abs(target.yaw_state(1, 0)) < armor_model_threshold) {
                // 速度低于装甲板模型阈值，切换到纯装甲板模型
                target.updating_model_type = UpdatingModelType::ARMOR_MODEL;
            }
        } else if (target.updating_model_type == UpdatingModelType::VEHICLE_MODEL) {
            if (abs(target.yaw_state(1, 0)) < vehicle_model_threshold) {
                // 速度低于整车模型阈值，切换到混合模型
                target.updating_model_type = UpdatingModelType::BOTH;
            }
        }
    }

    /**
     * @brief 重置偏航角卡尔曼滤波器
     * @details 用当前观测值重新初始化偏航角滤波器状态
     */
    void Tracker::reset_yaw_kf(bool preserve_velocity) {
        const double velocity = preserve_velocity && std::isfinite(target.yaw_state(1)) ? target.yaw_state(1) : 0.0;
        target.yaw_measurement << target.tracked_measurement(3, 0);
        target.yaw_state << target.tracked_measurement(3, 0), velocity;
        yaw_kf.reset(target.yaw_state);
    }

    /**
     * @brief 重置X坐标卡尔曼滤波器
     */
    void Tracker::reset_armor_x_kf() {
        target.armor_x_measurement << target.tracked_measurement(1, 0);
        target.armor_x_state << target.tracked_measurement(1, 0), 0;
        armor_x_kf.reset(target.armor_x_state);
    }

    /**
     * @brief 重置Y坐标卡尔曼滤波器
     */
    void Tracker::reset_armor_y_kf() {
        target.armor_y_measurement << target.tracked_measurement(0, 0);
        target.armor_y_state << target.tracked_measurement(0, 0), 0;
        armor_y_kf.reset(target.armor_y_state);
    }

    /**
     * @brief 重置Z坐标卡尔曼滤波器
     */
    void Tracker::reset_armor_z_kf() {
        target.armor_z_measurement << target.tracked_measurement(2, 0);
        target.armor_z_state << target.tracked_measurement(2, 0), 0;
        armor_z_kf.reset(target.armor_z_state);
    }

    /**
     * @brief 重置整车状态扩展卡尔曼滤波器
     * @details 从当前观测值推断车辆中心位置并重新初始化EKF
     */
    void Tracker::reset_whole_state_ekf() {
        // 从装甲板位置和朝向推算车辆中心位置
        double yc = target.tracked_measurement(0, 0) + 0.26 * cos(target.tracked_measurement(3, 0));
        double xc = target.tracked_measurement(1, 0) + 0.26 * sin(target.tracked_measurement(3, 0));

        // 设置初始状态：[yc, 0, xc, 0, zc, 0, yaw, 0, 0.26, 0, 0]
        target.tracked_state << yc, 0, xc, 0, target.tracked_measurement(2, 0), 0, target.tracked_measurement(3, 0), 0,
            0.26, 0, 0;

        whole_state_ekf.reset(target.tracked_state);

        // 重置相关参数
        target.another_r = 0.26;
        target.dz = 0;
        target.ab_counter = 0;
        yaw_speed_diverge_counter = 0;
        target.vehicle_model_rotation_count = 0;
        target.vehicle_model_trust = false;
        pending_switch_ = false;
    }

    /**
     * @brief 限制旋转半径在合理范围内
     * @details 将估计的旋转半径限制在合理范围内
     */
    void Tracker::radium_limit() {
        // 限制半径范围
        if (target.tracked_state(8, 0) < 0.12) {
            target.tracked_state(8, 0) = 0.12;
        } else if (target.tracked_state(8, 0) > 0.4) {
            target.tracked_state(8, 0) = 0.4;
        }

        if (target.tracked_state(8, 0) + target.tracked_state(9, 0) < 0.12) {
            target.tracked_state(9, 0) = 0.12 - target.tracked_state(8, 0);
        } else if (target.tracked_state(8, 0) + target.tracked_state(9, 0) > 0.4) {
            target.tracked_state(9, 0) = 0.4 - target.tracked_state(8, 0);
        }

        whole_state_ekf.set_state(target.tracked_state);
    }

    /**
     * @brief 检查扩展卡尔曼滤波器是否发散
     * @param attitude_yaw 机器人当前姿态偏航角
     * @return true表示检测到发散，需要重置滤波器
     * @details 根据估计的偏航角和偏航角速度检查整车模型是否发散
     */
    bool Tracker::check_ekf_divergence(const double attitude_yaw) {
        // 检查偏航角是否超出相机视野范围
        bool yaw_diverge = false;
        if (target.tracked_state(6, 0) > M_PI_2 - attitude_yaw || target.tracked_state(6, 0) < -M_PI_2 - attitude_yaw) {
            yaw_diverge = true;
        }

        // 检查两种模型对偏航角速度估计的一致性
        if (abs((target.tracked_state(7, 0) - target.yaw_state(1, 0))) > yaw_speed_diverge_threshold) {
            yaw_speed_diverge_counter++;
        } else {
            yaw_speed_diverge_counter = 0;
        }

        bool yaw_speed_diverge = false;
        if (yaw_speed_diverge_counter > yaw_speed_diverge_counter_threshold) {
            yaw_speed_diverge_counter = 0;
            yaw_speed_diverge = true;

            std::cout << "Yaw speed diverge detected! Resetting EKF..." << std::endl;
        }

        return yaw_diverge || yaw_speed_diverge;
    }

} // namespace predict
