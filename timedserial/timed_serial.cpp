//
// Created for hardware communication module - TimedSerial (Refactored)
// Event-driven architecture with dependency injection
//

#include "timed_serial.hpp"

#include <algorithm>
#include <cmath>

namespace hardware {
    // TimedSerial 实现
    TimedSerial::TimedSerial(const TimedSerialConfig &config, std::unique_ptr<SerialInterface> driver_impl,
                             pipeline::bridge::PlannerToSerialBridge &planner_bridge,
                             pipeline::bridge::SensorFromSerialAttitudeBridge &attitude_bridge,
                             pipeline::bridge::SensorFromSerialRobotStatusBridge &status_bridge)
        : BasicTask(), config_(config), driver_(std::move(driver_impl)), planner_bridge_(planner_bridge),
          attitude_bridge_(attitude_bridge), status_bridge_(status_bridge) {
        LOGM_S("[TimedSerial] constructing with injected driver");
        const auto &fire = fire_gate_.config();
        LOGM_S(
            "[FireControl] yaw_tol=%.3f deg pitch_tol=%.3f deg ref=%.2f m settle=%.1f ms rx_max_age=%.1f ms valid=%d",
            fire.yaw_tolerance_deg, fire.pitch_tolerance_deg, fire.reference_distance_m, fire.settle_ms,
            fire.rx_max_age_ms, fire.valid);

        // 注册为 Planner 消息接收者
        planner_bridge_.set_receiver(
            [this](const pipeline::bridge::PlannerToSerialMessage &msg) { this->handle_planner_message(msg); });

        attitude_bridge_.set_provider([this]() {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            return pipeline::bridge::SensorFromSerialAttitudeMessage{latest_attitude_, latest_attitude_received_at_};
        });

        attitude_bridge_.set_time_provider([this](std::chrono::steady_clock::time_point stamp) {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            const auto sample = attitude_history_.sample(stamp);
            return pipeline::bridge::SensorFromSerialAttitudeMessage{sample.attitude, sample.received_at, sample.valid,
                                                                     sample.error_ms};
        });
        status_bridge_.set_provider(
            [this]() { return pipeline::bridge::SensorFromSerialRobotStatusMessage{this->handle_robotstatus_get()}; });

        // 注册驱动层的两个独立回调
        driver_->set_attitude_callback([this](const Attitude &att) { this->handle_attitude_update(att); });

        driver_->set_robot_status_callback([this](const RobotStatus &sts) { this->handle_status_update(sts); });

        // 初始化驱动
        if (!driver_->init()) {
            LOGE_S("[TimedSerial] Failed to initialize serial driver");
            throw std::runtime_error("TimedSerial driver initialization failed");
        } else {
            LOGM_S("[TimedSerial] Serial driver initialized successfully");
            // 立即启动通讯
            driver_->start();
        }
    }

    /**
     * @brief 处理来自 Planner 的命令消息
     *
     * 这是 Planner 模块通过消息桥发送命令时的回调
     * 需要保护共享数据的访问
     */
    void TimedSerial::handle_planner_message(const pipeline::bridge::PlannerToSerialMessage &msg) {
        const auto finite_attitude = [](const Attitude &a) {
            return std::isfinite(a.yaw()) && std::isfinite(a.pitch()) && std::isfinite(a.roll());
        };
        const bool valid = finite_attitude(msg.attitude) && msg.plan_period.count() > 0 &&
                           std::all_of(msg.command_array.begin(), msg.command_array.end(), [](const RobotCommand &c) {
                               return std::isfinite(c.yaw_angle) && std::isfinite(c.pitch_angle) &&
                                      std::isfinite(c.yaw_speed) && std::isfinite(c.pitch_speed) &&
                                      std::isfinite(c.yaw_acc) && std::isfinite(c.pitch_acc) &&
                                      std::isfinite(c.distance) && c.distance >= 0.0f;
                           });
        const Attitude pose = finite_attitude(msg.attitude) ? msg.attitude : handle_attitude_get();
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_start_time_ = std::chrono::steady_clock::now();
        attitude_at_last_frame_ = pose;
        observation_deadline_ = msg.observation_deadline;
        fire_control_eligible_ = valid && msg.fire_control_eligible;
        fire_aim_mode_ = msg.aim_mode;
        single_shot_id_ = msg.single_shot_id;
        center_window_ = valid ? msg.center_window : center_fire::Snapshot{};
        model_trusted_ = msg.vehicle_model_trusted;
        model_rotation_count_ = msg.model_rotation_count;
        tracker_model_ = msg.tracker_model;
        plan_period_ = valid ? msg.plan_period : std::chrono::microseconds{2000};
        command_array_ = valid ? msg.command_array : command_array_t{};
        if (!valid) {
            LOGW_S("[TimedSerial] rejected non-finite planner command; holding RX angles");
            LOGW_F("[TimedSerial] rejected non-finite planner command; holding RX angles");
        }
    }

    /**
     * @brief 处理姿态数据更新
     *
     * 这是驱动层收到姿态数据时的回调
     * 立即更新本地状态，确保数据的实时性
     */
    void TimedSerial::handle_attitude_update(const Attitude &att) {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        latest_attitude_ = att;
        latest_attitude_received_at_ = std::chrono::steady_clock::now();
        attitude_history_.push(latest_attitude_received_at_, att);

        if (config_.debug.log_text) {
            // LOGM_S("[TimedSerial] Attitude updated: yaw=%.2f, pitch=%.2f",
            //        att.yaw(), att.pitch);
        }
    }

    Attitude TimedSerial::handle_attitude_get() {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        return latest_attitude_;
    }

    RobotStatus TimedSerial::handle_robotstatus_get() {
        std::lock_guard<std::mutex> lock(sensor_mutex_);

        return latest_robot_status_;
    }

    /**
     * @brief 处理机器人状态更新
     *
     * 这是驱动层收到状态数据时的回调
     * 采用合并策略：保留已有的射速信息，更新其他字段
     *
     * 注意：射速可能来自两个来源：
     * 1. IMU 包中的 shoot_speed（实时射速）
     * 2. 裁判系统包中的默认值
     *
     * 这里采用"保留已有值"的策略，避免被默认值覆盖
     */
    void TimedSerial::handle_status_update(const RobotStatus &sts) {
        RobotStatus snapshot{};
        bool report = false;
        {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            // 智能合并：如果新数据的射速来自IMU包，则更新；
            // 否则保留已有值（避免被裁判系统包的默认值覆盖）
            float current_speed = latest_robot_status_.robot_speed_mps;
            ProgramMode current_program_mode = latest_robot_status_.program_mode;

            constexpr float DEFAULT_SPEED = INF_BALL_SPEED;
            if (sts.robot_speed_mps == DEFAULT_SPEED) {
                latest_robot_status_ = sts;
                latest_robot_status_.robot_speed_mps = current_speed;
                latest_robot_status_.program_mode = current_program_mode;
            } else {
                latest_robot_status_.robot_speed_mps = sts.robot_speed_mps;
                latest_robot_status_.program_mode = sts.program_mode;
                latest_robot_status_.enemy_color = sts.enemy_color;
            }

            snapshot = latest_robot_status_;
            const uint64_t key = uint64_t(snapshot.enemy_color) | (uint64_t(snapshot.program_mode) << 8);
            report = config_.debug.log_text &&
                     status_log_gate_.sample(hero_log::Clock::now(), key, std::chrono::milliseconds(1000)).emit;
        }
        // Console output must not hold the attitude/status mutex.
        if (report) {
            LOGM_S("[TimedSerial] Status updated: enemy_color=%d, speed=%.2f", static_cast<int>(snapshot.enemy_color),
                   snapshot.robot_speed_mps);
        }
    }

    /**
     * @brief 读取最新命令和姿态数据，基于时间戳进行线性插值
     *
     * 此函数在发送线程中调用，需要锁保护
     */
    bool TimedSerial::read_latest_command_and_attitude() {
        std::lock_guard<std::mutex> lock(command_mutex_);
        fire_eligible_cache_ = fire_control_eligible_;
        fire_aim_mode_cache_ = fire_aim_mode_;
        single_shot_id_cache_ = single_shot_id_;
        center_window_cache_ = center_window_;
        model_trusted_cache_ = model_trusted_;
        model_rotation_count_cache_ = model_rotation_count_;
        tracker_model_cache_ = tracker_model_;
        observation_deadline_cache_ = observation_deadline_;
        if (plan_period_.count() <= 0) return false;
        if (command_array_.front().target_id != 0 &&
            (observation_deadline_ == std::chrono::steady_clock::time_point{} ||
             std::chrono::steady_clock::now() >= observation_deadline_)) {
            command_cache_ = RobotCommand{};
            command_array_ = command_array_t{};
            have_fresh_command_ = false;
            fire_eligible_cache_ = false;
            plan_period_ = std::chrono::microseconds{0};
            return true;
        }
        const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() -
                                                                                   command_start_time_);
        if (elapsed.count() < 0) return false;
        if (elapsed < command_hold_timeout_) {
            const auto index = elapsed.count() / plan_period_.count();
            if (index < static_cast<int64_t>(CMDARRAYLENGTH - 1)) {
                const auto offset = elapsed % plan_period_;
                command_cache_ = command_linear_interpolation(command_array_[index], command_array_[index + 1],
                                                              float(offset.count()) / float(plan_period_.count()));
            } else {
                // Hero accepts only final angles. Continue PC extrapolation
                // after the 18 ms command array, up to the 200 ms freshness limit.
                const double extra = (elapsed.count() - int64_t(CMDARRAYLENGTH - 1) * plan_period_.count()) / 1e6;
                command_cache_ = command_extrapolation(command_array_.back(), extra);
            }
            attitude_cache_ = attitude_at_last_frame_;
            have_fresh_command_ = true;
            return true;
        }
        if (have_fresh_command_) {
            command_cache_ = RobotCommand{};
            have_fresh_command_ = false;
            fire_eligible_cache_ = false;
            return true;
        }
        return false;
    }

    void TimedSerial::apply_tracking_slew_limit(float &yaw_deg, float &pitch_deg, bool target_valid) {
        if (!target_valid || !std::isfinite(yaw_deg) || !std::isfinite(pitch_deg)) {
            filtered_target_initialized_ = false;
            return;
        }

        if (!filtered_target_initialized_ || !std::isfinite(filtered_yaw_deg_) || !std::isfinite(filtered_pitch_deg_)) {
            filtered_yaw_deg_ = yaw_deg;
            filtered_pitch_deg_ = pitch_deg;
            filtered_target_initialized_ = true;
            return;
        }

        // std::remainder chooses the shortest yaw path across +/-180 deg.
        const float yaw_delta = std::remainder(yaw_deg - filtered_yaw_deg_, 360.0f);
        filtered_yaw_deg_ += std::clamp(yaw_delta, -yaw_step_limit_deg_, yaw_step_limit_deg_);
        filtered_pitch_deg_ +=
            std::clamp(pitch_deg - filtered_pitch_deg_, -pitch_step_limit_deg_, pitch_step_limit_deg_);

        yaw_deg = filtered_yaw_deg_;
        pitch_deg = filtered_pitch_deg_;
    }

    TimedSerial::~TimedSerial() {
        if (driver_) {
            driver_->close();
        }
        std::cout << "[TimedSerial] destroyed" << std::endl;
    }

    fire_control::Input TimedSerial::make_fire_input(bool target_valid, float desired_yaw, float desired_pitch,
                                                     float yaw_target, float pitch_target, bool holding_shot) {
        fire_control::Input fire_input;
        {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            fire_input.rx_yaw = latest_attitude_.yaw();
            fire_input.rx_pitch = latest_attitude_.pitch();
            fire_input.rx_time = latest_attitude_received_at_;
        }
        fire_input.now = std::chrono::steady_clock::now();
        fire_input.desired_yaw = desired_yaw;
        fire_input.desired_pitch = desired_pitch;
        fire_input.sent_yaw = yaw_target;
        fire_input.sent_pitch = pitch_target;
        fire_input.distance = command_cache_.distance;
        fire_input.target_id = command_cache_.target_id;
        fire_input.aim_mode = static_cast<int>(fire_aim_mode_cache_);
        fire_input.eligible = target_valid && fire_eligible_cache_;
        fire_input.requested = command_cache_.fire_enable;
        fire_input.single_shot_id = single_shot_id_cache_;
        fire_input.center_window = center_window_cache_;
        if (holding_shot) fire_input.requested = 0;

        return fire_input;
    }

    void TimedSerial::record_fire_diagnostics(const fire_control::Input &fire_input,
                                              const fire_control::Decision &fire_decision, bool holding_shot,
                                              float planned_yaw, float planned_pitch,
                                              fire_control::Clock::time_point decision_started,
                                              fire_control::Clock::time_point decision_finished) {
        fire_control::Diagnostics diagnostic;
        diagnostic.valid = true;
        diagnostic.decision = fire_decision;
        diagnostic.synchronized = true;
        diagnostic.input = fire_input;
        diagnostic.decision_started_at = decision_started;
        diagnostic.decision_finished_at = decision_finished;
        diagnostic.planned_yaw = planned_yaw;
        diagnostic.planned_pitch = planned_pitch;
        diagnostic.fire_config = fire_gate_.config();
        diagnostic.center_config = fire_gate_.center_config();
        diagnostic.requested = command_cache_.fire_enable;
        diagnostic.gate_requested = fire_input.requested;
        diagnostic.aim_mode = fire_input.aim_mode;
        diagnostic.eligible = fire_input.eligible;
        diagnostic.model_trusted = model_trusted_cache_;
        diagnostic.model_rotation_count = model_rotation_count_cache_;
        diagnostic.tracker_model = tracker_model_cache_;
        diagnostic.shot_hold = holding_shot;
        if (fire_input.rx_time != std::chrono::steady_clock::time_point{})
            diagnostic.rx_age_ms =
                std::chrono::duration<double, std::milli>(fire_input.now - fire_input.rx_time).count();
        diagnostic.model_speed_mps = center_window_cache_.bullet_speed;
        diagnostic.observation_fresh = observation_deadline_cache_ != std::chrono::steady_clock::time_point{} &&
                                       fire_input.now < observation_deadline_cache_;
        if (observation_deadline_cache_ != std::chrono::steady_clock::time_point{})
            diagnostic.observation_remaining_ms =
                std::chrono::duration<double, std::milli>(observation_deadline_cache_ - fire_input.now).count();
        driver_->set_fire_diagnostics(diagnostic);
    }

    void TimedSerial::log_transmission(float yaw_target, float pitch_target, uint8_t advice) {
        const auto status = handle_robotstatus_get();
        const uint64_t console_key =
            uint64_t(command_cache_.target_id) | (uint64_t(advice) << 8) | (uint64_t(status.program_mode) << 16);
        if (config_.debug.log_text &&
            transmit_log_gate_.sample(hero_log::Clock::now(), console_key, std::chrono::milliseconds(1000)).emit) {
            LOGM_S("[TimedSerial][transmit] p-m:%6.2f | p-s:%6.2f | ps-s:%6.2f | y-m:%6.2f | y-s:%6.2f | ys-s:%6.2f | "
                   "shoot_s:%6.2f | enemy_color:%d | target_id:%d | in_autoaim:%d | fire:%d",
                   attitude_cache_.pitch(), pitch_target, command_cache_.pitch_speed, attitude_cache_.yaw(), yaw_target,
                   command_cache_.yaw_speed, status.robot_speed_mps, status.enemy_color == EnemyColor::BLUE,
                   command_cache_.target_id, status.program_mode == ProgramMode::AUTO_AIM, advice);
        }
    }

    void TimedSerial::transmit_cached_command() {
        float yaw_target = attitude_cache_.yaw() + command_cache_.yaw_angle;
        float pitch_target = attitude_cache_.pitch() + command_cache_.pitch_angle;
        const bool target_valid = command_cache_.target_id != 0 && command_cache_.distance > 0.0f &&
                                  std::isfinite(yaw_target) && std::isfinite(pitch_target);
        if (!target_valid) {
            const auto current = handle_attitude_get();
            yaw_target = current.yaw();
            pitch_target = current.pitch();
            command_cache_ = RobotCommand{};
        }
        const float planned_yaw = yaw_target, planned_pitch = pitch_target;
        const bool holding_shot = std::chrono::steady_clock::now() < shot_hold_until_;
        if (holding_shot) {
            yaw_target = shot_hold_yaw_;
            pitch_target = shot_hold_pitch_;
        }
        const float desired_yaw = yaw_target, desired_pitch = pitch_target;
        apply_tracking_slew_limit(yaw_target, pitch_target, target_valid);
        auto fire_input =
            make_fire_input(target_valid, desired_yaw, desired_pitch, yaw_target, pitch_target, holding_shot);
        const auto decision_started = fire_control::Clock::now();
        const auto fire_decision = fire_gate_.evaluate(fire_input);
        const auto decision_finished = fire_control::Clock::now();
        if (fire_decision.advice == 2 && driver_->fire_actuation_enabled() &&
            fire_aim_mode_cache_ == AimedTargetType::VEHICLE_CENTER_WITH_VEHICLE_MODEL) {
            shot_hold_yaw_ = fire_input.rx_yaw;
            shot_hold_pitch_ = fire_input.rx_pitch;
            shot_hold_until_ =
                fire_input.now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                     std::chrono::duration<double>(center_window_cache_.release_delay_s));
            yaw_target = shot_hold_yaw_;
            pitch_target = shot_hold_pitch_;
            filtered_yaw_deg_ = yaw_target;
            filtered_pitch_deg_ = pitch_target;
            filtered_target_initialized_ = true;
        }
        record_fire_diagnostics(fire_input, fire_decision, holding_shot, planned_yaw, planned_pitch,
                                decision_started, decision_finished);
        driver_->transmit_cmd(yaw_target, command_cache_.yaw_speed, pitch_target, command_cache_.pitch_speed,
                              command_cache_.yaw_acc, command_cache_.pitch_acc, command_cache_.distance,
                              fire_decision.advice, command_cache_.target_id);
        log_transmission(yaw_target, pitch_target, fire_decision.advice);
    }

    void TimedSerial::operator()() {
        while (true) {
            if (!wait_for_state_change()) {
                break;
            }
            unsigned long long frame_index = 0;
            while (isalive()) {
                auto start_time = std::chrono::high_resolution_clock::now();
                if (!driver_) {
                    if (config_.debug.log_text) {
                        LOGE_S("[TimedSerial] Driver not available");
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    continue;
                }
                if (read_latest_command_and_attitude()) {
                    auto read_time_cost = std::chrono::high_resolution_clock::now() - start_time;
                    if (config_.debug.log_text)
                        if (read_time_cost > std::chrono::microseconds(500) &&
                            cost_log_gate_.sample(hero_log::Clock::now(), 0, std::chrono::milliseconds(1000)).emit)
                            LOGM_S("[timedserial] Cost time: %lld us",
                                   (long long)std::chrono::duration_cast<std::chrono::microseconds>(read_time_cost)
                                       .count());
                    transmit_cached_command();
                } else {
                    if (config_.debug.log_text &&
                        idle_log_gate_.sample(hero_log::Clock::now(), 0, std::chrono::milliseconds(1000)).emit)
                        LOGW_S("[TimedSerial] No new command to send");
                }
                if (config_.debug.log_text) {
                    CNT_FPS(total_fps, {});
                }
                auto end_time = std::chrono::high_resolution_clock::now();
                auto sleep_duration = send_period - (end_time - start_time);
                if (sleep_duration > std::chrono::milliseconds(0)) {
                    std::this_thread::sleep_for(sleep_duration);
                } else {
                    if (config_.debug.log_text &&
                        overrun_log_gate_.sample(hero_log::Clock::now(), 0, std::chrono::milliseconds(1000)).emit)
                        LOGW_S(
                            "[TimedSerial] sending overrun by %lld us",
                            (long long)std::chrono::duration_cast<std::chrono::microseconds>(-sleep_duration).count());
                    LOGW_F("[TimedSerial] sending overrun by %lld us",
                           (long long)std::chrono::duration_cast<std::chrono::microseconds>(-sleep_duration).count());
                }
            }
        }
    }
} // namespace hardware
