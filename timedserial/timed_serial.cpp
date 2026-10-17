//
// Created for hardware communication module - TimedSerial (Refactored)
// Event-driven architecture with dependency injection
//

#include "timed_serial.hpp"

#include <algorithm>
#include <cmath>

namespace hardware
{
    // TimedSerial 实现
    TimedSerial::TimedSerial(const TimedSerialConfig& config,
                                   std::unique_ptr<SerialInterface> driver_impl, 
                                   pipeline::bridge::PlannerToSerialBridge &planner_bridge,
                                   pipeline::bridge::SensorFromSerialAttitudeBridge &attitude_bridge,
                                   pipeline::bridge::SensorFromSerialRobotStatusBridge &status_bridge) 
        : BasicTask(), 
          config_(config),
          driver_(std::move(driver_impl)), 
          planner_bridge_(planner_bridge),
          attitude_bridge_(attitude_bridge),
          status_bridge_(status_bridge)
    {
        LOGM_S("[TimedSerial] constructing with injected driver");

        // 注册为 Planner 消息接收者
        planner_bridge_.set_receiver([this](const pipeline::bridge::PlannerToSerialMessage &msg) {
            this->handle_planner_message(msg);
        });

        attitude_bridge_.set_provider([this]() {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            return pipeline::bridge::SensorFromSerialAttitudeMessage{
                latest_attitude_, latest_attitude_received_at_};
        });

        status_bridge_.set_provider([this]() {
            return pipeline::bridge::SensorFromSerialRobotStatusMessage{this->handle_robotstatus_get()};
        });

        // 注册驱动层的两个独立回调
        driver_->set_attitude_callback([this](const Attitude& att) {
            this->handle_attitude_update(att);
        });

        driver_->set_robot_status_callback([this](const RobotStatus& sts) {
            this->handle_status_update(sts);
        });

        // 初始化驱动
        if (!driver_->init())
        {
            LOGE_S("[TimedSerial] Failed to initialize serial driver");
            throw std::runtime_error("TimedSerial driver initialization failed");
        }
        else
        {
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
    void TimedSerial::handle_planner_message(const pipeline::bridge::PlannerToSerialMessage &msg)
    {
        const auto finite_attitude = [](const Attitude &a) {
            return std::isfinite(a.yaw()) && std::isfinite(a.pitch()) && std::isfinite(a.roll());
        };
        const bool valid = finite_attitude(msg.attitude) && msg.plan_period.count() > 0 &&
            std::all_of(msg.command_array.begin(), msg.command_array.end(),
                [](const RobotCommand &c) {
                    return std::isfinite(c.yaw_angle) && std::isfinite(c.pitch_angle) &&
                           std::isfinite(c.yaw_speed) && std::isfinite(c.pitch_speed) &&
                           std::isfinite(c.yaw_acc) && std::isfinite(c.pitch_acc) &&
                           std::isfinite(c.distance) && c.distance >= 0.0f;
                });
        const Attitude pose = finite_attitude(msg.attitude) ? msg.attitude : handle_attitude_get();
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_start_time_ = std::chrono::steady_clock::now();
        attitude_at_last_frame_ = pose;
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
    void TimedSerial::handle_attitude_update(const Attitude& att)
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        latest_attitude_ = att;
        latest_attitude_received_at_ = std::chrono::steady_clock::now();
        
        if (config_.debug.log_text)
        {
            // LOGM_S("[TimedSerial] Attitude updated: yaw=%.2f, pitch=%.2f", 
            //        att.yaw(), att.pitch);
        }
    }

    Attitude TimedSerial::handle_attitude_get()
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        return latest_attitude_;
    }

    RobotStatus TimedSerial::handle_robotstatus_get()
    {
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
    void TimedSerial::handle_status_update(const RobotStatus& sts)
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        
        // 智能合并：如果新数据的射速来自IMU包，则更新；
        // 否则保留已有值（避免被裁判系统包的默认值覆盖）
        float current_speed = latest_robot_status_.robot_speed_mps;
        ProgramMode current_program_mode = latest_robot_status_.program_mode;

        constexpr float DEFAULT_SPEED = INF_BALL_SPEED;
        if (sts.robot_speed_mps == DEFAULT_SPEED)
        {
            latest_robot_status_ = sts;
            latest_robot_status_.robot_speed_mps = current_speed;
            latest_robot_status_.program_mode = current_program_mode;
        }
        else {
            latest_robot_status_.robot_speed_mps = sts.robot_speed_mps;
            latest_robot_status_.program_mode = sts.program_mode;
            latest_robot_status_.enemy_color = sts.enemy_color;
        }
        
        if (config_.debug.log_text)
        {
            LOGM_S("[TimedSerial] Status updated: enemy_color=%d, speed=%.2f", 
                   static_cast<int>(latest_robot_status_.enemy_color), 
                   latest_robot_status_.robot_speed_mps);
        }
    }

    /**
     * @brief 读取最新命令和姿态数据，基于时间戳进行线性插值
     * 
     * 此函数在发送线程中调用，需要锁保护
     */
    bool TimedSerial::read_latest_command_and_attitude()
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (plan_period_.count() <= 0) return false;
        const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - command_start_time_);
        if (elapsed.count() < 0) return false;
        if (elapsed < command_hold_timeout_) {
            const auto index = elapsed.count() / plan_period_.count();
            if (index < static_cast<int64_t>(CMDARRAYLENGTH - 1)) {
                const auto offset = elapsed % plan_period_;
                command_cache_ = command_linear_interpolation(
                    command_array_[index], command_array_[index + 1],
                    float(offset.count()) / float(plan_period_.count()));
            } else {
                // Hero accepts only final angles. Continue PC extrapolation
                // after the 18 ms command array, up to the 200 ms freshness limit.
                const double extra = (elapsed.count() -
                    int64_t(CMDARRAYLENGTH - 1) * plan_period_.count()) / 1e6;
                command_cache_ = command_extrapolation(command_array_.back(), extra);
            }
            attitude_cache_ = attitude_at_last_frame_;
            have_fresh_command_ = true;
            return true;
        }
        if (have_fresh_command_) {
            command_cache_ = RobotCommand{};
            have_fresh_command_ = false;
            return true;
        }
        return false;
    }

    void TimedSerial::apply_tracking_slew_limit(float &yaw_deg, float &pitch_deg,
                                                 bool target_valid)
    {
        if (!target_valid || !std::isfinite(yaw_deg) || !std::isfinite(pitch_deg))
        {
            filtered_target_initialized_ = false;
            return;
        }

        if (!filtered_target_initialized_ ||
            !std::isfinite(filtered_yaw_deg_) || !std::isfinite(filtered_pitch_deg_))
        {
            filtered_yaw_deg_ = yaw_deg;
            filtered_pitch_deg_ = pitch_deg;
            filtered_target_initialized_ = true;
            return;
        }

        // std::remainder chooses the shortest yaw path across +/-180 deg.
        const float yaw_delta = std::remainder(yaw_deg - filtered_yaw_deg_, 360.0f);
        filtered_yaw_deg_ += std::clamp(yaw_delta, -yaw_step_limit_deg_, yaw_step_limit_deg_);
        filtered_pitch_deg_ += std::clamp(
            pitch_deg - filtered_pitch_deg_, -pitch_step_limit_deg_, pitch_step_limit_deg_);

        yaw_deg = filtered_yaw_deg_;
        pitch_deg = filtered_pitch_deg_;
    }

    TimedSerial::~TimedSerial()
    {
        if (driver_)
        {
            driver_->close();
        }
        std::cout << "[TimedSerial] destroyed" << std::endl;
    }

    void TimedSerial::operator()()
    {
        // basictask框架级实现：统一的等待-工作循环
        while (true)
        {
            // basictask框架级实现：等待-启动信号或终止信号
            if (!wait_for_state_change())
            {
                break; // 收到终止信号，退出线程
            }

            // basictask框架级实现：收到启动信号，开始工作循环
            // 本循环内部是具体的任务实现
            unsigned long long frame_index = 0;
            while (isalive())
            {
                auto start_time = std::chrono::high_resolution_clock::now();

                if (!driver_)
                {
                    if (config_.debug.log_text)
                    {
                        LOGE_S("[TimedSerial] Driver not available");
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    continue;
                }

                // 发送控制指令
                if (read_latest_command_and_attitude())
                {
                    auto read_time_cost = std::chrono::high_resolution_clock::now() - start_time;
                    if(config_.debug.log_text)
                        if(read_time_cost > std::chrono::microseconds(500))
                            LOGM_S("[timedserial] Cost time: %lld us", 
                                   (long long)std::chrono::duration_cast<std::chrono::microseconds>(read_time_cost).count());
                    
                    float yaw_target = attitude_cache_.yaw() + command_cache_.yaw_angle;
                    float pitch_target = attitude_cache_.pitch() + command_cache_.pitch_angle;
                    const bool target_valid = command_cache_.target_id != 0 &&
                        command_cache_.distance > 0.0f &&
                        std::isfinite(yaw_target) && std::isfinite(pitch_target);
                    if (!target_valid) {
                        const auto current = handle_attitude_get();
                        yaw_target = current.yaw();
                        pitch_target = current.pitch();
                        command_cache_ = RobotCommand{};
                    }
                    apply_tracking_slew_limit(yaw_target, pitch_target, target_valid);

                    driver_->transmit_cmd(
                        yaw_target,
                        command_cache_.yaw_speed,
                        pitch_target,
                        command_cache_.pitch_speed,
                        command_cache_.yaw_acc,
                        command_cache_.pitch_acc,
                        command_cache_.distance,
                        command_cache_.fire_enable,
                        command_cache_.target_id);

                    if (config_.debug.log_text)
                    {
                        LOGM_S("[TimedSerial][transmit] p-m:%6.2f | p-s:%6.2f | ps-s:%6.2f | y-m:%6.2f | y-s:%6.2f | ys-s:%6.2f | shoot_s:%6.2f | enemy_color:%d | target_id:%d | in_autoaim:%d | fire:%d",
                               attitude_cache_.pitch(),
                               pitch_target,
                               command_cache_.pitch_speed,
                               attitude_cache_.yaw(),
                               yaw_target,
                               command_cache_.yaw_speed,
                               latest_robot_status_.robot_speed_mps,
                               latest_robot_status_.enemy_color==EnemyColor::BLUE,
                               command_cache_.target_id,
                               latest_robot_status_.program_mode==ProgramMode::AUTO_AIM,
                               command_cache_.fire_enable);
                    }
                }
                else
                {
                    if (config_.debug.log_text)
                        LOGW_S("[TimedSerial] No new command to send");
                }

                if (config_.debug.log_text)
                {
                    CNT_FPS(total_fps, {});
                }

                auto end_time = std::chrono::high_resolution_clock::now();
                auto sleep_duration = send_period - (end_time - start_time);
                if (sleep_duration > std::chrono::milliseconds(0))
                {
                    std::this_thread::sleep_for(sleep_duration);
                    // LOGW_S("[TimedSerial] sending on time, slept");
                }
                else
                {
                    // LOGW_S("[TimedSerial] sending cost %lld ms", 
                    //     (long long)std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count());
                    LOGW_S("[TimedSerial] sending overrun by %lld us", 
                           (long long)std::chrono::duration_cast<std::chrono::microseconds>(-sleep_duration).count());
                    LOGW_F("[TimedSerial] sending overrun by %lld us", 
                           (long long)std::chrono::duration_cast<std::chrono::microseconds>(-sleep_duration).count());
                }
                
                // LOGM_F("[timedserial_new]%llu start time: %lld|last time: %lld", frame_index++,
                //        static_cast<long long>(std::chrono::duration_cast<std::chrono::microseconds>(start_time.time_since_epoch()).count()),
                //        static_cast<long long>(std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count())
                //);
            }
            // basictask框架级实现：工作循环结束（被stop），回到等待状态
        }
    }
}
