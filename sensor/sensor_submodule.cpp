#include "gui_control.hpp"
#include <cstdlib>
//
// Created for pipeline refactor - SensorSubModule
// Wraps original Sensor logic as SubModule (Camera only, hardware communication moved to hardware::TimedSerial)
//

#include "sensor_submodule.hpp"
#ifdef ENABLE_HERO_SHARED_FRAME
#include "shared_frame_wrapper.hpp"
#endif
#include "calibration_capture.hpp"
#include "diagnostic_capture.hpp"

// submodules
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <video/video_wrapper.hpp>
#ifdef ENABLE_HIKCAM
#include <hikcam/hikcam_wrapper.hpp>
#endif

namespace sensor {
    // SensorSubModule 实现
    SensorSubModule::SensorSubModule(const SensorConfig &config, const std::string &VideoSource,
                                     const std::string &flip_image,
                                     pipeline::bridge::SensorFromSerialAttitudeBridge &attitude_bridge,
                                     pipeline::bridge::SensorFromSerialRobotStatusBridge &status_bridge)
        : SubModule(SubModuleName::SENSOR), config_(config), attitude_bridge(attitude_bridge),
          status_bridge(status_bridge) {
        LOGM_S("[sensor_submodule] constructing with video: %s", VideoSource.c_str());
        const bool is_camera_source = (VideoSource == "0");
        shared_source_ = VideoSource == "shared" || VideoSource.compare(0, 7, "shared:") == 0;

        // Shared input only consumes frames; it never opens or manages a camera.
        if (shared_source_) {
#ifdef ENABLE_HERO_SHARED_FRAME
            video = std::make_unique<SharedFrameWrapper>(VideoSource == "shared" ? std::string{} : VideoSource.substr(7));
#else
            throw std::runtime_error("shared source requested without HERO_SHARED_FRAME_ROOT");
#endif
        } else if (is_camera_source) {
#ifdef ENABLE_HIKCAM
            video = std::make_unique<HikCamWrapper>();
#else
            throw std::runtime_error("camera source requested but ENABLE_HIKCAM is disabled");
#endif
        } else {
            video = std::make_unique<VideoWrapper>(VideoSource);
        }

        if (!video) {
            throw std::runtime_error("failed to create video source wrapper");
        }

        // 初始化视频设备
        if (is_camera_source) {
            int retry_count = 0;
            const auto check_stop = [&] {
                if (config_.stop_requested && config_.stop_requested())
                    throw std::runtime_error("camera initialization cancelled by stop request");
            };
            for (;;) {
                check_stop();
                if (video->init()) {
                    check_stop();
                    break;
                }
                check_stop();
                ++retry_count;
                if (retry_count == 1 || retry_count % 10 == 0) {
                    LOGE_S("[sensor_submodule]Error: Initialize camera stream failed, retrying at 10Hz... (%d)",
                           retry_count);
                }
                // Preserve 10 Hz retries, but notice Ctrl+C during the wait.
                for (int i = 0; i < 10; ++i) {
                    check_stop();
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }

            if (retry_count > 0) {
                LOGM_S("[sensor_submodule] camera stream initialized after %d retries", retry_count);
            }
        } else if (!video->init()) {
            throw std::runtime_error("failed to initialize video stream: " + VideoSource);
        }
        LOGM_S("[sensor_submodule] video initialized");

        // 设置图像翻转标志
        if (flip_image == "1") {
            is_image_input_flipped = true;
            LOGM_S("[sensor_submodule] Input image will be flipped");
        } else {
            is_image_input_flipped = false;
            // LOGM_S("[sensor_submodule] Input image will not be flipped");
        }

        LOGM_S("[sensor_submodule] construction completed");
    }

    SensorSubModule::~SensorSubModule() {
        if (video) {
            video->close();
            video.reset();
        }
    }

    bool SensorSubModule::read_frame(ThreadDataPack &data, CaptureContext &context) {
        bool state = video->read(data.frame, config_.debug.log_text);
        // Missing/stale frames are a normal producer outage. Never reopen a camera
        // or rewrite the old frame's capture timestamp while waiting for recovery.
        if (!state && shared_source_) return false;
        const auto read_completed = std::chrono::steady_clock::now();
        auto frame_received = video->frame_received_at();
        if (frame_received == std::chrono::steady_clock::time_point{} || frame_received > read_completed ||
            read_completed - frame_received > std::chrono::seconds(1))
            frame_received = read_completed;
        static const double image_delay_ms = [] {
            const char *value = std::getenv("AUTOAIM_IMAGE_DELAY_MS");
            if (!value || !*value) return 0.0;
            char *end = nullptr;
            const double delay = std::strtod(value, &end);
            return end && !*end && std::isfinite(delay) && delay >= 0.0 && delay <= 200.0 ? delay : 0.0;
        }();
        const auto frame_time = frame_received - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                     std::chrono::duration<double, std::milli>(image_delay_ms));
        data.capture_time = frame_time;
        data.time = std::chrono::high_resolution_clock::now() -
                    std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
                        std::chrono::steady_clock::now() - frame_time);
        data.camera_frame_number = video->frame_number();
        data.camera_host_timestamp = video->host_timestamp();
        data.camera_device_timestamp = video->device_timestamp();
        context.image_read_unix_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
                .count();
        context.attitude_message = attitude_bridge.get_at(frame_time);
        context.attitude_sampled_at = std::chrono::steady_clock::now();
        data.attitude = context.attitude_message.attitude;
        context.attitude_fresh = calibration::fresh(context.attitude_message.received_at, context.attitude_sampled_at);
        data.attitude_age_ms = context.attitude_message.received_at == std::chrono::steady_clock::time_point{}
                                   ? INFINITY
                                   : std::chrono::duration<double, std::milli>(context.attitude_sampled_at -
                                                                               context.attitude_message.received_at)
                                         .count();
        data.attitude_alignment_error_ms = context.attitude_message.alignment_error_ms;
        data.attitude_valid = context.attitude_fresh && context.attitude_message.aligned;
        data.robotstatus = status_bridge.get().robotstatus;
        if (!state) {
            LOGE_S("[sensor_submodule]Error: read image fail!");
            if (config_.debug.log_text) {
                LOGM_S("[sensor_submodule] Total frames handled: %d", data.index);
                LOGM_S("[sensor_submodule] ReOpen Camera");
            }
            video->close(config_.debug.log_text);
            video->init(config_.debug.log_text);
            return false;
        }
        if (data.frame.empty()) {
            LOGE_S("[sensor_submodule] empty image");
            return false;
        }
        if (is_image_input_flipped) {
            cv::flip(data.frame, data.frame, -1);
        }
        return true;
    }

    bool SensorSubModule::check_board(const ThreadDataPack &data, CaptureContext &context) {
        if (!context.board_checked) {
            context.board_found = calibration::detect_board(data.frame, context.calibration_corners);
            context.board_checked = true;
        }
        return context.board_found;
    }

    bool SensorSubModule::save_pair(const ThreadDataPack &data, CaptureContext &context) {
        const char *dry_run = std::getenv("AUTOAIM_XUC_DRY_RUN");
        std::string error;
        uint64_t sample_id = 0;
        bool ok = false;
        if (!dry_run || std::string(dry_run) != "1")
            error = "capture requires AUTOAIM_XUC_DRY_RUN=1";
        else if (!context.attitude_fresh)
            error = "no fresh RX attitude (limit: 250 ms)";
        else if (!context.validation_capture && !check_board(data, context))
            error = "complete 9x7 board not found";
        else {
            static calibration::PairWriter writer;
            ok = writer.save(data.frame, {data.attitude.yaw(), data.attitude.pitch(), data.attitude.roll()}, sample_id,
                             error);
            if (ok) {
                const auto directory = writer.directory().string();
                capture_hint_ = "Saved pair " + std::to_string(sample_id);
                const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        context.attitude_sampled_at - context.attitude_message.received_at)
                                        .count();
                if (context.validation_capture) {
                    const std::string metadata_path = directory + "/frame_" + std::to_string(sample_id) + ".json";
                    std::ofstream metadata(metadata_path);
                    metadata << std::setprecision(12)
                             << "{\n\"context.image_read_unix_ms\":" << context.image_read_unix_ms
                             << ",\n\"rx_age_ms\":" << age_ms << ",\n\"yaw_deg\":" << data.attitude.yaw()
                             << ",\n\"pitch_deg\":" << data.attitude.pitch()
                             << ",\n\"roll_deg\":" << data.attitude.roll()
                             << ",\n\"board_detection_required\":false\n}\n";
                    metadata.flush();
                    if (!metadata)
                        LOGW_S("[sensor_submodule] pair saved, but A/B metadata write failed: %s",
                               metadata_path.c_str());
                }
                LOGM_S("[sensor_submodule] saved pair: %s/frame_%llu.png + euler_%llu.txt; "
                       "yaw=%.6f pitch=%.6f roll=%.6f deg; RX age at image read=%lld ms",
                       directory.c_str(), static_cast<unsigned long long>(sample_id),
                       static_cast<unsigned long long>(sample_id), data.attitude.yaw(), data.attitude.pitch(),
                       data.attitude.roll(), static_cast<long long>(age_ms));
            }
        }
        if (!ok) {
            capture_hint_ = "Not saved: " + error;
            LOGW_S("[sensor_submodule] calibration pair rejected: %s", error.c_str());
        }
        last_capture_ok_ = ok;
        hint_until_ = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        return ok;
    }

    void SensorSubModule::show_calibration_frame(const ThreadDataPack &data, CaptureContext &context) {
        cv::Mat im2show;
        const double scale = std::min(1.0, std::min(800.0 / data.frame.cols, 560.0 / data.frame.rows));
        if (scale < 1.0)
            cv::resize(data.frame, im2show, cv::Size(), scale, scale, cv::INTER_AREA);
        else
            im2show = data.frame.clone();
        cv::putText(im2show, context.validation_capture ? "A/B DRY_RUN / raw frame + RX" : "9x7 board / 20 mm",
                    cv::Point(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        cv::putText(im2show, "Stop, then press P to save image + Euler", cv::Point(20, 70), cv::FONT_HERSHEY_SIMPLEX,
                    0.65, cv::Scalar(0, 255, 0), 2);
        std::ostringstream attitude_text;
        attitude_text << std::fixed << std::setprecision(2) << "Yaw " << data.attitude.yaw() << "  Pitch "
                      << data.attitude.pitch() << "  Roll " << data.attitude.roll() << " deg";
        cv::putText(im2show, attitude_text.str(), cv::Point(20, 105), cv::FONT_HERSHEY_SIMPLEX, 0.65,
                    cv::Scalar(255, 255, 0), 2);
        const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(context.attitude_sampled_at -
                                                                                  context.attitude_message.received_at)
                                .count();
        const std::string rx_text = context.attitude_fresh ? "RX fresh at image read: " + std::to_string(age_ms) + " ms"
                                                           : "RX missing/stale - cannot save";
        cv::putText(im2show, rx_text, cv::Point(20, 140), cv::FONT_HERSHEY_SIMPLEX, 0.65,
                    context.attitude_fresh ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
        if (config_.calib_detect) {
            if (check_board(data, context)) {
                std::vector<cv::Point2f> display_corners;
                for (const auto &corner : context.calibration_corners)
                    display_corners.emplace_back(corner.x * static_cast<float>(im2show.cols) / data.frame.cols,
                                                 corner.y * static_cast<float>(im2show.rows) / data.frame.rows);
                cv::drawChessboardCorners(im2show, calibration::board_size, display_corners, true);
            }
            cv::putText(im2show, context.board_found ? "Detected: 63 corners" : "Complete board not detected",
                        cv::Point(20, 175), cv::FONT_HERSHEY_SIMPLEX, 0.65,
                        context.board_found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
        }
        if (std::chrono::steady_clock::now() < hint_until_)
            cv::putText(im2show, capture_hint_, cv::Point(20, 210), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        last_capture_ok_ ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 0, 255), 2);
        autoaim_gui::imshow("sensor_submodule", im2show);
        const int key = autoaim_gui::waitKey(1);
        if (key == 'p' || key == 'P') save_pair(data, context);
    }

    SubModuleResult SensorSubModule::process(std::shared_ptr<ThreadDataPack> data, const pipeline::BasicTask *) {
        CaptureContext context;
        if (!read_frame(*data, context)) return SubModuleResult::FAILURE;
        diagnostic_capture::sample(*data, PROJECT_DIR);
        const char *validation = std::getenv("AUTOAIM_VALIDATION_CAPTURE");
        context.validation_capture = validation && std::string(validation) == "1";
        const bool auto_save_due =
            config_.save_image && config_.save_interval > 0 && ++frames_since_save_ >= config_.save_interval;
        if (auto_save_due) {
            frames_since_save_ = 0;
            save_pair(*data, context);
        }
        if (config_.debug.show_image) show_calibration_frame(*data, context);
        if (config_.debug.log_text) CNT_FPS(total_fps, {});
        return SubModuleResult::SUCCESS;
    }
} // namespace sensor
