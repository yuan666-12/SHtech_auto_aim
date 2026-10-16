//
// Created for pipeline refactor - SensorSubModule
// Wraps original Sensor logic as SubModule (Camera only, hardware communication moved to hardware::TimedSerial)
//

#include "sensor_submodule.hpp"
#include "calibration_capture.hpp"

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
#warning ENABLE_HIKCAM
#include <hikcam/hikcam_wrapper.hpp>
#else
#warning
#warning DISABLE_HIKCAM
#endif

namespace sensor
{
    // SensorSubModule 实现
    SensorSubModule::SensorSubModule(const SensorConfig& config, 
                                    const std::string &VideoSource, 
                                    const std::string &flip_image, 
                                    pipeline::bridge::SensorFromSerialAttitudeBridge& attitude_bridge, 
                                    pipeline::bridge::SensorFromSerialRobotStatusBridge& status_bridge) 
        : SubModule(SubModuleName::SENSOR), config_(config), attitude_bridge(attitude_bridge), status_bridge(status_bridge)
    {
        LOGM_S("[sensor_submodule] constructing with video: %s", VideoSource.c_str());
        const bool is_camera_source = (VideoSource == "0");

        // 初始化视频源
        if (is_camera_source)
        {
#ifdef ENABLE_HIKCAM
            video = new HikCamWrapper();
#else
            throw std::runtime_error("camera source requested but ENABLE_HIKCAM is disabled");
#endif
        }
        else
        {
            video = new VideoWrapper(VideoSource);
        }

        if (!video)
        {
            throw std::runtime_error("failed to create video source wrapper");
        }

        // 初始化视频设备
        if (is_camera_source)
        {
            int retry_count = 0;
            while (!video->init())
            {
                ++retry_count;
                if (retry_count == 1 || retry_count % 10 == 0)
                {
                    LOGE_S("[sensor_submodule]Error: Initialize camera stream failed, retrying at 10Hz... (%d)", retry_count);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (retry_count > 0)
            {
                LOGM_S("[sensor_submodule] camera stream initialized after %d retries", retry_count);
            }
        }
        else if (!video->init())
        {
            throw std::runtime_error("failed to initialize video stream: " + VideoSource);
        }
        LOGM_S("[sensor_submodule] video initialized");

        // 设置图像翻转标志
        if (flip_image == "1")
        {
            is_image_input_flipped = true;
            LOGM_S("[sensor_submodule] Input image will be flipped");
        }
        else
        {
            is_image_input_flipped = false;
            // LOGM_S("[sensor_submodule] Input image will not be flipped");
        }

        LOGM_S("[sensor_submodule] construction completed");
    }

    SensorSubModule::~SensorSubModule()
    {
        if (video)
        {
            video->close();
            delete video;
        }
    }

    SubModuleResult SensorSubModule::process(std::shared_ptr<ThreadDataPack> data,
                                             const pipeline::BasicTask *parent)
    {
        auto t1 = std::chrono::steady_clock::now();

        // 读取图像
        bool state = video->read(data->frame, config_.debug.log_text);
        data->time = std::chrono::high_resolution_clock::now();
        const auto image_read_unix_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // 读取imu
        const auto attitude_message = attitude_bridge.get();
        const auto attitude_sampled_at = std::chrono::steady_clock::now();
        data->attitude = attitude_message.attitude;
        const bool attitude_fresh = calibration::fresh(
            attitude_message.received_at, attitude_sampled_at);
        data->robotstatus = status_bridge.get().robotstatus;

        if (!state)
        {
            LOGE_S("[sensor_submodule]Error: read image fail!");
            if (config_.debug.log_text)
            {
                LOGM_S("[sensor_submodule] Total frames handled: %d", data->index);
                LOGM_S("[sensor_submodule] ReOpen Camera");
            }
            video->close(config_.debug.log_text);
            video->init(config_.debug.log_text);
            // 在失败情况下，返回 false 表示不应该传递到下游
            return SubModuleResult::FAILURE;
        }

        if (data->frame.empty())
        {
            LOGE_S("[sensor_submodule] empty image");
            // 在空图像情况下，也返回 false
            return SubModuleResult::FAILURE;
        }

        if (is_image_input_flipped)
        {
            cv::flip(data->frame, data->frame, -1);
        }

        const char* validation_flag = std::getenv("AUTOAIM_VALIDATION_CAPTURE");
        const bool validation_capture = validation_flag && std::string(validation_flag) == "1";
        // One board definition and one pair writer for automatic and P-key capture.
        static int frames_since_save = 0;
        const bool auto_save_due = config_.save_image && config_.save_interval > 0 &&
                                  ++frames_since_save >= config_.save_interval;
        if (auto_save_due) frames_since_save = 0;
        bool board_checked = false;
        bool board_found = false;
        std::vector<cv::Point2f> calibration_corners;
        auto check_board = [&]() {
            if (!board_checked) {
                board_found = calibration::detect_board(data->frame, calibration_corners);
                board_checked = true;
            }
            return board_found;
        };

        static std::string capture_hint;
        static bool last_capture_ok = false;
        static std::chrono::steady_clock::time_point hint_until{};
        auto save_pair = [&]() {
            const char* dry_run = std::getenv("AUTOAIM_XUC_DRY_RUN");
            std::string error;
            uint64_t sample_id = 0;
            bool ok = false;
            if (!dry_run || std::string(dry_run) != "1")
                error = "capture requires AUTOAIM_XUC_DRY_RUN=1";
            else if (!attitude_fresh)
                error = "no fresh RX attitude (limit: 250 ms)";
            else if (!validation_capture && !check_board())
                error = "complete 9x7 board not found";
            else {
                static calibration::PairWriter writer;
                ok = writer.save(data->frame,
                    {data->attitude.yaw(), data->attitude.pitch(), data->attitude.roll()},
                    sample_id, error);
                if (ok) {
                    const auto directory = writer.directory().string();
                    capture_hint = "Saved pair " + std::to_string(sample_id);
                    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        attitude_sampled_at - attitude_message.received_at).count();
                    if (validation_capture) {
                        const std::string metadata_path = directory + "/frame_" +
                            std::to_string(sample_id) + ".json";
                        std::ofstream metadata(metadata_path);
                        metadata << std::setprecision(12)
                            << "{\n\"image_read_unix_ms\":" << image_read_unix_ms
                            << ",\n\"rx_age_ms\":" << age_ms
                            << ",\n\"yaw_deg\":" << data->attitude.yaw()
                            << ",\n\"pitch_deg\":" << data->attitude.pitch()
                            << ",\n\"roll_deg\":" << data->attitude.roll()
                            << ",\n\"board_detection_required\":false\n}\n";
                        metadata.flush();
                        if (!metadata)
                            LOGW_S("[sensor_submodule] pair saved, but A/B metadata write failed: %s",
                                   metadata_path.c_str());
                    }
                    LOGM_S("[sensor_submodule] saved pair: %s/frame_%llu.png + euler_%llu.txt; "
                           "yaw=%.6f pitch=%.6f roll=%.6f deg; RX age at image read=%lld ms",
                           directory.c_str(), static_cast<unsigned long long>(sample_id),
                           static_cast<unsigned long long>(sample_id), data->attitude.yaw(),
                           data->attitude.pitch(), data->attitude.roll(),
                           static_cast<long long>(age_ms));
                }
            }
            if (!ok) {
                capture_hint = "Not saved: " + error;
                LOGW_S("[sensor_submodule] calibration pair rejected: %s", error.c_str());
            }
            last_capture_ok = ok;
            hint_until = std::chrono::steady_clock::now() + std::chrono::seconds(3);
            return ok;
        };

        if (auto_save_due) save_pair();
        auto t2 = std::chrono::steady_clock::now();

        if (config_.debug.show_image) {
            cv::Mat im2show;
            const double scale = std::min(1.0, std::min(800.0 / data->frame.cols,
                                                       560.0 / data->frame.rows));
            if (scale < 1.0)
                cv::resize(data->frame, im2show, cv::Size(), scale, scale, cv::INTER_AREA);
            else im2show = data->frame.clone();

            cv::putText(im2show, validation_capture ? "A/B DRY_RUN / raw frame + RX" : "9x7 board / 20 mm", cv::Point(20, 35),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            cv::putText(im2show, "Stop, then press P to save image + Euler",
                        cv::Point(20, 70), cv::FONT_HERSHEY_SIMPLEX, 0.65,
                        cv::Scalar(0, 255, 0), 2);
            std::ostringstream attitude_text;
            attitude_text << std::fixed << std::setprecision(2)
                          << "Yaw " << data->attitude.yaw()
                          << "  Pitch " << data->attitude.pitch()
                          << "  Roll " << data->attitude.roll() << " deg";
            cv::putText(im2show, attitude_text.str(), cv::Point(20, 105),
                        cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 0), 2);
            const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                attitude_sampled_at - attitude_message.received_at).count();
            const std::string rx_text = attitude_fresh
                ? "RX fresh at image read: " + std::to_string(age_ms) + " ms"
                : "RX missing/stale - cannot save";
            cv::putText(im2show, rx_text, cv::Point(20, 140),
                        cv::FONT_HERSHEY_SIMPLEX, 0.65,
                        attitude_fresh ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);

            if (config_.calib_detect) {
                if (check_board()) {
                    std::vector<cv::Point2f> display_corners;
                    for (const auto& corner : calibration_corners)
                        display_corners.emplace_back(
                            corner.x * static_cast<float>(im2show.cols) / data->frame.cols,
                            corner.y * static_cast<float>(im2show.rows) / data->frame.rows);
                    cv::drawChessboardCorners(im2show, calibration::board_size, display_corners, true);
                }
                cv::putText(im2show, board_found ? "Detected: 63 corners" : "Complete board not detected",
                            cv::Point(20, 175), cv::FONT_HERSHEY_SIMPLEX, 0.65,
                            board_found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
            }
            if (std::chrono::steady_clock::now() < hint_until)
                cv::putText(im2show, capture_hint, cv::Point(20, 210),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            last_capture_ok ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 0, 255), 2);

            cv::imshow("sensor_submodule", im2show);
            const int key = cv::waitKey(1);
            if (key == 'p' || key == 'P') save_pair();
        }

        // 调试信息
        if (config_.debug.log_text)
        {
            CNT_FPS(total_fps, {});
            // LOGM_S("[sensor_submodule]Info: Idx = %d, Bytes = %d", data->index, data->frame.size().height * data->frame.size().width);
        }

        // 成功处理，返回 true 表示应该传递到下游
        return SubModuleResult::SUCCESS;
    }
}
