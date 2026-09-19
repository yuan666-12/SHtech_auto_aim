//
// Created for pipeline refactor - SensorSubModule
// Wraps original Sensor logic as SubModule (Camera only, hardware communication moved to hardware::TimedSerial)
//

#include "sensor_submodule.hpp"

// submodules
#include <chrono>
#include <cstdlib>
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

        // 读取imu
        data->attitude = attitude_bridge.get().attitude;
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

        // 自动保存原始帧用于相机标定（无头环境下按固定帧间隔保存，不依赖 GUI/按键）
        if (config_.save_image && config_.save_interval > 0)
        {
            static int frames_since_save = 0;
            static int saved_counter = 0;
            static bool dir_created = false;

            if (++frames_since_save >= config_.save_interval)
            {
                frames_since_save = 0;
                const std::string out_dir = "./calib_frames";
                if (!dir_created)
                {
                    system(("mkdir -p " + out_dir).c_str());
                    dir_created = true;
                }
                const std::string filename = out_dir + "/raw_" + std::to_string(saved_counter++) + ".png";
                cv::imwrite(filename, data->frame);
                LOGM_S("[sensor_submodule] saved raw frame: %s", filename.c_str());
            }
        }

        auto t2 = std::chrono::steady_clock::now();

        // 显示图像（如果需要）；按 p 保存原始帧用于标定
        if (config_.debug.show_image)
        {
            static int manual_saved_counter = 0;
            static bool save_dir_created = false;
            static int save_hint_frames = 0;    // 剩余显示"保存成功"提示的帧数
            static std::string last_saved_name; // 最近保存的文件名

            // 缩小到适配屏幕的显示尺寸（图像 1440x1080，屏幕 1024x768 且顶部有面板，需同时限制宽高）
            cv::Mat im2show;
            const int max_display_width = 800;
            const int max_display_height = 560;
            double scale = 1.0;
            const double scale_w = static_cast<double>(max_display_width) / data->frame.cols;
            const double scale_h = static_cast<double>(max_display_height) / data->frame.rows;
            if (scale_w < scale) scale = scale_w;
            if (scale_h < scale) scale = scale_h;
            if (scale < 1.0)
            {
                cv::resize(data->frame, im2show, cv::Size(), scale, scale, cv::INTER_AREA);
            }
            else
            {
                im2show = data->frame.clone();
            }

            // 常驻提示
            cv::putText(im2show, "CALIBRATION MODE", cv::Point(20, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            cv::putText(im2show, "Press 'p' to save frame", cv::Point(20, 80),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

            // 保存成功提示（短暂显示约 3 秒）
            if (save_hint_frames > 0)
            {
                cv::putText(im2show, "Saved: " + last_saved_name, cv::Point(20, 120),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
                --save_hint_frames;
            }

            // 棋盘格角点检测与可视化（11x8 内角点），帮助确认棋盘格识别与对焦是否到位
            {
                static const cv::Size pattern_size(11, 8);
                cv::Mat gray;
                cv::cvtColor(data->frame, gray, cv::COLOR_BGR2GRAY);
                cv::Mat gray_small;
                cv::resize(gray, gray_small, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
                std::vector<cv::Point2f> corners;
                const bool found = cv::findChessboardCorners(
                    gray_small, pattern_size, corners,
                    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
                if (found)
                {
                    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
                    cv::cornerSubPix(gray_small, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
                    // 角点从 0.5 降采样图放大回原图，再缩放到显示图
                    std::vector<cv::Point2f> corners_scaled;
                    corners_scaled.reserve(corners.size());
                    for (const auto &c : corners)
                        corners_scaled.emplace_back(static_cast<float>(c.x * 2.0 * scale),
                                                    static_cast<float>(c.y * 2.0 * scale));
                    cv::drawChessboardCorners(im2show, pattern_size, corners_scaled, true);
                    cv::putText(im2show, "Detected: " + std::to_string(corners.size()) + " corners",
                                cv::Point(20, 160), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
                }
                else
                {
                    cv::putText(im2show, "No chessboard detected", cv::Point(20, 160),
                                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                }
            }

            cv::imshow("sensor_submodule", im2show);
            int key = cv::waitKey(1);
            if (key == 'p' || key == 'P')
            {
                const std::string out_dir = "./calib_frames";
                if (!save_dir_created)
                {
                    system(("mkdir -p " + out_dir).c_str());
                    save_dir_created = true;
                }
                const std::string filename = out_dir + "/raw_" + std::to_string(manual_saved_counter++) + ".png";
                cv::imwrite(filename, data->frame);
                last_saved_name = "raw_" + std::to_string(manual_saved_counter - 1) + ".png";
                save_hint_frames = 30;
                LOGM_S("[sensor_submodule] saved raw frame (key p): %s", filename.c_str());
            }
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