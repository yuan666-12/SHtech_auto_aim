// Copyright (C) 2025
// Licensed under the MIT License.

#include "gui_control.hpp"
#include "armor_corner_optimizer.hpp"
#include <chrono>

namespace detect {
    ArmorCornerOptimizer::ArmorCornerOptimizer(const bool adjust_, const int &bin_thres,
                                               const LightParams &light_params,
                                               const YoloModelCharacteristics &yolo_params)
        : binary_thres(bin_thres), light_params(light_params), yolo_params(yolo_params) {
    }

    float ArmorCornerOptimizer::vertical_angle(const cv::Vec2f &vector) {
        float angle = std::abs(std::atan2(vector[0], vector[1]) * 180.0f / CV_PI);
        return angle > 90.0f ? 180.0f - angle : angle;
    }

    ArmorCornerOptimizer::LightSearch
    ArmorCornerOptimizer::describe_light(const cv::Point2f &top, const cv::Point2f &bottom, cv::Size image_size) {
        LightSearch light;
        light.center = (top + bottom) * 0.5f;
        const cv::Vec2f vector = bottom - top;
        light.height = std::abs(vector[1]);
        light.angle = vertical_angle(vector);
        const float width = std::abs(vector[0]);
        const float length =
            std::max(float(yolo_params.min_light_height), std::min(light.height, float(yolo_params.max_light_height)));
        light.roi = validateRect(calculateLightRoi(light.center, light.height, width, length), image_size.width,
                                 image_size.height);
        return light;
    }

    bool ArmorCornerOptimizer::roi_inside(const cv::Rect &roi, cv::Size image_size) {
        return roi.x > 0 && roi.y > 0 && roi.width > 0 && roi.height > 0 && roi.x + roi.width < image_size.width &&
               roi.y + roi.height < image_size.height;
    }

    void ArmorCornerOptimizer::refine_light(const LightSearch &search, cv::Point2f &top, cv::Point2f &bottom,
                                            CornerRefineCallStats *stats) {
        const int best = selectBestLightBar(search.candidates, search.center, search.height, search.angle);
        if (best < 0) return;
        top = search.candidates[best].top;
        bottom = search.candidates[best].bottom;
        if (stats) ++stats->successful_lights;
    }

    bool ArmorCornerOptimizer::compatible_lights(const std::array<cv::Point2f, 4> &corners) {
        const cv::Vec2f left = corners[1] - corners[0], right = corners[2] - corners[3];
        const float left_length = cv::norm(left), right_length = cv::norm(right);
        const float ratio = std::min(left_length, right_length) / std::max(left_length, right_length);
        return std::isfinite(ratio) && ratio >= 0.7f && std::abs(vertical_angle(left) - vertical_angle(right)) <= 5;
    }

    std::optional<std::array<cv::Point2f, 4>> ArmorCornerOptimizer::optimizeCorners(const cv::Mat &input,
                                                                                    const cv::Point2f yolo_corners[],
                                                                                    bool show,
                                                                                    CornerRefineCallStats *stats) {
        using Clock = std::chrono::steady_clock;
        const auto milliseconds = [](Clock::time_point start) {
            return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        };
        if (stats) *stats = {};
        if (input.empty()) return std::nullopt;
        std::array<cv::Point2f, 4> corners;
        std::copy_n(yolo_corners, corners.size(), corners.begin());
        auto started = Clock::now();
        auto left = describe_light(corners[0], corners[1], input.size());
        auto right = describe_light(corners[3], corners[2], input.size());
        if (stats) stats->roi_ms = milliseconds(started);
        if (!roi_inside(left.roi, input.size()) || !roi_inside(right.roi, input.size())) return std::nullopt;
        started = Clock::now();
        left.binary = preprocessImage(input, left.roi);
        right.binary = preprocessImage(input, right.roi);
        if (stats) stats->preprocess_ms = milliseconds(started);
        started = Clock::now();
        left.candidates = findLightBars(left.binary, left.roi);
        right.candidates = findLightBars(right.binary, right.roi);
        if (stats) {
            stats->find_light_ms = milliseconds(started);
            stats->candidate_light_bars = left.candidates.size() + right.candidates.size();
        }
        if (left.candidates.empty() || right.candidates.empty()) return std::nullopt;
        started = Clock::now();
        refine_light(left, corners[0], corners[1], stats);
        refine_light(right, corners[3], corners[2], stats);
        if (stats) stats->select_ms = milliseconds(started);
        started = Clock::now();
        const bool valid = compatible_lights(corners);
        if (stats) stats->final_check_ms = milliseconds(started);
        if (!valid) return std::nullopt;
        if (show) {
            started = Clock::now();
            autoaim_gui::imshow("ROI Visualization", visualizeROIs(input, left.roi, right.roi));
            autoaim_gui::imshow("Binary Visualization",
                                visualizeBinaryResults(input, left.binary, right.binary, left.roi, right.roi));
            if (stats) stats->visualize_ms = milliseconds(started);
        }
        return corners;
    }

    cv::Rect ArmorCornerOptimizer::calculateLightRoi(const cv::Point2f &center, float roi_height, float roi_width,
                                                     float light_length) {
        roi_height += light_length * yolo_params.roi_height_multiplier;
        roi_width += light_length * yolo_params.roi_width_multiplier;
        // Create ROI with adaptive size
        cv::Rect roi(int(center.x - roi_width * 0.5f), int(center.y - roi_height * 0.5f), int(roi_width),
                     int(roi_height));

        // Ensure minimum ROI size
        roi.width = std::max(yolo_params.min_roi_size, roi.width);
        roi.height = std::max(yolo_params.min_roi_size, roi.height);

        return roi;
    }

    cv::Rect ArmorCornerOptimizer::validateRect(const cv::Rect &rect, int img_width, int img_height) {
        cv::Rect valid_rect = rect;

        // Ensure x, y are non-negative
        valid_rect.x = std::max(0, valid_rect.x);
        valid_rect.y = std::max(0, valid_rect.y);

        // Ensure width and height don't exceed image dimensions
        valid_rect.width = std::min(img_width - valid_rect.x, valid_rect.width);
        valid_rect.height = std::min(img_height - valid_rect.y, valid_rect.height);

        return valid_rect;
    }

    int ArmorCornerOptimizer::selectBestLightBar(const std::vector<LightBar> &light_bars,
                                                 const cv::Point2f &yolo_center, float expected_height,
                                                 float expected_angle) {
        int best_idx = 0;
        float best_score = std::numeric_limits<float>::max();

        for (size_t i = 0; i < light_bars.size(); ++i) {
            // Calculate distance between centers (normalized by expected height)
            float center_dist = cv::norm(light_bars[i].center - yolo_center) / expected_height;

            // Calculate height difference ratio
            float height_diff = std::abs(light_bars[i].length - expected_height) / expected_height;

            // Calculate angle difference (normalized to 0-1 range, assuming max 90 degree difference)
            float angle_diff = std::abs(light_bars[i].tilt_angle - expected_angle) / 90.0f;

            // Combined weighted score (lower is better)
            float score = center_dist * yolo_params.center_distance_weight +
                          height_diff * yolo_params.length_difference_weight +
                          angle_diff * yolo_params.angle_difference_weight;

            if (score < best_score) {
                best_score = score;
                best_idx = i;
            }
        }

        if (best_score > 0.3) {
            best_idx = -1;
        }

        return best_idx;
    }

    cv::Mat ArmorCornerOptimizer::preprocessImage(const cv::Mat &rgb_img, const cv::Rect &roi) {
        // Extract ROI from the image
        cv::Mat roi_img = rgb_img(roi);

        // Fast green channel extraction
        // cv::Mat green_channel(roi_img.rows, roi_img.cols, CV_8UC1);
        // for (int i = 0; i < roi_img.rows; i++) {
        //   const uchar* src = roi_img.ptr<uchar>(i);
        //   uchar* dst = green_channel.ptr<uchar>(i);
        //   for (int j = 0; j < roi_img.cols; j++) {
        //     *dst++ = src[1]; // Green channel in BGR
        //     src += 3;        // Move to next pixel
        //   }
        // }

        cv::Mat green_channel;
        cv::cvtColor(roi_img, green_channel, cv::COLOR_BGR2GRAY);
        // Calculate the light bar area: length × (length/4) considering 4:1 ratio
        // Apply threshold
        cv::Mat binary_img;
        cv::threshold(green_channel, binary_img, binary_thres, 255, cv::THRESH_BINARY);

        // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 3));
        // cv::morphologyEx(binary_img, binary_img, cv::MORPH_OPEN, kernel);

        return binary_img;
    }

    std::vector<LightBar> ArmorCornerOptimizer::findLightBars(const cv::Mat &binary_img, const cv::Rect &roi) {
        // Find contours in binary image
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<LightBar> light_bars;
        light_bars.reserve(contours.size());

        // Process each contour
        for (const auto &contour : contours) {
            // Skip small contours
            if (contour.size() < 5) continue;

            // Get bounding rectangle and rotated rectangle
            auto b_rect = cv::boundingRect(contour);
            auto r_rect = cv::minAreaRect(contour);

            // Create mask for the contour
            cv::Mat mask = cv::Mat::zeros(b_rect.size(), CV_8UC1);
            std::vector<cv::Point> mask_contour;
            mask_contour.reserve(contour.size());
            for (const auto &p : contour) {
                mask_contour.emplace_back(p - cv::Point(b_rect.x, b_rect.y));
            }
            cv::fillPoly(mask, {mask_contour}, 255);

            // Find non-zero points in mask
            std::vector<cv::Point> points;
            cv::findNonZero(mask, points);

            // Check fill ratio
            bool is_fill_rotated_rect =
                points.size() / (r_rect.size.width * r_rect.size.height) > light_params.min_fill_ratio;

            if (!is_fill_rotated_rect) continue;

            // Fit line to points
            cv::Vec4f line_params;
            cv::fitLine(points, line_params, cv::DIST_L2, 0, 0.01, 0.01);

            // Calculate top and bottom points
            cv::Point2f top, bottom;
            float angle_k;

            if (int(line_params[0] * 100) == 100 || int(line_params[1] * 100) == 0) {
                // Vertical line
                top = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y) + cv::Point2f(roi.x, roi.y);
                bottom = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y + b_rect.height) + cv::Point2f(roi.x, roi.y);
                angle_k = 0;
            } else {
                // Angled line
                float k = line_params[1] / line_params[0];
                float b = (line_params[3] + b_rect.y) - k * (line_params[2] + b_rect.x);

                // Calculate line endpoints
                top = cv::Point2f((b_rect.y - b) / k, b_rect.y) + cv::Point2f(roi.x, roi.y);
                bottom = cv::Point2f((b_rect.y + b_rect.height - b) / k, b_rect.y + b_rect.height) +
                         cv::Point2f(roi.x, roi.y);

                // Calculate angle
                angle_k = std::abs(std::atan(k) / CV_PI * 180 - 90);
                if (angle_k > 90) {
                    angle_k = 180 - angle_k;
                }
            }

            // Create light bar
            LightBar light(top, bottom);
            // light.width = cv::norm(cv::Point2f(r_rect.size.width, r_rect.size.height)) / 2.0;
            light.width = points.size() / light.length;
            light.tilt_angle = angle_k;

            // Validate light bar
            if (isValidLightBar(light)) {
                light_bars.push_back(light);
            }
        }

        return light_bars;
    }

    bool ArmorCornerOptimizer::isValidLightBar(const LightBar &light) {
        // Calculate width/length ratio
        float ratio = light.width / light.length;

        bool ratio_ok = light_params.min_ratio < ratio && ratio < light_params.max_ratio;

        // Check angle
        bool angle_ok = light.tilt_angle < light_params.max_angle;

        // if (ratio_ok == false) {
        //   std::cout << "Light bar rejected due to ratio: " << ratio << std::endl;
        // }

        // if (angle_ok == false) {
        //   std::cout << "Light bar rejected due to angle: " << light.tilt_angle << std::endl;
        // }

        return ratio_ok && angle_ok;
    }

    cv::Mat ArmorCornerOptimizer::visualizeROIs(const cv::Mat &input, const cv::Rect &left_roi,
                                                const cv::Rect &right_roi) {
        // 创建黑色背景图像（与输入图像相同大小）
        cv::Mat visualization = cv::Mat::zeros(input.size(), input.type());

        // 确保ROI在图像范围内
        cv::Rect valid_left_roi = validateRect(left_roi, input.cols, input.rows);
        cv::Rect valid_right_roi = validateRect(right_roi, input.cols, input.rows);

        // 将原始图像的ROI复制到可视化图像的对应位置
        if (valid_left_roi.width > 0 && valid_left_roi.height > 0) {
            input(valid_left_roi).copyTo(visualization(valid_left_roi));
        }

        if (valid_right_roi.width > 0 && valid_right_roi.height > 0) {
            input(valid_right_roi).copyTo(visualization(valid_right_roi));
        }

        // 绘制ROI边界框
        cv::rectangle(visualization, valid_left_roi, cv::Scalar(0, 255, 0), 2);
        cv::rectangle(visualization, valid_right_roi, cv::Scalar(0, 255, 0), 2);

        return visualization;
    }

    cv::Mat ArmorCornerOptimizer::visualizeBinaryResults(const cv::Mat &input, const cv::Mat &left_binary,
                                                         const cv::Mat &right_binary, const cv::Rect &left_roi,
                                                         const cv::Rect &right_roi) {
        // 创建黑色背景图像（与输入图像相同大小）
        cv::Mat visualization = cv::Mat::zeros(input.size(), input.type());

        // 确保ROI在图像范围内
        cv::Rect valid_left_roi = validateRect(left_roi, input.cols, input.rows);
        cv::Rect valid_right_roi = validateRect(right_roi, input.cols, input.rows);

        // 将二值化结果转换为彩色图像（白色区域将显示为绿色）
        if (valid_left_roi.width > 0 && valid_left_roi.height > 0 && left_binary.cols == valid_left_roi.width &&
            left_binary.rows == valid_left_roi.height) {

            // 为了更好的可视化，将二值图像转换为彩色图像
            cv::Mat colored_binary = cv::Mat::zeros(left_binary.size(), CV_8UC3);
            for (int y = 0; y < left_binary.rows; ++y) {
                for (int x = 0; x < left_binary.cols; ++x) {
                    if (left_binary.at<uchar>(y, x) > 0) {
                        colored_binary.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0); // 绿色表示二值化后的亮区
                    }
                }
            }

            // 复制到可视化图像中
            colored_binary.copyTo(visualization(valid_left_roi));
        }

        if (valid_right_roi.width > 0 && valid_right_roi.height > 0 && right_binary.cols == valid_right_roi.width &&
            right_binary.rows == valid_right_roi.height) {

            // 为了更好的可视化，将二值图像转换为彩色图像
            cv::Mat colored_binary = cv::Mat::zeros(right_binary.size(), CV_8UC3);
            for (int y = 0; y < right_binary.rows; ++y) {
                for (int x = 0; x < right_binary.cols; ++x) {
                    if (right_binary.at<uchar>(y, x) > 0) {
                        colored_binary.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0); // 绿色表示二值化后的亮区
                    }
                }
            }

            // 复制到可视化图像中
            colored_binary.copyTo(visualization(valid_right_roi));
        }

        // 绘制ROI边界框
        cv::rectangle(visualization, valid_left_roi, cv::Scalar(0, 255, 255), 2);
        cv::rectangle(visualization, valid_right_roi, cv::Scalar(0, 255, 255), 2);

        return visualization;
    }

} // namespace detect
