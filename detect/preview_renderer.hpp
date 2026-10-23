#pragma once
#include "backend.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>

namespace detect {
// Presentation only. Source pixels and detector coordinates stay untouched.
class PreviewRenderer {
    cv::Mat image_;
public:
    const cv::Mat &render(const cv::Mat &source, const std::vector<bbox_t> &boxes) {
        if (source.empty()) { image_.release(); return image_; }
        CV_Assert(source.datastart != image_.datastart);
        const double scale = std::min({1.0, 800.0 / source.cols, 560.0 / source.rows});
        const cv::Size size(cvRound(source.cols * scale), cvRound(source.rows * scale));
        if (size == source.size()) source.copyTo(image_);
        else cv::resize(source, image_, size, 0, 0, cv::INTER_LINEAR);
        const float sx = static_cast<float>(image_.cols) / source.cols;
        const float sy = static_cast<float>(image_.rows) / source.rows;
        const int line_width = std::max(1, cvRound(4 * scale));
        const int text_width = std::max(1, cvRound(3 * scale));
        static const cv::Scalar colors[] = {{0, 0, 255}, {255, 0, 0}, {255, 255, 255}};
        static const char *labels[] = {"RED", "BLUE", "GRAY"};
        for (const auto &box : boxes) {
            const int color = box.color_id >= 0 && box.color_id < 3 ? box.color_id : 2;
            cv::Point2f points[4];
            for (int i = 0; i < 4; ++i) points[i] = {box.pts[i].x * sx, box.pts[i].y * sy};
            for (int i = 0; i < 4; ++i)
                cv::line(image_, points[i], points[(i + 1) % 4], colors[color], line_width, cv::LINE_AA);
            const std::string label = std::string(labels[color]) + " id=" + std::to_string(box.tag_id);
            cv::putText(image_, label, points[0], cv::FONT_HERSHEY_SIMPLEX, scale,
                        colors[color], text_width, cv::LINE_AA);
        }
        return image_;
    }
};
}
