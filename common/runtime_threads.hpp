#pragma once
#include "thread_config.hpp"
#include <opencv2/core.hpp>
#include <cstdlib>

namespace autoaim_runtime {
inline int parse_opencv_threads(const char *value, int fallback = 2) {
    return parse_thread_count(value, fallback, -1, 64, "AUTOAIM_OPENCV_THREADS");
}
struct OpenCVThreadConfig { int requested, effective; };
// Call once at startup, before camera/pipeline workers use OpenCV.
inline OpenCVThreadConfig configure_opencv_threads() {
    const int requested = parse_opencv_threads(std::getenv("AUTOAIM_OPENCV_THREADS"));
    cv::setNumThreads(requested);
    return {requested, cv::getNumThreads()};
}
}
