#pragma once
#include <opencv2/core.hpp>
#include <charconv>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace autoaim_runtime {
inline int parse_opencv_threads(const char *value, int fallback = 2) {
    if (!value) return fallback;
    int threads = 0;
    const auto end = value + std::strlen(value);
    const auto parsed = std::from_chars(value, end, threads);
    if (parsed.ec != std::errc{} || parsed.ptr != end || threads < -1 || threads > 64)
        throw std::invalid_argument("AUTOAIM_OPENCV_THREADS must be an integer in [-1,64]");
    return threads;
}
struct OpenCVThreadConfig { int requested, effective; };
// Call once at startup, before camera/pipeline workers use OpenCV.
inline OpenCVThreadConfig configure_opencv_threads() {
    const int requested = parse_opencv_threads(std::getenv("AUTOAIM_OPENCV_THREADS"));
    cv::setNumThreads(requested);
    return {requested, cv::getNumThreads()};
}
}
