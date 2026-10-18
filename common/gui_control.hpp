#pragma once

#include <cstdlib>
#include <cstring>
#include <opencv2/highgui.hpp>

namespace autoaim_gui {
// Read once per process, before any worker can open a window. Unset/0 keeps
// the existing display and adjustment configuration unchanged.
inline bool headless() {
    static const bool value = [] {
        const char* text = std::getenv("AUTOAIM_HEADLESS");
        return text && std::strcmp(text, "1") == 0;
    }();
    return value;
}

inline void imshow(const cv::String& name, cv::InputArray image) {
    if (!headless()) cv::imshow(name, image);
}

inline void namedWindow(const cv::String& name, int flags = cv::WINDOW_AUTOSIZE) {
    if (!headless()) cv::namedWindow(name, flags);
}

inline int waitKey(int delay = 0) {
    // No GUI and no keyboard events; never block on a display-less waitKey(0).
    return headless() ? -1 : cv::waitKey(delay);
}

inline int createTrackbar(const cv::String& name, const cv::String& window,
                         int* value, int count,
                         cv::TrackbarCallback on_change = nullptr,
                         void* userdata = nullptr) {
    if (headless()) return 0;
    return cv::createTrackbar(name, window, value, count, on_change, userdata);
}
} // namespace autoaim_gui
