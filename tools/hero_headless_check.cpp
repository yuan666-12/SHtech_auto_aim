#include "gui_control.hpp"
#include <iostream>
#include <stdexcept>

// Normal-mode checks use a recording backend, so they require neither a desktop
// nor GTK. A second build removes these overrides and uses real OpenCV in
// headless mode with DISPLAY and WAYLAND_DISPLAY unset.
#ifndef HEADLESS_REAL_BACKEND
int calls[4]{};
namespace cv {
void imshow(const String&, InputArray) { ++calls[0]; }
void namedWindow(const String&, int) { ++calls[1]; }
int waitKey(int) { ++calls[2]; return 42; }
int createTrackbar(const String&, const String&, int*, int, TrackbarCallback, void*) {
    ++calls[3]; return 37;
}
}
#endif

int main(int argc, char** argv) {
    try {
        const bool expected = argc == 2 && std::string(argv[1]) == "1";
        if (autoaim_gui::headless() != expected)
            throw std::runtime_error("headless setting/default mismatch");
        const cv::Mat image(2, 2, CV_8UC1, cv::Scalar(0));
        int value = 12;
        autoaim_gui::namedWindow("headless check");
        const int trackbar = autoaim_gui::createTrackbar("test", "headless check", &value, 100);
        autoaim_gui::imshow("headless check", image);
        const int key = autoaim_gui::waitKey(0);
        if (value != 12) throw std::runtime_error("GUI suppression changed tuning value");
        if (expected && (key != -1 || trackbar != 0))
            throw std::runtime_error("headless GUI was not suppressed");
#ifndef HEADLESS_REAL_BACKEND
        for (int count : calls)
            if (count != (expected ? 0 : 1))
                throw std::runtime_error("wrong backend delegation count");
        if (!expected && (key != 42 || trackbar != 37))
            throw std::runtime_error("normal-mode return values changed");
#endif
        std::cout << "PASS headless=" << expected << "; no camera/UART opened\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
