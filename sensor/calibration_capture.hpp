#ifndef SENSOR_CALIBRATION_CAPTURE_HPP
#define SENSOR_CALIBRATION_CAPTURE_HPP

#include <opencv2/opencv.hpp>
#include <array>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <fcntl.h>
#include <unistd.h>

namespace sensor::calibration {
inline const cv::Size board_size{9, 7};
inline constexpr double square_m = 0.020;
inline constexpr auto max_rx_age = std::chrono::milliseconds(250);

inline bool fresh(std::chrono::steady_clock::time_point received,
                  std::chrono::steady_clock::time_point now) {
    return received != std::chrono::steady_clock::time_point{} &&
           now >= received && now - received <= max_rx_age;
}

inline bool detect_board(const cv::Mat& frame, std::vector<cv::Point2f>& corners) {
    if (frame.empty()) return false;
    cv::Mat gray, small;
    if (frame.channels() == 1) gray = frame;
    else cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::resize(gray, small, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
    if (!cv::findChessboardCorners(small, board_size, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE)) return false;
    // Use the actual resize ratio, including odd input dimensions.
    for (auto& point : corners) {
        point.x *= static_cast<float>(gray.cols) / small.cols;
        point.y *= static_cast<float>(gray.rows) / small.rows;
    }
    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));
    return corners.size() == static_cast<size_t>(board_size.area());
}

class PairWriter {
public:
    explicit PairWriter(std::filesystem::path directory = {}) : directory_(std::move(directory)) {}
    const std::filesystem::path& directory() const { return directory_; }

    bool save(const cv::Mat& frame, const std::array<double, 3>& ypr,
              uint64_t& id, std::string& error) {
        error.clear();
        if (frame.empty() || !std::all_of(ypr.begin(), ypr.end(),
                [](double value) { return std::isfinite(value); })) {
            error = "empty image or non-finite yaw/pitch/roll";
            return false;
        }
        try {
            prepare();
            std::vector<unsigned char> png;
            if (!cv::imencode(".png", frame, png)) {
                error = "PNG encoding failed"; return false;
            }
            std::ostringstream text;
            text << std::fixed << std::setprecision(6)
                 << ypr[0] << ' ' << ypr[1] << ' ' << ypr[2] << '\n';
            const std::string euler = text.str();
            for (;;) {
                id = next_id_++;
                const auto image_path = directory_ / ("frame_" + std::to_string(id) + ".png");
                const auto euler_path = directory_ / ("euler_" + std::to_string(id) + ".txt");
                if (std::filesystem::exists(image_path) || std::filesystem::exists(euler_path)) continue;
                const int image_fd = ::open(image_path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
                if (image_fd < 0) {
                    if (errno == EEXIST) continue;
                    error = "cannot create image: " + std::string(std::strerror(errno)); return false;
                }
                const int euler_fd = ::open(euler_path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
                if (euler_fd < 0) {
                    const int cause = errno;
                    ::close(image_fd);
                    ::unlink(image_path.c_str()); // Only the file created by this attempt.
                    if (cause == EEXIST) continue;
                    error = "cannot create Euler file: " + std::string(std::strerror(cause)); return false;
                }
                const bool image_ok = write_close(image_fd, png.data(), png.size());
                const bool euler_ok = write_close(euler_fd, euler.data(), euler.size());
                if (!image_ok || !euler_ok) {
                    ::unlink(image_path.c_str());
                    ::unlink(euler_path.c_str());
                    error = "pair write/flush failed; newly created pair removed";
                    return false;
                }
                return true;
            }
        } catch (const std::exception& exception) {
            error = exception.what(); return false;
        }
    }

private:
    static bool write_close(int fd, const void* data, size_t size) {
        const auto* bytes = static_cast<const unsigned char*>(data);
        bool ok = true;
        while (size > 0) {
            const ssize_t count = ::write(fd, bytes, size);
            if (count < 0 && errno == EINTR) continue;
            if (count <= 0) { ok = false; break; }
            bytes += count; size -= static_cast<size_t>(count);
        }
        if (ok && ::fsync(fd) != 0) ok = false;
        if (::close(fd) != 0) ok = false;
        return ok;
    }

    void prepare() {
        if (prepared_) return;
        if (directory_.empty()) {
            const char* configured = std::getenv("AUTOAIM_CALIBRATION_DIR");
            if (configured && *configured) directory_ = configured;
            else {
                const auto stamp = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();
                directory_ = "./hero_extrinsic_session_" + std::to_string(stamp) +
                             "_" + std::to_string(::getpid());
            }
        }
        std::filesystem::create_directories(directory_);
        // Resume after the largest existing image OR pose ID, including incomplete pairs.
        for (const auto& entry : std::filesystem::directory_iterator(directory_)) {
            const auto name = entry.path().filename().string();
            if (name.rfind("frame_", 0) != 0 && name.rfind("euler_", 0) != 0) continue;
            const auto dot = name.find('.', 6);
            if (dot == std::string::npos ||
                (name.substr(dot) != ".png" && name.substr(dot) != ".txt")) continue;
            const auto number = name.substr(6, dot - 6);
            if (number.empty() || number.find_first_not_of("0123456789") != std::string::npos) continue;
            try { next_id_ = std::max(next_id_, static_cast<uint64_t>(std::stoull(number) + 1)); }
            catch (const std::exception&) {}
        }
        prepared_ = true;
    }

    std::filesystem::path directory_;
    uint64_t next_id_ = 0;
    bool prepared_ = false;
};
} // namespace sensor::calibration
#endif
