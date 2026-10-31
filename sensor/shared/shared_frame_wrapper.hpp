#pragma once
#include "cam_wrapper.hpp"
#include <hero_opencv_reader.hpp>
#include <memory>
#include <utility>

// Read-only consumer: no MVS handle, service controls, or image compression.
class SharedFrameWrapper final : public WrapperHead {
public:
    explicit SharedFrameWrapper(std::string path = {}) : path_(std::move(path)) {}
    bool init(bool = false) override {
        // Producer may start later. The reader rejects synthetic/stale frames.
        if (!reader_) reader_ = std::make_unique<hero::OpenCvReader>(path_);
        return true;
    }
    bool read(cv::Mat &image, bool = false) override {
        if (!reader_) init();
        hero::CapturedFrame frame;
        if (!reader_->next(frame)) return false;
        const auto ns = frame.meta.capture_monotonic_ns;
        if (generation_ == frame.meta.generation && last_ns_ && ns > last_ns_)
            fps_ = static_cast<int>(1000000000ULL / (ns - last_ns_));
        else fps_ = 0;
        last_ns_ = ns;
        generation_ = frame.meta.generation;
        received_at_ = frame.receipt_time();
        meta_ = frame.meta;
        size_ = frame.bgr.size();
        image = std::move(frame.bgr); // Independently owned, full-resolution BGR.
        return true;
    }
    std::chrono::steady_clock::time_point frame_received_at() const override { return received_at_; }
    uint64_t frame_number() const override { return meta_.source_frame_number; }
    int64_t host_timestamp() const override { return meta_.sdk_host_timestamp; }
    uint64_t device_timestamp() const override { return meta_.device_ticks; }
    cv::Size getSize() override { return size_; }
    int getFps() override { return fps_; }
    bool setBrightness(int) override { return false; } // Camera settings belong to the producer.
    bool close(bool = false) override {
        reader_.reset();
        received_at_ = {}; meta_ = {}; size_ = {}; last_ns_ = 0; generation_ = 0; fps_ = 0;
        return true;
    }
private:
    std::string path_;
    std::unique_ptr<hero::OpenCvReader> reader_;
    hero_frame_meta meta_{};
    cv::Size size_{};
    std::chrono::steady_clock::time_point received_at_{};
    uint64_t last_ns_ = 0, generation_ = 0;
    int fps_ = 0;
};
