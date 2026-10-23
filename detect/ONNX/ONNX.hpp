#ifndef ONNX_HPP
#define ONNX_HPP

#include "backend.hpp"
#include <onnxruntime_cxx_api.h>
#include <string>
#include <vector>
#include <memory>
#include <array>
#include <chrono>
#include <cstdint>

class ONNX : public BackEnd {
public:
    struct Timing {
        double preprocess_ms = 0, inference_ms = 0, postprocess_ms = 0, total_ms = 0;
    };
    explicit ONNX(const std::string &model_path);
    ~ONNX() override = default;
    void operator()(const cv::Mat &src, std::vector<bbox_t> &det) override;
    const Timing &last_timing() const { return last_timing_; }

private:
    void preprocess(const cv::Mat &src);
    void postprocess(const float* output_data, size_t output_count,
                     std::vector<bbox_t> &det, float scale_x, float scale_y);

    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo memory_info_;

    // ONNX 1.17+ 需要用 string 存名字，保持生命周期
    std::vector<std::string> input_names_str_;
    std::vector<std::string> output_names_str_;
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
    std::vector<int64_t> input_shape_;

    // One model instance is used by one detection worker. These buffers and
    // their tensor wrappers stay valid throughout the session's lifetime.
    std::vector<float> input_values_;
    Ort::Value input_tensor_{nullptr};
    std::array<cv::Mat, 3> input_planes_;
    cv::Mat resized_input_, float_input_;
    Timing last_timing_;
    int intra_threads_ = 0;
    bool profile_enabled_ = false, shape_logged_ = false;
    uint64_t profile_sequence_ = 0;

    static constexpr int input_w_ = 640;
    static constexpr int input_h_ = 512;
};

#endif // ONNX_HPP
