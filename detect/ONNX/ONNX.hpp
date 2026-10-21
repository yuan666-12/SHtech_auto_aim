#ifndef ONNX_HPP
#define ONNX_HPP

#include "backend.hpp"
#include <onnxruntime_cxx_api.h>
#include <string>
#include <vector>
#include <memory>

class ONNX : public BackEnd {
public:
    explicit ONNX(const std::string &model_path);
    ~ONNX() override = default;
    void operator()(const cv::Mat &src, std::vector<bbox_t> &det) override;

private:
    void preprocess(const cv::Mat &src, std::vector<float> &input_tensor_values);
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

    static constexpr int input_w_ = 640;
    static constexpr int input_h_ = 512;
};

#endif // ONNX_HPP
