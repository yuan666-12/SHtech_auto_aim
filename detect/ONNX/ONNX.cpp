#include "ONNX.hpp"
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

// SKD250526 output: 8 corner offsets, confidence, 8 armor-class scores,
// 2 color scores and 2 armor-size scores.  The corner offsets require the
// same 640x512 grid decode used by the AXCL backend.
static constexpr int OUTPUT_DIM = 21;

// BackEnd 的常量是 private 的，这里用局部常量（值相同）
static constexpr float CONF_THRESH = 0.1f;
static constexpr float NMS_THRESH = 0.45f;
static constexpr int MAX_DET = 128;

ONNX::ONNX(const std::string &model_path)
    : env_(ORT_LOGGING_LEVEL_WARNING, "autoaim"),
      memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
{
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);

    Ort::AllocatorWithDefaultOptions allocator;

    // ONNX Runtime 1.17+ API
    auto input_name_ptr = session_->GetInputNameAllocated(0, allocator);
    auto output_name_ptr = session_->GetOutputNameAllocated(0, allocator);

    input_names_str_.push_back(std::string(input_name_ptr.get()));
    output_names_str_.push_back(std::string(output_name_ptr.get()));

    input_names_.push_back(input_names_str_[0].c_str());
    output_names_.push_back(output_names_str_[0].c_str());

    auto input_type_info = session_->GetInputTypeInfo(0);
    auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
    input_shape_ = input_tensor_info.GetShape();

    if (input_shape_[0] == -1) input_shape_[0] = 1;
}

void ONNX::preprocess(const cv::Mat &src, std::vector<float> &input_tensor_values) {
    cv::Mat resized, float_img;
    cv::resize(src, resized, cv::Size(input_w_, input_h_));
    resized.convertTo(float_img, CV_32FC3, 1.0 / 255.0);

    std::vector<cv::Mat> chw(3);
    cv::split(float_img, chw);
    for (int c = 0; c < 3; ++c) {
        input_tensor_values.insert(input_tensor_values.end(),
                                   (float*)chw[c].datastart,
                                   (float*)chw[c].dataend);
    }
}

void ONNX::postprocess(const float* output_data, size_t output_count,
                       std::vector<bbox_t> &det, float scale_x, float scale_y) {
    det.clear();
    if (output_count == 0 || output_data == nullptr) return;

    int num_boxes = static_cast<int>(output_count) / OUTPUT_DIM;
    if (num_boxes == 0) return;

    std::vector<cv::Rect> nms_boxes;
    std::vector<float> nms_scores;
    std::vector<bbox_t> candidates;
    nms_boxes.reserve(128);
    nms_scores.reserve(128);
    candidates.reserve(128);

    float max_confidence = -1e9f;
    int max_objectness_index = -1;

    int stride = 8;
    int x_center = 0;
    int y_center = 0;

    for (int i = 0; i < num_boxes; ++i) {
        const float *row = output_data + i * OUTPUT_DIM;
        if (row[8] > max_confidence) {
            max_confidence = row[8];
            max_objectness_index = i;
        }
        if (row[8] >= CONF_THRESH) {
            int class_id = 0;
            int color_id = 0;
            for (int c = 1; c < 8; ++c)
                if (row[9 + c] > row[9 + class_id]) class_id = c;
            if (row[18] > row[17]) color_id = 1;

            bbox_t box;
            box.confidence = row[8];
            box.color_id = color_id;
            box.tag_id = class_id;
            box.source = DetectionSource::NEURAL_NETWORK;
            float x_min = 1e5f, y_min = 1e5f, x_max = -1e5f, y_max = -1e5f;
            for (int j = 0; j < 4; ++j) {
                box.pts[j].x = (row[j * 2] * 2.0f * stride + x_center) * scale_x;
                box.pts[j].y = (row[j * 2 + 1] * 2.0f * stride + y_center) * scale_y;
            }
            // Model order: LT, LB, RT, RB; internal order: LT, LB, RB, RT.
            std::swap(box.pts[2], box.pts[3]);
            for (const auto &pt : box.pts) {
                x_min = std::min(x_min, pt.x);
                y_min = std::min(y_min, pt.y);
                x_max = std::max(x_max, pt.x);
                y_max = std::max(y_max, pt.y);
            }
            const float width = x_max - x_min;
            const float height = y_max - y_min;
            if (width > 1.0f && height > 1.0f) {
                candidates.push_back(box);
                nms_boxes.emplace_back(static_cast<int>(x_min - 0.1f * width),
                                       static_cast<int>(y_min - 0.1f * height),
                                       static_cast<int>(1.2f * width),
                                       static_cast<int>(1.2f * height));
                nms_scores.push_back(row[8]);
            }
        }

        x_center += stride;
        x_center = (x_center == 640) ? 0 : x_center;
        y_center += (x_center == 0) ? stride : 0;
        y_center = (y_center == 512) ? 0 : y_center;
        if (x_center == 0 && y_center == 0) stride *= 2;
    }

    static unsigned diagnostic_counter = 0;
    if ((diagnostic_counter++ % 30U) == 0U && max_objectness_index >= 0) {
        const float *best = output_data + max_objectness_index * OUTPUT_DIM;
        int best_class = 0;
        for (int c = 1; c < 8; ++c)
            if (best[9 + c] > best[9 + best_class]) best_class = c;
        const int best_color = best[18] > best[17] ? 1 : 0;
        std::cerr << "[ONNX] best: confidence=" << max_confidence
                  << " color=" << best_color
                  << " class=" << best_class
                  << " candidates=" << candidates.size() << std::endl;
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(nms_boxes, nms_scores, CONF_THRESH, NMS_THRESH, indices);
    det.reserve(std::min(indices.size(), static_cast<size_t>(MAX_DET)));
    for (const int index : indices) {
        if (det.size() >= MAX_DET) break;
        det.push_back(candidates[index]);
    }
    std::sort(det.begin(), det.end(), std::greater<bbox_t>());
}

void ONNX::operator()(const cv::Mat &src, std::vector<bbox_t> &det) {
    std::vector<float> input_tensor_values;
    preprocess(src, input_tensor_values);

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info_, input_tensor_values.data(), input_tensor_values.size(),
        input_shape_.data(), input_shape_.size());

    // 用 auto 接收，避免 Ort::Value 拷贝（拷贝构造函数已删除）
    auto output_tensors = session_->Run(
        Ort::RunOptions{nullptr},
        input_names_.data(), &input_tensor, 1,
        output_names_.data(), 1);

    Ort::Value& output_tensor = output_tensors[0];

    static bool shape_logged = false;
    if (!shape_logged) {
        const auto output_shape = output_tensor.GetTensorTypeAndShapeInfo().GetShape();
        std::cerr << "[ONNX] runtime output shape:";
        for (const auto dim : output_shape) std::cerr << ' ' << dim;
        std::cerr << std::endl;
        shape_logged = true;
    }

    float *output_data = output_tensor.GetTensorMutableData<float>();
    size_t output_count = output_tensor.GetTensorTypeAndShapeInfo().GetElementCount();

    float scale_x = static_cast<float>(src.cols) / input_w_;
    float scale_y = static_cast<float>(src.rows) / input_h_;

    postprocess(output_data, output_count, det, scale_x, scale_y);
}
