//
// Created for pipeline refactor - DetectSubModule
// Wraps original Detect logic as SubModule
//

#include "gui_control.hpp"
#include "detect_submodule.hpp"
#include "chrono"
#include <iostream>
#include <filesystem>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#if INFERENCE_BACKEND_TYPE == 1
#include "ONNX/ONNX.hpp"
#elif INFERENCE_BACKEND_TYPE == 2
#include "TensorRT/TRTModule.hpp"
#elif INFERENCE_BACKEND_TYPE == 3
#include "AXCL/AXCL.hpp"
#endif

namespace detect {
    static inline void map_bbox_to_source(const DetectInputMap &map, bbox_t &bbox) {
        if (!map.valid || map.dst_size.width <= 0 || map.dst_size.height <= 0) {
            return;
        }

        const float scale_x = static_cast<float>(map.src_roi.width) / static_cast<float>(map.dst_size.width);
        const float scale_y = static_cast<float>(map.src_roi.height) / static_cast<float>(map.dst_size.height);

        for (auto &pt : bbox.pts) {
            pt.x = map.src_roi.x + pt.x * scale_x;
            pt.y = map.src_roi.y + pt.y * scale_y;
        }
    }

    DetectSubModule::DetectSubModule(const DetectConfig &config, const std::string &OnnxFileName)
        : SubModule(SubModuleName::DETECT), config_(config) {
        LOGM_S("[detect] constructing with model: %s", OnnxFileName.c_str());
#if INFERENCE_BACKEND_TYPE == 1
        model.reset(new ONNX(OnnxFileName));
#elif INFERENCE_BACKEND_TYPE == 2
        model.reset(new TRTModule(OnnxFileName));
#elif INFERENCE_BACKEND_TYPE == 3
        model.reset(new AXCL(OnnxFileName));
#else
#error "Invalid INFERENCE_BACKEND_TYPE"
#endif
        const char *profile = std::getenv("AUTOAIM_PREVIEW_PROFILE");
        profile_preview_ = profile && std::strcmp(profile, "1") == 0;
        LOGM_S("[detect] model loaded");
    }

    void DetectSubModule::show_preview(const ThreadDataPack &data) {
        using Clock = std::chrono::steady_clock;
        const auto started = Clock::now();
        const cv::Mat &image = preview_renderer_.render(data.frame, data.bboxes);
        if (image.empty()) return;
        const auto rendered = Clock::now();
        autoaim_gui::imshow("detect_submodule", image);
        const auto submitted = Clock::now();
        const int key = autoaim_gui::waitKey(1);
        const auto events_done = Clock::now();
        if (key == 'p' || key == 'P') {
            std::filesystem::create_directories("./frames/");
            cv::imwrite("./frames/frame_" + std::to_string(preview_snapshot_sequence_++) + ".png", image);
        }
        const auto completed = Clock::now();
        const auto ms = [](auto a, auto z) { return std::chrono::duration<double,std::milli>(z-a).count(); };
        if (profile_preview_ && logfile && logfile->out && logfile->out != stdout && logfile->out != stderr)
            std::fprintf(logfile->out,
                "[DIAG_PREVIEW] frame_index=%d render_ms=%.4f submit_ms=%.4f events_ms=%.4f "
                "snapshot_ms=%.4f total_ms=%.4f events_per_frame=1 preview_w=%d preview_h=%d opencv_threads=%d\n",
                data.index, ms(started,rendered), ms(rendered,submitted), ms(submitted,events_done),
                ms(events_done,completed), ms(started,completed), image.cols, image.rows, cv::getNumThreads());
    }

    SubModuleResult DetectSubModule::process(std::shared_ptr<ThreadDataPack> data, const pipeline::BasicTask *parent) {
        std::vector<bbox_t> output_bboxes;
        data->bboxes.clear();
        auto t2 = std::chrono::steady_clock::now();

        if (!data->detect_input_map.valid || data->detect_input.empty()) {
            LOGE_S("[detect] missing preprocess output");
            return SubModuleResult::FAILURE;
        }

        (*model)(data->detect_input, output_bboxes);

        auto t3 = std::chrono::steady_clock::now();

        for (auto &t_bbox : output_bboxes) {
            map_bbox_to_source(data->detect_input_map, t_bbox);
        }
        data->bboxes = output_bboxes;

        if (config_.debug.show_image && !autoaim_gui::headless()) show_preview(*data);

        if (config_.debug.log_text) {
            LOGM_S("[detect] Info: detected %ld objects", data->bboxes.size());
            for (const auto &b : data->bboxes) {
                LOGM_S("[detect] Detect_Data: colorid: %d, tag_id: %d, "
                       "pts=(%.1f,%.1f)(%.1f,%.1f)(%.1f,%.1f)(%.1f,%.1f)",
                       b.color_id, b.tag_id, b.pts[0].x, b.pts[0].y, b.pts[1].x, b.pts[1].y, b.pts[2].x, b.pts[2].y,
                       b.pts[3].x, b.pts[3].y);
            }
        }

        auto t4 = std::chrono::steady_clock::now();
        if (config_.debug.log_text)
            LOGM_S("DetectSubModule Inference %.2lfms Show %.2lfms",
                   std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count() * 1000,
                   std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3).count() * 1000);

        return SubModuleResult::SUCCESS;
    }
} // namespace detect
