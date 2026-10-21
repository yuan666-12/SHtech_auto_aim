// Regression cases for the behavior corrected during the local cleanup.
#include "common.hpp"
#include "CoordTransformer.hpp"
#include "MultiPolicyPredictor_submodule.hpp"
#include "preprocess_submodule.hpp"
#include "sensor_submodule.hpp"
#include <filesystem>
#include <iostream>
#include <stdexcept>
#define private public
#include "armor_corner_optimizer.hpp"
#undef private

namespace {
    int checks = 0;
    void require(bool ok, const char *message) {
        ++checks;
        if (!ok) throw std::runtime_error(message);
    }

    void image_checks() {
        detect::ArmorCornerOptimizer optimizer(false, 50);
        cv::Mat blue(5, 5, CV_8UC3, cv::Scalar(255, 0, 0));
        cv::Mat red(5, 5, CV_8UC3, cv::Scalar(0, 0, 255));
        require(cv::countNonZero(optimizer.preprocessImage(blue, {0, 0, 5, 5})) == 0,
                "BGR blue uses the blue luminance coefficient");
        require(cv::countNonZero(optimizer.preprocessImage(red, {0, 0, 5, 5})) == 25,
                "BGR red uses the red luminance coefficient");
        cv::Point2f corners[4]{{100, 80}, {100, 160}, {220, 160}, {220, 80}};
        require(!optimizer.optimizeCorners({}, corners, false), "empty image rejects refinement");
        cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
        cv::ellipse(frame, {100, 120}, {4, 40}, 0, 0, 360, {255, 255, 255}, -1);
        cv::ellipse(frame, {220, 120}, {4, 40}, 0, 0, 360, {255, 255, 255}, -1);
        detect::CornerRefineCallStats stats;
        auto refined = optimizer.optimizeCorners(frame, corners, false, &stats);
        require(refined.has_value() && stats.successful_lights == 2, "both synthetic light bars refine");
        auto data = std::make_shared<ThreadDataPack>();
        data->frame = cv::Mat(1080, 1440, CV_8UC3, cv::Scalar(1, 2, 3));
        detect::PreprocessSubModule preprocess(detect::PreprocessConfig{});
        require(preprocess.process(data, nullptr) == SubModuleResult::SUCCESS, "preprocess succeeds");
        require(data->detect_input.size() == cv::Size(640, 512) &&
                    data->detect_input.at<cv::Vec3b>(0, 0) == cv::Vec3b(3, 2, 1),
                "preprocess converts BGR to RGB");
        require(data->detect_input_map.src_roi == cv::Rect(0, 0, 1440, 1080), "preprocess retains full image extent");
    }

    bbox_t projected_armor(const std::string &root, const Eigen::Vector3d &center, int id) {
        cv::FileStorage file(root + "/asset/camParam/cam_param_hero.yml", cv::FileStorage::READ);
        cv::Mat rotation_cv, intrinsic, distortion;
        file["R_c2i"] >> rotation_cv;
        file["K"] >> intrinsic;
        file["D"] >> distortion;
        Eigen::Matrix3d camera_to_imu, world_armor;
        cv::cv2eigen(rotation_cv, camera_to_imu);
        world_armor << 1, 0, 0, 0, 0, 1, 0, -1, 0;
        const auto attitude = Attitude();
        const Eigen::Matrix3d rotation = camera_to_imu.transpose() * attitude.R_world2imu() * world_armor;
        const auto camera = mathutils::CoordTransformer::Get().pw_to_pc(center, attitude.R_world2imu());
        cv::Mat matrix, rvec, tvec;
        cv::eigen2cv(rotation, matrix);
        cv::Rodrigues(matrix, rvec);
        cv::eigen2cv(camera, tvec);
        std::vector<cv::Point2d> pixels;
        cv::projectPoints(mathutils::pw_blue_small, rvec, tvec, intrinsic, distortion, pixels);
        bbox_t box{};
        for (int i = 0; i < 4; ++i)
            box.pts[i] = pixels[i];
        box.tag_id = id;
        box.color_id = 1;
        box.confidence = .9;
        box.source = DetectionSource::NEURAL_NETWORK;
        return box;
    }

    void lock_checks(const std::string &root) {
        setenv("AUTOAIM_IGNORE_ARMOR_ID", "0", 1);
        predict::MultiPolicyPredictorSubModule predictor(predict::MultiPolicyPredictorConfig{});
        const auto first = projected_armor(root, {.4, 3, .2}, 2);
        const auto closer = projected_armor(root, {0, 2, .2}, 3);
        const auto started = std::chrono::high_resolution_clock::now();
        std::shared_ptr<ThreadDataPack> data;
        for (int i = 0; i < 17; ++i) {
            // Every frame is a fresh packet: lock state must belong to the predictor.
            data = std::make_shared<ThreadDataPack>();
            data->frame = cv::Mat::zeros(1080, 1440, CV_8UC3);
            data->attitude_valid = true;
            data->time = started + std::chrono::milliseconds(10 * i);
            data->robotstatus.enemy_color = EnemyColor::BLUE;
            data->robotstatus.program_mode = ProgramMode::AUTO_AIM;
            data->bboxes = i < 15 ? std::vector<bbox_t>{first} : std::vector<bbox_t>{first, closer};
            predictor.process(data, nullptr);
            require(data->target.tracked_armor.tag_id == 2, "new packet and closer rival do not steal locked target");
        }
        require(data->has_fixed_target, "autoaim lock survives packet replacement");
        data->attitude_valid = false;
        predictor.process(data, nullptr);
        require(!data->has_fixed_target && data->target.predictor_state == TrackingState::IDLE,
                "invalid attitude clears published target and lock");
        unsetenv("AUTOAIM_IGNORE_ARMOR_ID");
    }

    void log_checks() {
        bool rejected = false;
        try {
            Log invalid("");
        } catch (const std::invalid_argument &) {
            rejected = true;
        }
        require(rejected, "empty log directory reports an error without indexing an empty string");
        Log console;
        rejected = false;
        try {
            console.reopen("/proc/hero-refactor-does-not-exist/log.txt");
        } catch (const std::runtime_error &) {
            rejected = true;
        }
        require(rejected && console.out == stdout, "failed reopen preserves the old stream");
    }
} // namespace

int main(int argc, char **argv) {
    try {
        Log console;
        screen = &console;
        require(argc == 2, "project directory argument is required");
        setenv("AUTOAIM_HEADLESS", "1", 1);
        mathutils::CoordTransformer::Init(std::string(argv[1]) + "/asset/camParam/cam_param_hero.yml", false);
        image_checks();
        lock_checks(argv[1]);
        log_checks();
        mathutils::CoordTransformer::Destroy();
        std::cout << "PASS " << checks << " refactor checks; no camera/UART opened\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
