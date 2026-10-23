/**
 * @file CoordTransformer.cpp
 * @brief 坐标变换模块实现 - 处理相机、IMU和世界坐标系之间的转换
 * @author Cao Jingyan
 * @date 2025/11/15
 */

#include "gui_control.hpp"
#include "CoordTransformer.hpp"
#include "armor_policy.hpp"
#include <stdexcept>
#include "hero_diagnostics.hpp"

namespace mathutils {
    // 定义静态成员变量
    std::unique_ptr<CoordTransformer> CoordTransformer::instance_ = nullptr;

    void CoordTransformer::load_armor_geometry(const cv::FileStorage &config) {
        const auto width_node = config["armor_small_width_m"];
        const auto height_node = config["armor_small_height_m"];
        if (width_node.empty() && height_node.empty()) return;
        if (width_node.empty() || height_node.empty())
            throw std::runtime_error("measured small armor requires both width and height");
        double width = 0, height = 0;
        width_node >> width;
        height_node >> height;
        if (!std::isfinite(width) || !std::isfinite(height) || width <= 0 || height <= 0 || width > 1 || height > 1)
            throw std::runtime_error("measured small armor dimensions must be finite metres in (0,1]");
        measured_small_armor_ = {{-width/2,-height/2,0}, {-width/2,height/2,0},
                                 {width/2,height/2,0}, {width/2,-height/2,0}};
    }

    const std::vector<cv::Point3d> &CoordTransformer::small_armor_model(int color_id) const {
        if (!measured_small_armor_.empty()) return measured_small_armor_;
        return color_id == 0 ? pw_red_small : pw_blue_small;
    }

    void CoordTransformer::load_gimbal_geometry() {
        cv::FileStorage geometry(PROJECT_DIR "/asset/gimbal_geometry_hero.yml", cv::FileStorage::READ);
        cv::Mat offset;
        geometry["T_pivot2imu"] >> offset;
        if (!geometry.isOpened() || offset.rows != 3 || offset.cols != 1 || !cv::checkRange(offset)) {
            throw std::runtime_error("invalid Hero pivot-to-IMU geometry");
        }
        cv::cv2eigen(offset, T_pivot2imu);
    }

    void CoordTransformer::finish_initialization(bool adjust_) {
        cv::cv2eigen(T_camera2imu_MAT, T_camera2imu);
        cv::cv2eigen(R_camera2imu_MAT, R_camera2imu);
        cv::cv2eigen(F_MAT, F);
        cv::cv2eigen(C_MAT, C);

        adjust = adjust_;

        if (adjust) {
            autoaim_gui::namedWindow("transformer trackbar", cv::WINDOW_AUTOSIZE);

            autoaim_gui::createTrackbar("pw_length", "transformer trackbar", &pw_length, 250, 0);
            autoaim_gui::createTrackbar("pw_width", "transformer trackbar", &pw_width, 250, 0);
        }
    }

    /**
     * @brief 默认构造函数 - 初始化所有变换矩阵为零矩阵
     */
    CoordTransformer::CoordTransformer(bool adjust_) {
        load_gimbal_geometry();
        // 初始化相机到IMU的平移向量 (3x1)
        T_camera2imu_MAT = cv::Mat::zeros(3, 1, CV_64FC1);

        // 初始化相机到IMU的旋转矩阵 (3x3)
        R_camera2imu_MAT = cv::Mat::zeros(3, 3, CV_64FC1);

        // 初始化相机内参矩阵 (3x3)
        F_MAT = cv::Mat::zeros(3, 3, CV_64FC1);

        // 初始化相机畸变参数 (1x5)
        C_MAT = cv::Mat::zeros(1, 5, CV_64FC1);

        // 将OpenCV矩阵转换为Eigen矩阵，便于后续数学运算
        finish_initialization(adjust_);
    }

    /**
     * @brief 带参数构造函数 - 从yml文件加载相机参数
     * @param camera_param yml文件路径
     * @details 从YAML文件中读取相机内参、畸变参数和相机-IMU外参
     */
    CoordTransformer::CoordTransformer(const std::string &camera_param, bool adjust_) {
        load_gimbal_geometry();
        // 打开相机参数文件
        cv::FileStorage fin(camera_param, cv::FileStorage::READ);
        load_armor_geometry(fin);

        // 读取相机到IMU的外参（平移向量和旋转矩阵）
        fin["T_c2i"] >> T_camera2imu_MAT; // 相机到IMU的平移向量
        fin["R_c2i"] >> R_camera2imu_MAT; // 相机到IMU的旋转矩阵

        // 读取相机内参
        fin["K"] >> F_MAT; // 相机内参矩阵 (焦距、主点等)
        fin["D"] >> C_MAT; // 相机畸变参数 (径向畸变、切向畸变)

        // 将OpenCV格式转换为Eigen格式，便于数学运算
        finish_initialization(adjust_);
    }

    /**
     * @brief 显式初始化单例
     */
    void CoordTransformer::Init(const std::string &param_file, bool adjust) {
        if (!instance_) {
            instance_ = std::unique_ptr<CoordTransformer>(new CoordTransformer(param_file, adjust));
        }
    }

    /**
     * @brief 获取单例实例
     */
    CoordTransformer &CoordTransformer::Get() {
        if (!instance_) {
            // 防御性编程：如果未初始化就调用，直接报错终止
            std::cerr << "[FATAL] CoordTransformer accessed before Init!" << std::endl;
            std::abort();
        }
        return *instance_;
    }

    /**
     * @brief 销毁单例
     */
    void CoordTransformer::Destroy() {
        instance_.reset();
    }

    /**
     * @brief PnP算法获取装甲板测量值
     * @param p 装甲板四个角点的图像像素坐标 (按顺序：左上、左下、右下、右上)
     * @param armor_number 装甲板编号 (0,1,8为大装甲板，其他为小装甲板)
     * @param color_id 颜色ID
     * @param attitude_yaw 机器人当前姿态的偏航角 (弧度)
     * @param R_world2imu 世界坐标系到IMU坐标系的旋转矩阵
     * @param yaw_in_camera 输出参数：装甲板在相机坐标系中的偏航角
     * @param measurement 输出参数：装甲板的测量值 [y, x, z, absolute_yaw]
     * @return bool 成功标志，true表示PnP求解成功，false表示失败
     * @details 通过PnP算法从2D图像点反推3D世界坐标，并计算装甲板朝向
     */
    std::vector<cv::Point3d> CoordTransformer::armor_model(int armor_number, int color_id) const {
        std::vector<cv::Point3d> pw_cur;
        if (armor_policy::ignore_armor_id()) {
            pw_cur = small_armor_model(color_id);
        } else if (adjust) {
            double pw_x = pw_length / 1000.0 / 2.0;
            double pw_y = pw_width / 1000.0 / 2.0;
            std::vector<cv::Point3d> pw_temp = {
                {-pw_x, -pw_y, 0.}, {-pw_x, pw_y, 0.}, {pw_x, pw_y, 0.}, {pw_x, -pw_y, 0.}};
            pw_cur = pw_temp;
        } else {
            if (color_id == 0) {
                if (armor_number == 0 || armor_number == 1 || armor_number == 8)
                    pw_cur = pw_red_big;
                else
                    pw_cur = small_armor_model(color_id);
            } else {
                if (armor_number == 3 && std::getenv("AUTOAIM_BLUE3_TEST_TARGET") != nullptr)
                    pw_cur = pw_blue_3_test;
                else if (armor_number == 0 || armor_number == 1 || armor_number == 8)
                    pw_cur = pw_blue_big;
                else
                    pw_cur = small_armor_model(color_id);
            }
        }
        return pw_cur;
    }

    bool CoordTransformer::pnp_get_measurement(const cv::Point2f (&p)[4], const int &armor_number, const int &color_id,
                                               const float &attitude_yaw, const Eigen::Matrix3d &R_world2imu,
                                               float &yaw_in_camera, Eigen::Vector4d &measurement) {
        const auto pw_cur = armor_model(armor_number, color_id);
        std::vector<cv::Point2f> polygon(p, p + 4);
        for (const auto &point : polygon)
            if (!std::isfinite(point.x) || !std::isfinite(point.y)) return false;
        if (!cv::isContourConvex(polygon) || std::abs(cv::contourArea(polygon)) < 16.0) return false;
        std::vector<cv::Point2d> pu(p, p + 4);
        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(pw_cur, pu, F_MAT, C_MAT, rvec, tvec, false, cv::SOLVEPNP_IPPE);
        if (!success || !cv::checkRange(rvec) || !cv::checkRange(tvec)) {
            return false;
        }
        std::vector<cv::Point2d> reprojected;
        cv::projectPoints(pw_cur, rvec, tvec, F_MAT, C_MAT, reprojected);
        double squared_error = 0.0;
        for (size_t i = 0; i < pu.size(); ++i) {
            const auto delta = pu[i] - reprojected[i];
            squared_error += delta.dot(delta);
        }
        const double rms = std::sqrt(squared_error / pu.size());
        if (!std::isfinite(rms) || rms > 5.0) {
            const auto report = hero_log::permit(hero_log::Stream::Pnp, 2);
            if (report.emit) {
                std::ostringstream text;
                text << "[PNP_REJECT] reprojection_rms_px=" << rms << " limit_px=5 seq=" << report.sequence
                     << " skipped=" << report.skipped;
                hero_log::append(text.str(), report.event);
            }
            return false;
        }
        Pos3D pc, pw;
        cv::cv2eigen(tvec, pc);
        if (!pc.allFinite() || pc.z() <= 0.0 || !R_world2imu.allFinite()) {
            return false;
        }
        pw = pc_to_pw(pc, R_world2imu);
        cv::Mat mat_R;
        Eigen::Matrix3d R;
        cv::Rodrigues(rvec, mat_R);
        cv::cv2eigen(mat_R, R);
        Pos3D p_a_c[4];
        for (int i = 0; i < 4; ++i) {
            Pos3D temp(pw_cur[i].x, pw_cur[i].y, pw_cur[i].z);
            p_a_c[i] = R * temp + pc;
        }
        Pos3D armor_v_x_c, armor_v_y_c, armor_v_z_c;
        armor_v_x_c = (p_a_c[0] - p_a_c[3]) + (p_a_c[1] - p_a_c[2]);
        armor_v_y_c = (p_a_c[1] - p_a_c[0]) + (p_a_c[2] - p_a_c[3]);
        armor_v_z_c = armor_v_x_c.cross(armor_v_y_c);
        yaw_in_camera = atan2(armor_v_z_c[0], -armor_v_z_c[2]);
        if (adjust)
            std::cout << pc[1] << std::endl << pc[0] << std::endl << pc[2] << std::endl << yaw_in_camera << std::endl;
        const Pos3D normal_world = R_world2imu.transpose() * R_camera2imu * armor_v_z_c;
        if (!normal_world.allFinite() || normal_world.head<2>().squaredNorm() <= 1e-16) return false;
        const double world_armor_yaw = std::atan2(-normal_world.x(), -normal_world.y());
        measurement << pw[1], pw[0], pw[2], world_armor_yaw;
        (void)attitude_yaw;
        return true;
    }
} // namespace mathutils
