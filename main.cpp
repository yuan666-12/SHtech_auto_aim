#include "runtime_threads.hpp"
// Resource order: allocate, bind callbacks, start workers, join, release.
#include <cstdlib>
#include <algorithm>
#include "gui_control.hpp"
#include "main.hpp"

int totalFrameCounter = 0;

std::atomic<bool> g_stop_request{false};

// 第一步：声明指针
// 使用通用的 PipelineTask 架构
pipeline::PipelineTask *pipeline_stage0 = nullptr;
pipeline::PipelineTask *pipeline_stage1 = nullptr;
pipeline::PipelineTask *pipeline_stage2 = nullptr;

hardware::TimedSerial *timed_serial = nullptr;
// foxgloveSer::FoxgloveServer_t* foxglove_server = nullptr;
pipeline::bridge::PlannerToSerialBridge *planner_to_serial_bridge = nullptr;

pipeline::bridge::EntryStageToFoxgloveRobotBridge *entrystage_to_foxglove_robot_bridge = nullptr;
pipeline::bridge::EntryStageToFoxgloveAliveBridge *entrystage_to_foxglove_alive_bridge = nullptr;
pipeline::bridge::SensorFromSerialAttitudeBridge *sensor_from_serial_attitude_bridge = nullptr;
pipeline::bridge::SensorFromSerialRobotStatusBridge *sensor_from_serial_robot_status_bridge = nullptr;
// 第一步结束

// 专门处理 SIGSEGV 的函数
void segv_handler(int sig) {
    // 1. 输出一句简单的提示 (使用 write 是异步安全的，不要用 LOG/printf)
    constexpr char msg[] = "\n[CRITICAL] Segmentation Fault Detected!\n";
    const auto written = write(STDERR_FILENO, msg, sizeof(msg) - 1);
    (void)written;

    // 2. 尝试打印堆栈信息
    void *array[20];
    int size = backtrace(array, 20);
    backtrace_symbols_fd(array, size, STDERR_FILENO);

    // 3. 直接暴力退出，不再尝试析构任何对象
    // 操作系统会负责回收内存和句柄
    _Exit(1);
}

// 专门处理 SIGINT 的函数
void int_handler(int sig) {
    g_stop_request.store(true);
}

// 线程退出监控函数
struct ThreadMonitor {
    std::string name;
    std::thread *t_ptr;
    std::future<void> join_task;
};

// 线程关闭等待超时时间
constexpr std::chrono::seconds THREAD_JOIN_TIMEOUT{3};

ThreadMonitor create_monitor(std::string name, std::thread &t) {
    return ThreadMonitor{name, &t, std::async(std::launch::async, [&t]() {
                             if (t.joinable()) {
                                 t.join();
                             }
                         })};
};

namespace {
    template <class Config> Config module_config(cmd_parser &parser, const std::string &prefix) {
        Config config;
        config.debug.log_text = parser.get_bool(prefix + "_log_text");
        config.debug.log_file = parser.get_bool(prefix + "_log_file");
        config.debug.show_image = parser.get_bool(prefix + "_show_image");
        return config;
    }

    void allocate_tasks_and_bridges() {
        pipeline_stage0 = new pipeline::PipelineTask();
        pipeline_stage1 = new pipeline::PipelineTask();
        pipeline_stage2 = new pipeline::PipelineTask();
        planner_to_serial_bridge = new pipeline::bridge::PlannerToSerialBridge();
        entrystage_to_foxglove_robot_bridge = new pipeline::bridge::EntryStageToFoxgloveRobotBridge();
        entrystage_to_foxglove_alive_bridge = new pipeline::bridge::EntryStageToFoxgloveAliveBridge();
        sensor_from_serial_attitude_bridge = new pipeline::bridge::SensorFromSerialAttitudeBridge();
        sensor_from_serial_robot_status_bridge = new pipeline::bridge::SensorFromSerialRobotStatusBridge();
    }

    std::unique_ptr<SerialInterface> make_serial_driver(const std::string &port) {
        if (port == "None" || port.empty()) {
            LOGW_S("[init] Using MockDriver (no hardware mode)");
            return std::make_unique<MockDriver>();
        }
        if (port.compare(0, 4, "xuc:") == 0) {
            const auto arg = port.substr(4);
            const auto sep = arg.find(':');
            const auto stm32 = arg.substr(0, sep);
            const auto imu = sep == std::string::npos ? std::string{} : arg.substr(sep + 1);
            LOGM_S("[init] Using XucDriver: stm32=%s, imu=%s", stm32.c_str(), imu.empty() ? "(none)" : imu.c_str());
            return std::make_unique<XucDriver>(stm32, imu);
        }
        LOGM_S("[init] Using UartDriver with port: %s", port.c_str());
#ifdef ENABLE_LEGACY_UART
        return std::make_unique<UartDriver>(port);
#else
        throw std::runtime_error("legacy UART driver was disabled at build time; use xuc: or None");
#endif
    }

    bool register_sensor_stage(cmd_parser &parser) {
        const auto entry_config = module_config<entrystage::EntryStageConfig>(parser, "entrystage");
        auto sensor_config = module_config<sensor::SensorConfig>(parser, "sensor");
        sensor_config.stop_requested = [] { return g_stop_request.load(); };
        sensor_config.save_image = parser.get_bool("sensor_save_image");
        sensor_config.save_interval = std::stoi(parser.get_string("sensor_save_interval"));
        sensor_config.calib_detect = parser.get_bool("sensor_calib_detect");
        const bool entry_ok = pipeline_stage0->register_submodule_with_params<entrystage::EntryStageSubModule>(
            entry_config, *entrystage_to_foxglove_robot_bridge, *entrystage_to_foxglove_alive_bridge);
        const bool sensor_ok = pipeline_stage0->register_submodule_with_params<sensor::SensorSubModule>(
            sensor_config, parser.get_string("source"), parser.get_string("flip"), *sensor_from_serial_attitude_bridge,
            *sensor_from_serial_robot_status_bridge);
        return entry_ok && sensor_ok;
    }

    bool register_vision_stages(cmd_parser &parser) {
        auto corner_config = module_config<detect::CornerRefineConfig>(parser, "corner_refine");
        corner_config.adjust_threshold = parser.get_bool("corner_refine_adjust_threshold");
        auto predictor_config = module_config<predict::MultiPolicyPredictorConfig>(parser, "predictor");
        predictor_config.adjust_mode = parser.get_bool("predictor_adjust_mode");
        predictor_config.adjust_tracker_noise = parser.get_bool("predictor_adjust_tracker_noise");
        auto planner_config = module_config<plan::PlannerConfig>(parser, "planner");
        planner_config.plot = parser.get_bool("planner_plot");
        const bool preprocess_ok = pipeline_stage0->register_submodule_with_params<detect::PreprocessSubModule>(
            module_config<detect::PreprocessConfig>(parser, "preprocess"));
        const bool detect_ok = pipeline_stage1->register_submodule_with_params<detect::DetectSubModule>(
            module_config<detect::DetectConfig>(parser, "detect"), parser.get_string("model"));
        const bool corner_ok =
            pipeline_stage2->register_submodule_with_params<detect::CornerRefineSubModule>(corner_config);
        const bool predict_ok =
            pipeline_stage2->register_submodule_with_params<predict::MultiPolicyPredictorSubModule>(predictor_config);
        const bool planner_ok = pipeline_stage2->register_submodule_with_params<plan::PlannerSubModule>(
            planner_config, *planner_to_serial_bridge, parser.get_string("planner_para"));
        return preprocess_ok && detect_ok && corner_ok && predict_ok && planner_ok;
    }

    void register_dependencies() {
        pipeline_stage0->register_skip_dependencies(SubModuleName::PREPROCESS, {SubModuleName::SENSOR});
        pipeline_stage1->register_skip_dependencies(SubModuleName::DETECT,
                                                    {SubModuleName::SENSOR, SubModuleName::PREPROCESS});
        pipeline_stage2->register_skip_dependencies(SubModuleName::CORNER_REFINE, {SubModuleName::DETECT});
        pipeline_stage2->register_skip_dependencies(
            SubModuleName::MULTI_POLICY_PREDICTOR,
            {SubModuleName::SENSOR, SubModuleName::PREPROCESS, SubModuleName::DETECT, SubModuleName::CORNER_REFINE});
        pipeline_stage2->register_skip_dependencies(SubModuleName::PLANNER, {SubModuleName::MULTI_POLICY_PREDICTOR});
    }

    void release_resources() {
        // Tasks release their callbacks before bridges are destroyed.
        delete pipeline_stage0;
        delete pipeline_stage1;
        delete pipeline_stage2;
        delete timed_serial;
        delete planner_to_serial_bridge;
        delete entrystage_to_foxglove_robot_bridge;
        delete entrystage_to_foxglove_alive_bridge;
        delete sensor_from_serial_attitude_bridge;
        delete sensor_from_serial_robot_status_bridge;
        mathutils::CoordTransformer::Destroy();
    }

    bool all_threads_finished(const std::vector<ThreadMonitor> &monitors) {
        return std::all_of(monitors.begin(), monitors.end(), [](const auto &monitor) {
            return monitor.join_task.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
        });
    }

    void wait_for_shutdown(std::vector<ThreadMonitor> &monitors) {
        const auto deadline = std::chrono::steady_clock::now() + THREAD_JOIN_TIMEOUT;
        while (!all_threads_finished(monitors) && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (all_threads_finished(monitors)) {
            LOGM_S("All threads joined successfully!");
            return;
        }
        std::string stuck;
        for (auto &monitor : monitors) {
            if (monitor.join_task.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
                stuck += "[" + monitor.name + "] ";
                LOGE_S("TIMEOUT: Thread %s is STUCK and failed to join!", monitor.name.c_str());
            }
        }
        const char *message = "\n[FATAL] System Hang Detected! Force Quitting.\n";
        const auto written = write(STDERR_FILENO, message, strlen(message));
        (void)written;
        std::cerr << "Stuck Threads: " << stuck << std::endl;
        _Exit(1); // Do not run destructors while a worker can still access its task.
    }
} // namespace

bool init() {
    signal(SIGINT, int_handler);
    signal(SIGTERM, int_handler);
    signal(SIGSEGV, segv_handler);
    screen = new Log();
    try {
        const auto thread_config = autoaim_runtime::configure_opencv_threads();
        LOGM_S("[Threads] OpenCV requested=%d effective=%d", thread_config.requested, thread_config.effective);
        const char *config = std::getenv("AUTOAIM_CONFIG");
        cmd_parser parser(config && *config ? config : "launch.cfg");
        LOGM_F("open log file success!");
        LOGM_S("open log file success!");
        LOGM_S("[GUI] headless=%d (AUTOAIM_HEADLESS)", autoaim_gui::headless());
        mathutils::CoordTransformer::Init(parser.get_string("camera_para"),
                                          parser.get_bool("transformer_adjust_armor_size"));
        allocate_tasks_and_bridges();
        timed_serial =
            new hardware::TimedSerial(module_config<hardware::TimedSerialConfig>(parser, "timed_serial"),
                                      make_serial_driver(parser.get_string("port")), *planner_to_serial_bridge,
                                      *sensor_from_serial_attitude_bridge, *sensor_from_serial_robot_status_bridge);
        if (!register_sensor_stage(parser) || g_stop_request.load()) {
            LOGW_S("[init] Camera unavailable or startup cancelled; aborting initialization");
            return false;
        }
        const bool vision_ok = register_vision_stages(parser);
        register_dependencies();
        if (!vision_ok) {
            LOGE_S("[init] Critical modules unavailable, system cannot start");
            return false;
        }
        LOGM_S("[init] all composite tasks registered successfully, system can start");
        return true;
    } catch (const std::exception &error) {
        LOGE_S("[init] Config/init failed: %s", error.what());
    } catch (...) {
        LOGE_S("[init] Config/init failed: unknown exception");
    }
    return false;
}

int main() {
    if (!init()) {
        LOGE_S("Init Fail, Quit");
        release_resources();
        return g_stop_request.load() ? 0 : 1;
    }
    constexpr int frame_pool_size = 4;
    pipeline::AutoAimHandshake cap2det(0);
    pipeline::AutoAimQueue det2pre(2), pre2cap(frame_pool_size + 1);
    for (int i = 0; i < frame_pool_size; ++i)
        pre2cap.put(std::make_shared<ThreadDataPack>());
    sigset_t previous_mask, mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &mask, &previous_mask);
    std::thread sensor_thread([&] { (*pipeline_stage0)(pre2cap, cap2det); });
    std::thread detect_thread([&] { (*pipeline_stage1)(cap2det, det2pre); });
    std::thread predict_thread([&] { (*pipeline_stage2)(det2pre, pre2cap); });
    std::thread serial_thread([&] { (*timed_serial)(); });
    pthread_sigmask(SIG_SETMASK, &previous_mask, nullptr);
    LOGM_S("Starting all composite tasks...");
    timed_serial->start();
    pipeline_stage0->start();
    pipeline_stage1->start();
    pipeline_stage2->start();
    LOGM_S("All composite tasks started successfully!");
    while (!g_stop_request.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    LOGM_S("Graceful shutdown initiated...");
    timed_serial->terminate();
    pipeline_stage0->terminate();
    pipeline_stage1->terminate();
    pipeline_stage2->terminate();
    std::vector<ThreadMonitor> monitors;
    monitors.push_back(create_monitor("TimedSerial", serial_thread));
    monitors.push_back(create_monitor("Sensor", sensor_thread));
    monitors.push_back(create_monitor("Detect", detect_thread));
    monitors.push_back(create_monitor("Predict", predict_thread));
    wait_for_shutdown(monitors);
    LOGM(screen, "Successfully Quit!");
    release_resources();
    return 0;
}
