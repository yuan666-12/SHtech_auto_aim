#include "startup_defaults.hpp"
#include "target_model.hpp"
#include "gui_control.hpp"
#include "diagnostic_session.hpp"
#include "center_fire_control.hpp"
#include "Planner.hpp"
#include <cmath>
#include <iostream>

void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
int main(int argc, char** argv) {
    try {
        require(argc >= 2, "repository root is required");
        const bool override = argc == 3;
        for (const auto& setting : startup_defaults::settings) unsetenv(setting.name);
        if (override) {
            setenv("AUTOAIM_TARGET_MODEL", "free", 1);
            setenv("AUTOAIM_HEADLESS", "0", 1);
            setenv("AUTOAIM_TRACK_ONLY", "1", 1);
            setenv("AUTOAIM_DIAG", "0", 1);
            setenv("AUTOAIM_CENTER_WIDTH_M", "0.14", 1);
            setenv("AUTOAIM_CENTER_HEIGHT_M", "0.06", 1);
            setenv("AUTOAIM_CENTER_MARGIN_X_M", "0.012", 1);
            setenv("AUTOAIM_CENTER_MARGIN_Z_M", "0.004", 1);
            setenv("AUTOAIM_SHOOT_INTERVAL_S", "0.8", 1);
            setenv("AUTOAIM_XUC_DRY_RUN", "1", 1);
        }
        startup_defaults::apply();
        require(target_model::config().fixed() == !override, "target model receives startup setting");
        require(autoaim_gui::headless() == !override, "GUI receives startup setting");
        require(diagnostic_session::Config::from_environment().enabled == !override,
                "diagnostics receive startup setting");
        const auto window = center_fire::Config::from_environment();
        require(window.valid, "window settings are valid");
        const auto near = [](double a, double b) { return std::abs(a-b) < 1e-12; };
        require(near(window.width_m, override ? .14 : .135), "window width");
        require(near(window.height_m, override ? .06 : .056), "window height");
        require(near(window.margin_x_m, override ? .012 : .010), "horizontal margin");
        require(near(window.margin_z_m, override ? .004 : .005), "vertical margin");
        require(std::string(std::getenv("AUTOAIM_TRACK_ONLY")) == (override ? "1" : "0"),
                "tracking-only override is preserved");
        require(std::string(std::getenv("AUTOAIM_XUC_DRY_RUN")) == (override ? "1" : "0"),
                "dry-run override is preserved");
        Log console; screen = &console;
        mathutils::CoordTransformer::Init(std::string(argv[1]) + "/asset/camParam/cam_param_hero.yml", false);
        predict::Planner planner(std::string(argv[1]) + "/tools/fixtures/planner_raw_rx.yml", false);
        require(near(planner.shoot_interval, override ? .8 : 1.0), "planner receives shot interval");
        std::cout << "PASS startup " << (override ? "overrides" : "defaults")
                  << "; no camera/UART opened\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n'; return 1;
    }
}
