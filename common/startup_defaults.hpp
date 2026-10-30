#pragma once
#include <cstdlib>
#include <ostream>
#include <stdexcept>
#include <string>

// 仅在程序入口补齐未设置的环境变量，显式设置始终优先。
namespace startup_defaults {
struct Setting { const char* name; const char* value; };
inline constexpr Setting settings[] = {
    {"AUTOAIM_TARGET_MODEL", "province_fixed"},
    {"AUTOAIM_HEADLESS", "1"},
    {"AUTOAIM_TRACK_ONLY", "0"},
    {"AUTOAIM_DIAG", "1"},
    {"AUTOAIM_CENTER_WIDTH_M", "0.135"},
    {"AUTOAIM_CENTER_HEIGHT_M", "0.056"},
    {"AUTOAIM_CENTER_MARGIN_X_M", "0.010"},
    // 成功测试命令中的 AIM_CENTER_MARGIN_Z_M 未被读取；实际生效值为 0.005。
    {"AUTOAIM_CENTER_MARGIN_Z_M", "0.005"},
    {"AUTOAIM_SHOOT_INTERVAL_S", "1"},
    {"AUTOAIM_XUC_DRY_RUN", "0"},
};
inline void apply() {
    for (const auto& setting : settings) {
        if (::setenv(setting.name, setting.value, 0) != 0)
            throw std::runtime_error(std::string("Cannot set startup default: ") + setting.name);
    }
}
inline void print(std::ostream& out) {
    out << "Startup settings (no camera/UART opened):\n";
    for (const auto& setting : settings)
        out << setting.name << '=' << std::getenv(setting.name) << '\n';
}
} // namespace startup_defaults
