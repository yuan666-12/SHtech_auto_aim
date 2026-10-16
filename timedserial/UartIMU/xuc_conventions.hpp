#pragma once
namespace xuc_conventions {
// Camera: right/down/forward. IMU: left/up/forward.
// RX yaw decreases clockwise viewed from above, matching the planner sign.
// Keep yaw and yaw rate unchanged at both protocol boundaries.
constexpr float internal_yaw_from_mcu(float yaw) { return yaw; }
constexpr float mcu_yaw_from_internal(float yaw) { return yaw; }
}
