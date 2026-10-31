#!/usr/bin/env bash
# Versioned desktop entry point. --check only prints effective configuration.
set -u
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project="$(cd -- "$script_dir/.." && pwd)"
launcher_dir="${HERO_LAUNCHER_DIR:-$HOME/.local/share/hero-launchers}"
mode="${1:-autoaim}"
fail() {
  printf '\n%s\n' "$1" >&2
  if [[ -t 0 ]]; then read -r -p '按回车关闭此窗口……' _; fi
  exit 1
}
[[ "$mode" == preview || "$mode" == autoaim ]] || fail '无效的启动模式。'
[[ -x "$project/build_real/auto-aim" ]] || fail '找不到自瞄程序 build_real/auto-aim。'
[[ -f "$project/launch.cfg" && -r "$launcher_dir/camera_preview.py" ]] || fail '缺少启动配置或相机预览脚本。'
if [[ "$mode" == autoaim ]]; then
  export AUTOAIM_TARGET_MODEL="${AUTOAIM_TARGET_MODEL-province_fixed}"
  export AUTOAIM_TARGET_RADIUS_M="${AUTOAIM_TARGET_RADIUS_M-0.265}"
  export AUTOAIM_HEADLESS="${AUTOAIM_HEADLESS-1}"
  export AUTOAIM_TRACK_ONLY="${AUTOAIM_TRACK_ONLY-0}"
  export AUTOAIM_XUC_DRY_RUN="${AUTOAIM_XUC_DRY_RUN-0}"
  export AUTOAIM_DIAG="${AUTOAIM_DIAG-1}"
  export AUTOAIM_CENTER_WIDTH_M="${AUTOAIM_CENTER_WIDTH_M-0.135}"
  export AUTOAIM_CENTER_HEIGHT_M="${AUTOAIM_CENTER_HEIGHT_M-0.056}"
  export AUTOAIM_CENTER_MARGIN_X_M="${AUTOAIM_CENTER_MARGIN_X_M-0.010}"
  export AUTOAIM_CENTER_MARGIN_Z_M="${AUTOAIM_CENTER_MARGIN_Z_M-0.005}"
  export AUTOAIM_SHOOT_INTERVAL_S="${AUTOAIM_SHOOT_INTERVAL_S-1}"
fi
print_startup_settings() {
  local key
  for key in AUTOAIM_TARGET_MODEL AUTOAIM_HEADLESS AUTOAIM_TRACK_ONLY AUTOAIM_DIAG \
      AUTOAIM_CENTER_WIDTH_M AUTOAIM_CENTER_HEIGHT_M AUTOAIM_CENTER_MARGIN_X_M \
      AUTOAIM_CENTER_MARGIN_Z_M AUTOAIM_SHOOT_INTERVAL_S AUTOAIM_XUC_DRY_RUN; do
    printf '%s=%s\n' "$key" "${!key}"
  done
}
if [[ "${2:-}" == --check ]]; then
  printf 'CHECK_OK mode=%s project=%s\n' "$mode" "$project"
  if [[ "$mode" == autoaim ]]; then
    printf 'Autoaim: build_real/auto-aim\n'
    printf 'Config: %s/launch.cfg; input=shared (hero-camera.service)\n' "$project"
    print_startup_settings
    printf 'Target model: %s radius_m=%s\n' "$AUTOAIM_TARGET_MODEL" "$AUTOAIM_TARGET_RADIUS_M"
  fi
  exit 0
fi
[[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || fail '请在小电脑图形桌面中打开此快捷方式。'
mkdir -p -- "$HOME/.cache/hero-launchers"
exec 9>"$HOME/.cache/hero-launchers/camera.lock"
flock -n 9 || fail '相机预览或自瞄链路已经由桌面快捷方式启动，请先关闭原窗口。'
if pgrep -x auto-aim >/dev/null; then fail '已有自瞄程序正在运行，请先在原终端按 Ctrl+C 退出。'; fi
if pgrep -x MVS >/dev/null; then fail '海康 MVS 正在运行，请先关闭 MVS，再打开此快捷方式。'; fi
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export LD_LIBRARY_PATH="/opt/MVS/lib/64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cd -- "$project" || fail '无法进入项目文件夹。'
if [[ "$mode" == preview ]]; then
  printf 'Hero 相机预览：仅打开实时画面。按 Q / Esc 或 Ctrl+C 退出。\n'
  /usr/bin/python3 "$launcher_dir/camera_preview.py"
else
  export AUTOAIM_CONFIG="$project/launch.cfg"
  printf 'Hero 自瞄链路：共享相机原图 → 检测 → 跟踪预测 → 云台串口。\n'
  printf '请先启动图传/hero-camera.service；当前自瞄不会独占打开相机。\n'
  printf '目标模型：%s，半径 %s m。\n' "$AUTOAIM_TARGET_MODEL" "$AUTOAIM_TARGET_RADIUS_M"
  if [[ "$AUTOAIM_TARGET_MODEL" == province_fixed ]]; then
    printf '省赛固定旋转靶：四板等半径、同高，中心速度约束为 0。\n'
  fi
  if [[ "$AUTOAIM_TRACK_ONLY" == 0 && "$AUTOAIM_XUC_DRY_RUN" == 0 ]]; then
    printf '当前允许云台控制与开火建议输出；实际输出取决于下位机许可和程序判定。\n'
  else
    printf '当前使用显式覆盖：TRACK_ONLY=%s，XUC_DRY_RUN=%s。\n' "$AUTOAIM_TRACK_ONLY" "$AUTOAIM_XUC_DRY_RUN"
  fi
  print_startup_settings
  printf '退出请在本终端按 Ctrl+C。\n\n'
  "$project/build_real/auto-aim"
fi
result=$?
if [[ "$result" != 0 && "$result" != 130 ]]; then fail "程序退出，状态码 $result。请查看上方提示。"; fi
exit 0
