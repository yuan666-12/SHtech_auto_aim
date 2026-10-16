#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
binary="$root/build_real/auto-aim"
config="$root/launch_calibration.cfg"
[[ -x "$binary" && -f "$config" ]] || { printf '%s\n' "Missing binary or calibration config" >&2; exit 1; }
if [[ "${1:-}" == "--check" ]]; then
    printf 'Binary: %s\nConfig: %s\nDRY_RUN=1; manual P; 9x7 board; yaw pitch roll in degrees\n' "$binary" "$config"
    exit 0
fi
[[ $# -le 1 ]] || { printf '%s\n' "Usage: bash tools/start_hero_calibration.sh [NEW_OUTPUT_DIRECTORY|--check]" >&2; exit 2; }
[[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || { printf '%s\n' "Run in the robot desktop terminal (GUI required)" >&2; exit 1; }
if pgrep -x auto-aim >/dev/null; then
    printf '%s\n' "An auto-aim process is already running; stop it before collecting" >&2
    exit 1
fi
output="${1:-$root/hero_extrinsic_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$}"
mkdir -- "$output" || { printf '%s\n' "Use a new directory; existing directories are refused" >&2; exit 1; }
output="$(cd -- "$output" && pwd)"
cd -- "$root"
export AUTOAIM_XUC_DRY_RUN=1
export AUTOAIM_TRACK_ONLY=1
export AUTOAIM_CONFIG="$config"
export AUTOAIM_CALIBRATION_DIR="$output"
printf 'Output: %s\nWait until stationary, then focus sensor_submodule and press P.\nCtrl+C in this terminal exits.\n' "$output"
exec "$binary"
