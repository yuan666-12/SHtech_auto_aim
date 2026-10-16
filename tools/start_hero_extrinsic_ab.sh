#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cfgdir="$root/asset/extrinsic_ab_20261012"
binary="$root/build_real/auto-aim"
if [[ "${1:-}" == "--check" ]]; then
    python3 - "$cfgdir" "$binary" <<'PY'
import pathlib,sys,cv2
cfg=pathlib.Path(sys.argv[1]);binary=pathlib.Path(sys.argv[2])
assert binary.is_file()
for label in ['A','B']:
    assert (cfg/f'launch_{label}.cfg').is_file()
    fs=cv2.FileStorage(str(cfg/f'camera_{label}.yml'),cv2.FILE_STORAGE_READ)
    assert fs.isOpened() and fs.getNode('R_c2i').mat().shape==(3,3) and fs.getNode('T_c2i').mat().shape==(3,1)
    fs.release()
fs=cv2.FileStorage(str(cfg/'planner_geometry_only.yml'),cv2.FILE_STORAGE_READ)
for key,value in [('geometry_only',1),('yaw_comp',0),('pitch_comp',0),('consider_air_resistence',0)]:
    assert fs.getNode('planner').getNode(key).real()==value
fs.release()
print('A/B configs readable; geometry_only=1; fixed compensation=0; launcher forces DRY_RUN=1')
PY
    exit 0
fi
[[ $# -eq 2 ]] || { printf '%s\n' "Usage: bash tools/start_hero_extrinsic_ab.sh A|B near|far" >&2; exit 2; }
candidate="$1"
distance="$2"
[[ "$candidate" == "A" || "$candidate" == "B" ]] || exit 2
[[ "$distance" == "near" || "$distance" == "far" ]] || exit 2
[[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || { printf '%s\n' "Use the robot desktop terminal" >&2; exit 1; }
if pgrep -x auto-aim >/dev/null; then
    printf '%s\n' "An auto-aim process is already running; stop it before switching candidates" >&2
    exit 1
fi
output="$root/extrinsic_ab_${candidate}_${distance}_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$"
mkdir -- "$output"
export AUTOAIM_XUC_DRY_RUN=1
export AUTOAIM_TRACK_ONLY=1
export AUTOAIM_CONFIG="$cfgdir/launch_${candidate}.cfg"
export AUTOAIM_CALIBRATION_DIR="$output"
export AUTOAIM_VALIDATION_CAPTURE=1
export AUTOAIM_XUC_LOG_FILE="$output/xuc.log"
python3 - "$output" "$candidate" "$distance" "$AUTOAIM_CONFIG" "$binary" <<'PY'
import pathlib,sys,json,datetime,shutil,hashlib
output=pathlib.Path(sys.argv[1]);launch=pathlib.Path(sys.argv[4]);binary=pathlib.Path(sys.argv[5])
values={}
for line in launch.read_text().splitlines():
    if '=' in line:
        key,value=line.split('=',1);values[key]=value
assert values['port'].startswith('xuc:'),'A/B logger requires the XUC driver with DRY_RUN support'
shutil.copy2(launch,output/'launch_used.cfg')
shutil.copy2(values['camera_para'],output/'camera_used.yml')
shutil.copy2(values['planner_para'],output/'planner_used.yml')
metadata={'candidate':sys.argv[2],'distance_label':sys.argv[3],
 'started_at':datetime.datetime.now().astimezone().isoformat(),
 'utc_offset_seconds':datetime.datetime.now().astimezone().utcoffset().total_seconds(),
 'DRY_RUN':True,'geometry_only':True,'fixed_compensation':False,
 'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
 'capture_rule':'stop before P; hold at least 1 second after P',
 'translation_origin':'true IMU measurement reference point'}
(output/'session.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2))
PY
printf 'Candidate: %s; distance: %s\nOutput: %s\n' "$candidate" "$distance" "$output"
printf '%s\n' "Focus sensor_submodule. Stop 1-2 s, press P, hold 1 s. Ctrl+C exits."
cd -- "$root"
exec "$binary" > >(tee "$output/runtime.log") 2>&1
