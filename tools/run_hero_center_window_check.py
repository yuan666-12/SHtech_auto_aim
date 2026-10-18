#!/usr/bin/env python3
"""Real planner bridge/200 Hz serial plus numerical crossing tests; no hardware."""
from pathlib import Path
import datetime,json,os,shlex,subprocess
root=Path(__file__).resolve().parents[1]
out=root/'log'/('hero-center-window-check-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
out.mkdir()
cmd=['g++','-std=c++20','-O2','-DNDEBUG','-march=native','-flto','-pthread','-DPROJECT_DIR="'+str(root)+'"']
cmd+=['-I'+str(root/p) for p in ['common','mathutils','planner','timedserial','/usr/include/eigen3']]
cmd+=shlex.split(subprocess.check_output(['pkg-config','--cflags','opencv4'],text=True))
cmd+=[str(root/'tools/hero_center_window_check.cpp')]
cmd+=[str(root/'build_real'/p) for p in [
    'planner/CMakeFiles/planner.dir/Planner.cpp.o',
    'planner/CMakeFiles/planner.dir/planner_submodule.cpp.o',
    'timedserial/CMakeFiles/timedserial.dir/timed_serial.cpp.o',
    'common/CMakeFiles/common.dir/log/log.cpp.o','common/CMakeFiles/common.dir/timer/timer.cpp.o',
    'mathutils/libmathutils.a','planner/tinympc/libtinympcstatic.a']]
cmd+=shlex.split(subprocess.check_output(['pkg-config','--libs','opencv4'],text=True))
cmd+=['-o',str(out/'center-window-check')]
r=subprocess.run(cmd,cwd=root,capture_output=True,text=True,timeout=120)
(out/'compile.log').write_text(r.stdout+r.stderr)
if r.returncode:print(r.stderr[-6000:]);raise SystemExit(r.returncode)
env=os.environ.copy()
env.update(AUTOAIM_HEADLESS='1',AUTOAIM_XUC_DRY_RUN='1',AUTOAIM_XUC_LOG_FILE=str(out/'xuc.log'))
for key in list(env):
    if key.startswith('AUTOAIM_CENTER_') or key.startswith('AUTOAIM_FIRE_'):env.pop(key)
r=subprocess.run([str(out/'center-window-check'),str(root)],cwd=out,env=env,
    capture_output=True,text=True,timeout=15)
(out/'test.log').write_text(r.stdout+r.stderr)
(out/'result.json').write_text(json.dumps(dict(returncode=r.returncode,hardware_started=False,
    production_planner_and_serial_exercised=True),indent=2))
print(r.stdout+r.stderr);print('RESULT_DIRECTORY',out)
raise SystemExit(r.returncode)
