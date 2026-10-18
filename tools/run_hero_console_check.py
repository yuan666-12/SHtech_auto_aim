#!/usr/bin/env python3
"""Verify bounded console output without reducing production TimedSerial updates."""
from pathlib import Path
import datetime,json,os,re,shlex,subprocess
root=Path(__file__).resolve().parents[1]
out=root/'log'/('hero-console-check-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
out.mkdir()
cmd=['g++','-std=c++20','-O2','-DNDEBUG','-march=native','-flto','-pthread',
     '-DPROJECT_DIR="'+str(root)+'"']
cmd+=['-I'+str(root/p) for p in ['common','timedserial','mathutils','/usr/include/eigen3']]
cmd+=shlex.split(subprocess.check_output(['pkg-config','--cflags','opencv4'],text=True))
cmd+=[str(root/'tools/hero_console_check.cpp')]
cmd+=[str(root/'build_real'/p) for p in [
    'timedserial/CMakeFiles/timedserial.dir/timed_serial.cpp.o',
    'common/CMakeFiles/common.dir/log/log.cpp.o',
    'common/CMakeFiles/common.dir/timer/timer.cpp.o','mathutils/libmathutils.a']]
cmd+=shlex.split(subprocess.check_output(['pkg-config','--libs','opencv4'],text=True))
cmd+=['-o',str(out/'console-check')]
r=subprocess.run(cmd,capture_output=True,text=True,timeout=90)
(out/'compile.log').write_text(r.stdout+r.stderr)
if r.returncode: print(r.stderr[-4000:]);raise SystemExit(r.returncode)
env=os.environ.copy();env.update(AUTOAIM_XUC_DRY_RUN='1',AUTOAIM_XUC_LOG_FILE=str(out/'xuc.log'))
r=subprocess.run([str(out/'console-check')],cwd=out,env=env,capture_output=True,text=True,timeout=10)
text=r.stdout+r.stderr
(out/'test.log').write_text(text)
status=text.count('[TimedSerial] Status updated:')
tx=text.count('[TimedSerial][transmit]')
assert r.returncode==0,text
assert 3<=status<=4 and 1<=tx<=3,text
metrics=re.search(r'CONSOLE_METRICS updates=(\d+) packets=(\d+) elapsed_ms=([\d.]+)',text)
assert metrics,text
result=dict(returncode=0,status_lines=status,tx_lines=tx,updates=int(metrics[1]),
            packets=int(metrics[2]),elapsed_ms=float(metrics[3]),hardware_started=False)
(out/'result.json').write_text(json.dumps(result,indent=2))
print('PASS console output and per-packet status/TX checks',result)
print('RESULT_DIRECTORY',out)
