#!/usr/bin/env python3
"""Production camera lifecycle and startup-signal tests; no hardware is opened."""
from pathlib import Path
import datetime, json, os, re, shlex, signal, subprocess, time

root = Path(__file__).resolve().parents[1]
out = root/'log'/('hero-startup-check-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
out.mkdir()
flags = shlex.split(subprocess.check_output(['pkg-config','--cflags','opencv4'],text=True))
libs = shlex.split(subprocess.check_output(['pkg-config','--libs','opencv4'],text=True))
base = ['g++','-std=c++17','-O2','-I'+str(root/'sensor'),'-I'+str(root/'sensor/hikcam')]+flags
source = str(root/'tools/hero_startup_check.cpp')
def compile_run(cmd, name):
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=90)
    (out/(name+'.log')).write_text(result.stdout+result.stderr)
    if result.returncode:
        print(result.stderr[-4000:]); raise SystemExit(result.returncode)

compile_run(base+[source,str(root/'sensor/hikcam/hikcam_wrapper.cpp')]+libs+
            ['-o',str(out/'camera-check')], 'compile-camera')
r = subprocess.run([str(out/'camera-check')],capture_output=True,text=True,timeout=10)
(out/'camera-test.log').write_text(r.stdout+r.stderr)
print(r.stdout,end='')
if r.returncode: raise SystemExit(r.returncode)

# The preloaded enum function always returns failure and never calls the real SDK.
compile_run(base+['-shared','-fPIC','-DSTARTUP_SDK_PRELOAD',source,
                  '-o',str(out/'no-camera.so')], 'compile-preload')
config = (root/'launch.cfg').read_text()
config, count = re.subn(r'^port=.*$', 'port=None', config, flags=re.M)
assert count==1 and re.search(r'^source=0$',config,re.M)
(out/'mock-camera.cfg').write_text(config)
env = os.environ.copy()
env.update(AUTOAIM_CONFIG=str(out/'mock-camera.cfg'),AUTOAIM_HEADLESS='1',
           AUTOAIM_XUC_DRY_RUN='1',AUTOAIM_TRACK_ONLY='1',
           LD_PRELOAD=str(out/'no-camera.so'))
signals = []
for sig in (signal.SIGINT,signal.SIGTERM):
    path = out/(sig.name+'.log')
    with path.open('w') as log:
        proc = subprocess.Popen([str(root/'build_real/auto-aim')],cwd=root,env=env,
                                stdout=log,stderr=subprocess.STDOUT)
        try:
            deadline=time.monotonic()+8
            while 'TEST_CAMERA_RETRY_NO_HARDWARE' not in path.read_text(errors='replace'):
                if proc.poll() is not None or time.monotonic()>deadline:
                    raise AssertionError('did not reach simulated initialization retry')
                time.sleep(.02)
            before=time.monotonic()
            proc.send_signal(sig)
            rc=proc.wait(timeout=3)
            elapsed_ms=(time.monotonic()-before)*1000
            text=path.read_text(errors='replace')
            assert rc==0, text[-2500:]
            assert 'startup cancelled' in text and 'destroyed' in text, text[-2500:]
            assert 'Starting all composite tasks' not in text, 'must not start vision/TX after cancellation'
            assert elapsed_ms<1000, 'startup cancellation exceeded one second'
            signals.append(dict(signal=sig.name,returncode=rc,elapsed_ms=elapsed_ms))
        finally:
            if proc.poll() is None:
                proc.kill();proc.wait()
    print('PASS startup cancellation',signals[-1])
(out/'result.json').write_text(json.dumps(dict(returncode=0,signals=signals,
    hardware_started=False,camera_sdk_stubbed=True,production_binary_exercised=True),indent=2))
print('RESULT_DIRECTORY',out)
