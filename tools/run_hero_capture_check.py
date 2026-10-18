#!/usr/bin/env python3
"""Exercise the production handoff with fake capture and SDK, without hardware."""
from pathlib import Path
import datetime
import json
import os
import shlex
import subprocess

root = Path(__file__).resolve().parents[1]
out = root / 'log' / ('hero-capture-check-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
out.mkdir()
cmd = ['g++', '-std=c++20', '-O2', '-DNDEBUG', '-march=native', '-flto', '-pthread',
       '-DPROJECT_DIR="' + str(root) + '"']
cmd += ['-I' + str(root / p) for p in ['common', 'sensor/hikcam', '/usr/include/eigen3']]
cmd += shlex.split(subprocess.check_output(['pkg-config', '--cflags', 'opencv4'], text=True))
cmd += [str(root / 'tools/hero_capture_check.cpp')]
cmd += [str(root / 'build_real' / p) for p in
        ['common/CMakeFiles/common.dir/log/log.cpp.o', 'common/CMakeFiles/common.dir/timer/timer.cpp.o']]
cmd += shlex.split(subprocess.check_output(['pkg-config', '--libs', 'opencv4'], text=True))
cmd += ['-o', str(out / 'hero_capture_check')]
with (out / 'compile.log').open('w') as log:
    result = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT)
if result.returncode:
    print((out / 'compile.log').read_text()[-5000:])
    raise SystemExit(result.returncode)
env = os.environ.copy()
env['AUTOAIM_HEADLESS'] = '1'
result = subprocess.run([str(out / 'hero_capture_check')], cwd=out, env=env,
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
(out / 'test.log').write_text(result.stdout)
(out / 'result.json').write_text(json.dumps(dict(returncode=result.returncode,
    hardware_started=False, production_pipeline_exercised=True, camera_sdk_stubbed=True), indent=2))
print(result.stdout)
print('RESULT_DIRECTORY', out)
raise SystemExit(result.returncode)
