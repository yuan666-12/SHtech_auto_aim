#!/usr/bin/env python3
"""Verify normal delegation and real OpenCV headless calls without hardware."""
from pathlib import Path
import datetime
import json
import os
import shlex
import subprocess

root = Path(__file__).resolve().parents[1]
out = root / 'log' / ('hero-headless-check-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
out.mkdir()
flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'opencv4'], text=True))
base = ['g++', '-std=c++20', '-O2', '-I' + str(root / 'common'), str(root / 'tools/hero_headless_check.cpp')]
cases = []
for real in (False, True):
    binary = out / ('real_opencv' if real else 'recording_backend')
    cmd = base + (['-DHEADLESS_REAL_BACKEND'] if real else []) + flags + ['-o', str(binary)]
    built = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (out / ('compile-real.log' if real else 'compile-recording.log')).write_text(built.stdout)
    if built.returncode:
        print(built.stdout)
        raise SystemExit(built.returncode)
    for value in (['1'] if real else [None, '0', '1']):
        env = os.environ.copy()
        for key in ('AUTOAIM_HEADLESS', 'DISPLAY', 'WAYLAND_DISPLAY', 'XAUTHORITY'):
            env.pop(key, None)
        if value is not None:
            env['AUTOAIM_HEADLESS'] = value
        result = subprocess.run([str(binary), '1' if value == '1' else '0'], env=env,
                                text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=5)
        case = dict(real_opencv=real, value=value, returncode=result.returncode, output=result.stdout)
        cases.append(case)
        print(case)
        if result.returncode:
            (out / 'result.json').write_text(json.dumps(cases, indent=2))
            raise SystemExit(result.returncode)
(out / 'result.json').write_text(json.dumps(cases, indent=2))
print('PASS 4 headless startup cases; no GUI/camera/UART opened')
print('RESULT_DIRECTORY', out)
