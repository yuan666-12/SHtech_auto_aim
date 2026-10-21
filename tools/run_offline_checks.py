#!/usr/bin/env python3
"""Compile and run current-source checks without opening camera or UART devices."""
import argparse
import json
import os
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', help='CTest test-name regular expression')
    parser.add_argument('--build-dir', type=Path)
    parser.add_argument('--jobs', type=int, default=2)
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    root = Path(__file__).resolve().parents[1]
    build = (args.build_dir or root / 'build_checks').resolve()
    output = root / 'log/checks/latest'
    output.mkdir(parents=True, exist_ok=True)
    commands = [
        ['cmake', '-S', str(root / 'tools/offline'), '-B', str(build),
         '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG'],
        ['cmake', '--build', str(build), '--parallel', str(args.jobs)],
        ['ctest', '--test-dir', str(build), '--output-on-failure'],
    ]
    if args.case:
        commands[-1] += ['-R', args.case, '--no-tests=error']
    env = os.environ.copy()
    for key in list(env):
        if key.startswith('AUTOAIM_'):
            env.pop(key)
    env.update(AUTOAIM_HEADLESS='1', AUTOAIM_XUC_DRY_RUN='1')
    for name, command in zip(('configure', 'build', 'tests'), commands):
        result = subprocess.run(command, cwd=root, env=env, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        (output / (name + '.log')).write_text(result.stdout, encoding='utf-8')
        print(result.stdout, end='')
        if result.returncode:
            (output / 'result.json').write_text(json.dumps({
                'returncode': result.returncode, 'failed_stage': name,
                'hardware_started': False}, indent=2), encoding='utf-8')
            return result.returncode
    (output / 'result.json').write_text(json.dumps({
        'returncode': 0, 'hardware_started': False, 'current_source_compiled': True,
        'case': args.case}, indent=2), encoding='utf-8')
    print('LATEST_RESULTS', output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
