#!/usr/bin/env python3
"""Small numeric regression check; uses deployed objects, no camera or UART."""
from pathlib import Path
import subprocess,os,datetime,shlex,json
root=Path(__file__).resolve().parents[1]
out=root/'log'/('hero-algorithm-check-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
out.mkdir()
objects=[
'planner/CMakeFiles/planner.dir/Planner.cpp.o',
'common/CMakeFiles/common.dir/log/log.cpp.o',
'common/CMakeFiles/common.dir/timer/timer.cpp.o',
'mathutils/libmathutils.a','planner/tinympc/libtinympcstatic.a']
cmd=['g++','-std=c++20','-O2','-DNDEBUG','-march=native','-flto','-pthread','-DPROJECT_DIR="'+str(root)+'"']
cmd += ['-I'+str(root/p) for p in ['common','mathutils','planner','/usr/include/eigen3']]
cmd += shlex.split(subprocess.check_output(['pkg-config','--cflags','opencv4'],text=True))
cmd += [str(root/'tools/hero_algorithm_check.cpp')]
cmd += [str(root/'build_real'/p) for p in objects]
cmd += shlex.split(subprocess.check_output(['pkg-config','--libs','opencv4'],text=True))
cmd += ['-o',str(out/'hero_algorithm_check')]
with (out/'compile.log').open('w') as f:
 result=subprocess.run(cmd,cwd=root,stdout=f,stderr=subprocess.STDOUT)
if result.returncode:
 print((out/'compile.log').read_text()[-6000:]);raise SystemExit(result.returncode)
env=os.environ.copy();env['AUTOAIM_XUC_DRY_RUN']='1'
result=subprocess.run([str(out/'hero_algorithm_check'),str(root),str(root/'asset/extrinsic_ab_20261012/planner_geometry_only.yml')],cwd=out,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=15)
(out/'test.log').write_text(result.stdout)
(out/'result.json').write_text(json.dumps({'returncode':result.returncode,'hardware_started':False,'compiled_against_deployed_objects':True},indent=2))
print(result.stdout);print('RESULT_DIRECTORY',out)
raise SystemExit(result.returncode)
