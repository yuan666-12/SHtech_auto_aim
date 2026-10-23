#!/usr/bin/env python3
"""No-hardware checks for bounded optional numeric/image recording."""
import argparse, csv, json, os, pathlib, shlex, subprocess, tempfile
p=argparse.ArgumentParser()
p.add_argument('--root',type=pathlib.Path,default=pathlib.Path(__file__).resolve().parents[1])
p.add_argument('--output',type=pathlib.Path)
a=p.parse_args();root=a.root.resolve()
def run(args,env=None):
    r=subprocess.run(args,cwd=root,env=env,capture_output=True,text=True,timeout=60)
    if r.returncode: raise RuntimeError(r.stdout+r.stderr)
    return r.stdout.strip()
with tempfile.TemporaryDirectory(prefix='hero-recording-check-') as td:
    tmp=pathlib.Path(td);binary=tmp/'check'
    flags=shlex.split(run(['pkg-config','--cflags','--libs','opencv4']))
    run(['c++','-O2','-std=c++20','-Icommon','-I/usr/include/eigen3',
         'tools/hero_diagnostic_recording_check.cpp','-pthread','-o',str(binary)]+flags)
    base={k:v for k,v in os.environ.items() if not k.startswith('AUTOAIM_')}
    report={}
    def case(mode,**values):
        project=tmp/mode;project.mkdir()
        env=base|values
        report[mode]=run([str(binary),mode,str(project)],env)
        return project
    case('geometry')
    case('disabled')
    case('invalid',AUTOAIM_DIAG='1',AUTOAIM_DIAG_CAPTURE='1',AUTOAIM_DIAG_CAPTURE_FPS='nan')
    case('duration',AUTOAIM_DIAG='1',AUTOAIM_DIAG_SECONDS='1')
    cap=case('capacity',AUTOAIM_DIAG='1',AUTOAIM_DIAG_MAX_MB='1')
    size=sum(f.stat().st_size for f in cap.rglob('*') if f.is_file())
    assert 0<size<=1024*1024,('capacity',size)
    report['capacity_bytes']=size
    writer=case('writer',AUTOAIM_DIAG='1',AUTOAIM_DIAG_MAX_MB='128')
    raw=next(writer.rglob('tx.csv'));detail=next(writer.rglob('tx.csv.geometry.csv'))
    def rows(path):
        with path.open() as f:return list(csv.DictReader(x for x in f if not x.startswith('#')))
    r,g=rows(raw),rows(detail)
    assert len(r)==len(g)==6000
    assert all(x['sequence']==y['sequence'] for x,y in zip(r,g))
    assert all(int(x['gate_end_ns'])>=int(x['gate_begin_ns'])>0 for x in g)
    frames=case('capture',AUTOAIM_DIAG='1',AUTOAIM_DIAG_CAPTURE='1',
                AUTOAIM_DIAG_CAPTURE_FPS='30',AUTOAIM_DIAG_CAPTURE_WIDTH='320',AUTOAIM_DIAG_MAX_MB='1')
    manifest=next(frames.rglob('frames.csv'));items=rows(manifest)
    assert 2<=len(items)<=10,len(items)
    assert all(int(x['saved_width'])==320 and int(x['saved_height'])==240 for x in items)
    assert all(int(x['image_ns'])>0 and int(x['enqueue_steady_ns'])>0 for x in items)
    assert all((manifest.parent/x['file']).is_file() for x in items)
    assert sum(f.stat().st_size for f in frames.rglob('*') if f.is_file())<=1024*1024
    report['capture_frames']=len(items)
    print(json.dumps(report,indent=2))
    if a.output:a.output.write_text(json.dumps(report,indent=2)+'\n')
