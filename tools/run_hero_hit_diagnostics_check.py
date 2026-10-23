#!/usr/bin/env python3
"""Compile and test synchronized snapshots/replay against existing build objects. No hardware."""
from pathlib import Path
import argparse,csv,json,os,shlex,subprocess,tempfile

def run(cmd,**kwargs):
    p=subprocess.run([str(x) for x in cmd],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,**kwargs)
    if p.returncode:raise RuntimeError(p.stdout[-5000:])
    return p.stdout

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--build',default='build_real');parser.add_argument('--output',type=Path)
    args=parser.parse_args();root=args.root.resolve();build=root/args.build
    with tempfile.TemporaryDirectory(prefix='hero-hit-check-') as tmp:
        d=Path(tmp);result={}
        for name in ['hero_hit_diagnostics_check','hero_hit_replay']:
            run(['c++','-O2','-std=c++20','-I'+str(root/'common'),root/'tools'/(name+'.cpp'),'-o',d/name])
        result['snapshot_checks']=run([d/'hero_hit_diagnostics_check',d/'synthetic.csv'])
        run([d/'hero_hit_replay',d/'synthetic.csv',d/'synthetic.json'])
        result['synthetic_replay']=json.loads((d/'synthetic.json').read_text())
        flags={}
        for line in (build/'CMakeFiles/auto-aim.dir/flags.make').read_text().splitlines():
            if ' = ' in line:
                key,value=line.split(' = ',1);flags[key]=shlex.split(value)
        prefix=['c++']+sum([flags[k] for k in ['CXX_DEFINES','CXX_INCLUDES','CXX_FLAGS']],[])+['-I'+str(root/'timedserial/UartIMU')]
        run(prefix+['-c',root/'tools/hero_hit_tx_check.cpp','-o',d/'tx.o'])
        link=shlex.split((build/'CMakeFiles/auto-aim.dir/link.txt').read_text())
        link[link.index('CMakeFiles/auto-aim.dir/main.cpp.o')]=str(d/'tx.o');link[link.index('-o')+1]=str(d/'tx')
        run(link,cwd=build)
        env=os.environ.copy();env.update(AUTOAIM_HEADLESS='1',AUTOAIM_XUC_DRY_RUN='1',AUTOAIM_TRACK_ONLY='1',
            AUTOAIM_XUC_LOG_FILE=str(d/'xuc.log'),LD_LIBRARY_PATH='/opt/MVS/lib/64:/usr/local/lib')
        result['tx_integration']=run([d/'tx',d/'tx.csv'],cwd=d,env=env)
        run([d/'hero_hit_replay',d/'tx.csv',d/'tx.json']);result['tx_replay']=json.loads((d/'tx.json').read_text())
        with (d/'tx.csv').open() as file:rows=list(csv.DictReader(line for line in file if not line.startswith('#')))
        assert [r['transport_status'] for r in rows]==['dry_run','invalid_command','link_disabled']
        assert all(r['written']=='0' and r['fire_byte']=='0' for r in rows)
        assert rows[0]['planned_yaw']=='4.5' and float(rows[0]['wire_yaw_mcu'])!=4.5
        lines=(d/'synthetic.csv').read_text().splitlines();columns=None
        for i,line in enumerate(lines):
            if line.startswith('sequence,'):columns=line.split(',');continue
            if line and not line.startswith('#'):
                values=line.split(',');values[columns.index('advice')]='9';lines[i]=','.join(values);break
        (d/'corrupted.csv').write_text('\n'.join(lines)+'\n')
        p=subprocess.run([str(d/'hero_hit_replay'),str(d/'corrupted.csv'),str(d/'corrupted.json')],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        assert p.returncode==2 and json.loads((d/'corrupted.json').read_text())['decision_mismatches']==1
        result['corruption_detection']=True
        if args.output:args.output.write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2))
if __name__=='__main__':main()
