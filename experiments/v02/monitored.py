"""Single process monitor: explicit logs, timeout, no automatic retries."""
import json
import subprocess
import time
from pathlib import Path

def run(command,logpath,cwd,timeout=600):
    logpath=Path(logpath);logpath.parent.mkdir(parents=True,exist_ok=True)
    started=time.time();timed_out=False;last_size=0;stalls=0
    with logpath.open('x') as log:
        proc=subprocess.Popen(command,cwd=cwd,stdout=log,stderr=subprocess.STDOUT)
        print('START',logpath.stem,proc.pid,flush=True)
        while proc.poll() is None:
            try:proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                size=logpath.stat().st_size;stalls=stalls+1 if size==last_size else 0;last_size=size
                print('MONITOR',logpath.stem,round(time.time()-started),size,flush=True)
                if stalls in (3,5):print('ADVISORY OUTPUT_STALL; continuing',logpath.stem,flush=True)
                if time.time()-started>=timeout:
                    print('HARD_TIMEOUT: terminating',logpath.stem,flush=True);timed_out=True;proc.terminate()
                    try:proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:proc.kill();proc.wait()
    record=dict(command=command,pid=proc.pid,exit_code=proc.returncode,duration_s=time.time()-started,
                status='timeout' if timed_out else ('completed' if proc.returncode==0 else 'crashed'))
    logpath.with_suffix('.process.json').write_text(json.dumps(record,indent=2))
    if record['status']!='completed':raise RuntimeError(str(record)+'; no automatic retry')
    return record

if __name__=='__main__':
    import sys
    run(sys.argv[2:],sys.argv[1],Path.cwd())
