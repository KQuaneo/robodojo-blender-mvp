"""Sequential monitored batch: one control plus 20 seeds, no retries."""
import csv,hashlib,json,os,subprocess,time,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1];OUT=HERE/'results'
manifest_path=OUT/(sys.argv[1] if len(sys.argv)>1 else 'manifest.json')
manifest=json.loads(manifest_path.read_text())
for name,sha in manifest['harness'].items():assert hashlib.sha256((HERE/name).read_bytes()).hexdigest()==sha,f'Harness changed: {name}'
logs=OUT/'process_logs';logs.mkdir(exist_ok=True);records=[]
for trial in manifest['trials']:
    ident=trial['id'];dest=OUT/ident
    assert not dest.exists(),f'Refusing automatic retry/overwrite: {dest}'
    command=['blender','--background',str(ROOT/'output_smooth/sweep_blocks_smooth.blend'),'--python-exit-code','1','--python',str(HERE/'run_trial.py'),'--','--manifest',str(manifest_path),'--trial',ident,'--output',str(dest)]
    started=time.time();last_size=0;stalls=0;timed_out=False
    with (logs/f'{ident}.log').open('x') as log:
        proc=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        print('START',ident,'pid',proc.pid,flush=True)
        while proc.poll() is None:
            try:proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                size=log.tell();stalls=stalls+1 if size==last_size else 0;last_size=size
                print('MONITOR',ident,'pid',proc.pid,'seconds',round(time.time()-started),'log_bytes',size,'stall_checks',stalls,flush=True)
                if stalls in (3,5):print('ADVISORY OUTPUT_STALL',ident,'continuing without retry',flush=True)
                if time.time()-started>=600:
                    print('HARD_TIMEOUT: terminating',ident,flush=True);timed_out=True;proc.terminate()
                    try:proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:proc.kill();proc.wait()
    record={'id':ident,'command':command,'pid':proc.pid,'exit_code':proc.returncode,'duration_s':time.time()-started,'status':'timeout' if timed_out else ('completed' if proc.returncode==0 else 'crashed')}
    records.append(record);(OUT/'processes.json').write_text(json.dumps(records,indent=2))
    if record['status']!='completed':raise RuntimeError(f'{ident} {record["status"]}; batch halted, no automatic retry')
    result=json.loads((dest/'result.json').read_text());print('DONE',ident,result['termination_reason'],flush=True)
    if ident=='control' and not result['qualified']:raise RuntimeError('Control not qualified; random trials not started')
rows=[json.loads((OUT/t['id']/'result.json').read_text()) for t in manifest['trials']]
columns=['id','seed','initialization_valid','qualified','termination_reason','executed_through_frame','task_success','first_success_frame','collision_samples','collision_events','full_episode_audited','duration_s']
with (OUT/'summary.csv').open('w') as f:
    writer=csv.DictWriter(f,fieldnames=columns,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
summary={'control_qualified':rows[0]['qualified'],'random_attempted':20,'random_valid':sum(r['initialization_valid'] for r in rows[1:]),'random_qualified':sum(r['qualified'] for r in rows[1:]),'failures':{reason:sum(r['termination_reason']==reason for r in rows[1:]) for reason in sorted(set(r['termination_reason'] for r in rows[1:]))},'total_process_seconds':sum(p['duration_s'] for p in records),'baseline_files_unchanged':all(hashlib.sha256((ROOT/n).read_bytes()).hexdigest()==s for n,s in manifest['artifacts'].items())}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2));print('SUMMARY',json.dumps(summary),flush=True)
