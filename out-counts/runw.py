#!/usr/bin/env python3
# runw.py TAG SETFILE OUTDIR [ENV=...] : runs warnings.sh under the lock, timed. Extra args after: command prefix for docker.
import subprocess,sys,time,os
tag,setf,od=sys.argv[1:4]; prefix=sys.argv[4:]
flags=open(setf).read().strip()
od=os.path.abspath(od); os.makedirs(od,exist_ok=True)
repo='/root/work/013-design-principles-and-r2/repo'
cmd=prefix+['tools/architecture/warnings.sh','--flags',flags,'--jobs','3','--raw',os.path.abspath(f'{od}/raw-{tag}')]
t=time.monotonic()
with open(f'{od}/{tag}.out','w') as o, open(f'{od}/{tag}.err','w') as e:
    r=subprocess.run(['flock','/root/work/013-design-principles-and-r2/heavy.lock']+cmd,cwd=repo,stdout=o,stderr=e)
open(f'{od}/{tag}.time','w').write('rc=%d wall_s=%.1f (includes waiting for lock)\n'%(r.returncode,time.monotonic()-t))
