#!/usr/bin/env python3
# runt.py TAG CHECKSFILE OUTDIR [command prefix...] : tidy.sh under the lock; records wall time (inside the lock)
# and the peak of the cgroup's memory.peak (reset through our own open file, so it is the peak during this run).
import subprocess,sys,time,os,threading
tag,cf,od=sys.argv[1:4]; prefix=sys.argv[4:]
od=os.path.abspath(od); os.makedirs(od,exist_ok=True)
checks=open(cf).read().strip()
repo='/root/work/013-design-principles-and-r2/repo'
raw=f'{od}/raw-{tag}'
inner=['tools/architecture/tidy.sh','--checks',checks,'--jobs','3','--raw',raw]
def run(extra,name):
    with open(f'{od}/{tag}.{name}','w') as o, open(f'{od}/{tag}.{name}.err','w') as e:
        return subprocess.run(prefix+inner+extra,cwd=repo,stdout=o,stderr=e).returncode
# one run only; --list and --by-file are produced from the raw output by post_tidy.py
lock=open('/root/work/013-design-principles-and-r2/heavy.lock','a')
import fcntl
t_wait=time.monotonic(); fcntl.flock(lock,fcntl.LOCK_EX); waited=time.monotonic()-t_wait
peak0=int(open('/sys/fs/cgroup/memory.peak').read())
cur=[0,10**18]; stop=[False]
def samp():
    while not stop[0]:
        v=int(open('/sys/fs/cgroup/memory.current').read()); cur[0]=max(cur[0],v); cur[1]=min(cur[1],v); time.sleep(1)
th=threading.Thread(target=samp); th.start()
t=time.monotonic(); rc=run([], 'out'); wall=time.monotonic()-t
stop[0]=True; th.join(); peak=int(open('/sys/fs/cgroup/memory.peak').read())
fcntl.flock(lock,fcntl.LOCK_UN)
ps=subprocess.run(['ps','-eo','pid,etime,rss,args','--sort=-rss'],stdout=subprocess.PIPE,text=True).stdout.splitlines()[:8]
open(f'{od}/{tag}.time','w').write('rc=%d wall_s_inside_lock=%.1f waited_for_lock_s=%.1f memory.peak_after_MiB=%.0f(before=%.0f) max_current_MiB=%.0f min_current_MiB=%.0f\nprocesses by RSS at the end:\n%s\n'%(rc,wall,waited,peak/2**20,peak0/2**20,cur[0]/2**20,cur[1]/2**20,'\n'.join(x[:160] for x in ps)))
