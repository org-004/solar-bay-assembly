import subprocess,re,sys,json
cc=sys.argv[1:] 
def st(flags):
    r=subprocess.run(cc+['-x','c++','-std=c++20','-Q','--help=warnings']+flags,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True).stdout
    d={}
    for l in r.splitlines():
        m=re.match(r'^\s+(-W\S+)\s+(.*)$',l)
        if m: d[m.group(1)]=m.group(2).strip()
    return d
base=st([]);wall=st(['-Wall']);wex=st(['-Wall','-Wextra'])
flags=open('flagsA.txt').read().split()+open('flagsB.txt').read().split()
out={}
def look(d,f):
    k=f.split('=')[0]
    for key in (f, k, k+'='):
        if key in d: return d[key]
    for key in d:
        if key.startswith(k+'='): return d[key]
    return None
for f in flags:
    b,w,e=look(base,f),look(wall,f),look(wex,f)
    on=lambda v: v is not None and v not in('[disabled]','0','[available]','[default]') and not v.startswith('[disabled')
    out[f]=('base' if on(b) else 'Wall' if on(w) else 'Wextra' if on(e) else 'no', b,w,e)
json.dump(out,open('inwall.json','w'),indent=0)
for f,v in out.items(): print(f,v)
