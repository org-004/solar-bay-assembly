#!/usr/bin/env python3
# post.py PREFIX OUTDIR TAG...  : reads OUTDIR/TAG.out (tool output) and OUTDIR/raw-TAG/*.txt
# and writes per-flag by-file lines and place lists, using the tool's own counting rule
# (place = file,line,col,diagnostic name; flag = the names its D lines assign to it).
import sys,os,re,glob,collections,json
prefix,od=sys.argv[1:3]; tags=sys.argv[3:]
DIAG=re.compile(r"^(\S[^:]*):(\d+):(\d+): (warning|error|fatal error): (.*)$")
NAMED=re.compile(r"^(.*?) \[(-[A-Za-z0-9=+_-]+)\]$")
res={}   # flag -> dict(ins=,outs=,files=)
rows=[]; byfile=[]; lists={}; dnames=[]
for tag in tags:
    inside=collections.defaultdict(dict); outside=collections.defaultdict(set)
    for p in glob.glob(f"{od}/raw-{tag}/*.txt"):
        for line in open(p,errors='replace'):
            m=DIAG.match(line.rstrip("\n"))
            if not m or m.group(4)!="warning": continue
            path,ln,col,_,msg=m.groups()
            nm=NAMED.match(msg)
            name,text=(nm.group(2).rstrip('='),nm.group(1)) if nm else ("(none)",msg)
            f=os.path.normpath(path)
            if '/execroot/_main/' in f: f=f.split('/execroot/_main/',1)[1]
            key=(f,int(ln),int(col))
            if f.startswith(("src/","tests/")): inside[name].setdefault(key,text)
            else: outside[name].add(key)
    groups=collections.defaultdict(set); W={}
    for l in open(f"{od}/{tag}.out"):
        p=l.split()
        if l.startswith("W "): W[p[1]]=(int(p[2]),int(p[3]))
        if l.startswith("D ") and p[4]!="-":
            for f in p[4].split(","): groups[f].add(p[1])
        if l.startswith("D "): dnames.append((p[1],int(p[2]),int(p[3]),p[4]))
    for f,(wi,wo) in W.items():
        ins={}; outs=set()
        for n in groups[f]:
            for k,t in inside.get(n,{}).items(): ins[(n,)+k]=t
            outs|={(n,)+k for k in outside.get(n,())}
        ok = (len(ins),len(outs))==(wi,wo)
        if not ok: sys.stderr.write(f"MISMATCH {tag} {f}: tool {wi},{wo} mine {len(ins)},{len(outs)}\n")
        c=collections.Counter(k[1] for k in ins)
        for fl,n in sorted(c.items()): byfile.append(f"{prefix} {f} {fl} {n}")
        lists[f]=sorted(((k[1],k[2],k[3],t,k[0]) for k,t in ins.items()))
        res[f]=(wi,wo,len(c))
    # names with no asked flag
json.dump({"res":res,"lists":lists,"d":dnames},open(f"{od}/post-{'-'.join(tags)}.json","w"))
open(f"{od}/byfile-{'-'.join(tags)}.txt","w").write("\n".join(byfile)+"\n")
