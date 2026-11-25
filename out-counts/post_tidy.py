#!/usr/bin/env python3
# post_tidy.py DIR TAG : from DIR/raw-TAG/*.txt makes DIR/TAG.places.json {check: [[file,line,col,msg],...]}
# using the same rule as tidy.py (place = file,line,col,check under src/ or tests/; an alias counts under each name).
import sys,os,re,glob,json,importlib.util
d,tag=sys.argv[1:3]
spec=importlib.util.spec_from_file_location('tidy','/root/work/013-design-principles-and-r2/repo/tools/architecture/tidy.py'); t=importlib.util.module_from_spec(spec); spec.loader.exec_module(t)
places={}
for p in sorted(glob.glob(f'{d}/raw-{tag}/*.txt')):
    for line in open(p,errors='replace'):
        m=t.DIAG.match(line.rstrip('\n'))
        if not m: continue
        path,ln,col,sev,msg,checks=m.groups()
        names=[c for c in (checks or '').split(',') if c!='-warnings-as-errors']
        f=t.repo_path(path,t.ROOT)
        if f is None or not names: continue
        if sev=='error' and 'clang-diagnostic-error' in names: continue
        for c in names: places.setdefault((f,int(ln),int(col),c),msg)
out={}
for (f,ln,col,c),msg in sorted(places.items()): out.setdefault(c,[]).append([f,ln,col,msg])
json.dump(out,open(f'{d}/{tag}.places.json','w'))
print({c:len(v) for c,v in sorted(out.items())}, 'total',len(places))
