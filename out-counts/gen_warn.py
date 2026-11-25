import json,collections
N='/root/work/013-design-principles-and-r2/notes/'
tags=('A1','A2','B')
P={c:{} for c in('gcc14','gcc13')}; L={c:{} for c in P}; D={c:{} for c in P}
for c in P:
    for t in tags:
        d=json.load(open(f'{c}/post-{t}.json'))
        for f,v in d['res'].items(): P[c][f]=v
        for f,v in d['lists'].items(): L[c][f]=v
        for n,i,o,fl in d['d']: D[c][n]=(i,o,fl)
# classification from the GCC manual (not measured)
wall_c="Wall"
default="on by default"
wextra="Wextra only"
cls={}
def s(v,*fl):
    for f in fl: cls[f]=v
s('Wall -Wunused-function -Wunused-variable ...','-Wunused')
s('Wall','-Wparentheses','-Wsequence-point','-Wmisleading-indentation','-Wdangling-else','-Wcomment','-Wsign-compare','-Wreorder','-Wdelete-non-virtual-dtor','-Waddress','-Wbool-compare','-Wtautological-compare','-Wsizeof-pointer-memaccess','-Wmemset-transposed-args','-Wswitch','-Wunused-value','-Wunused-variable','-Wunused-but-set-variable','-Wunused-function','-Wpessimizing-move','-Wrange-loop-construct','-Wclass-memaccess','-Wnonnull','-Wformat','-Wstrict-aliasing','-Warray-bounds','-Wstringop-truncation','-Wuninitialized','-Wmaybe-uninitialized','-Wint-in-bool-context','-Winfinite-recursion','-Wuse-after-free','-Wdangling-pointer','-Winit-self','-Wlogical-not-parentheses','-Wsizeof-array-div','-Wc++20-compat','-Wself-move','-Wdangling-reference','-Wcatch-value=3','-Wmaybe-uninitialized')
s('Wall (lower level)','-Wformat=2','-Wformat-overflow=2','-Wformat-truncation=2','-Warray-bounds=2','-Wdangling-pointer=2','-Wuse-after-free=3','-Wstrict-overflow=2','-Woverloaded-virtual','-Wformat-signedness')
s('Wextra','-Wunused-parameter','-Wempty-body','-Wignored-qualifiers','-Wtype-limits','-Wmissing-field-initializers','-Wimplicit-fallthrough','-Wredundant-move','-Wcast-function-type','-Wenum-conversion','-Wdeprecated-copy-dtor')
s('Wextra (level 3; asked =5)','-Wimplicit-fallthrough=5')
s('default on','-Wreturn-type','-Wnarrowing','-Wunused-result','-Wreturn-local-addr','-Wint-to-pointer-cast','-Wshift-count-overflow','-Wshift-count-negative','-Wdiv-by-zero','-Woverflow','-Wmultichar','-Wdeprecated','-Wdeprecated-declarations','-Wmismatched-new-delete','-Wfree-nonheap-object','-Wvexing-parse','-Wstringop-overflow','-Wvolatile','-Wcomma-subscript','-Wdeprecated-enum-enum-conversion','-Wdeprecated-enum-float-conversion','-Wxor-used-as-pow','-Wbidi-chars=any','-Wplacement-new=2','-Wstringop-overflow=4','-Wshift-overflow=2','-Wformat-security')
cls['-Wformat-security']='no (on with -Wformat=2)'
cls['-Wextra']='(is the flag)'
cls['-Wunsafe-loop-optimizations']='no (option is ignored by GCC 14)'
def c(f): return cls.get(f,'no')
flags=open('flagsA.txt').read().split()+open('flagsB.txt').read().split()
extra=['-Wcomment','-Wdangling-else']
def row(f):
    a=P['gcc14'].get(f,(0,0,0)); b=P['gcc13'].get(f,(0,0,0))
    return a,b
out=[]
out.append("| flag | in -Wall/-Wextra already? (GCC manual, not measured) | GCC 14.2 inside | GCC 14.2 outside | GCC 13.3 inside | GCC 13.3 outside | files (GCC 14.2) | unknown to which compiler |")
out.append("|---|---|---|---|---|---|---|---|")
for f in flags:
    a,b=row(f)
    flag=' <-- differs' if (a[0],a[1])!=(b[0],b[1]) else ''
    out.append(f"| {f} | {c(f)} | {a[0]} | {a[1]} | {b[0]} | {b[1]} | {a[2]} | none |{flag}")
tab='\n'.join(out)
# D table
names=sorted(set(D['gcc14'])|set(D['gcc13']))
dt=["| diagnostic name printed by GCC | asked flags that stand for it (D line) | GCC 14.2 inside | outside | GCC 13.3 inside | outside |","|---|---|---|---|---|---|"]
for n in names:
    a=D['gcc14'].get(n,(0,0,'-')); b=D['gcc13'].get(n,(0,0,'-'))
    dt.append(f"| {n} | {a[2]} | {a[0]} | {a[1]} | {b[0]} | {b[1]} |"+(' <-- differs' if a[:2]!=b[:2] else ''))
# lists
ls=[]; diff=[]
for f in flags:
    k14=L['gcc14'].get(f,[]); k13=L['gcc13'].get(f,[])
    s14={(x[0],x[1],x[2],x[4]) for x in k14}; s13={(x[0],x[1],x[2],x[4]) for x in k13}
    if s14!=s13: diff.append((f,sorted(s14^s13)))
    if 1<=len(k14)<=60:
        ls.append(f"### {f} ({len(k14)} places, GCC 14.2)\n```")
        for x in k14: ls.append(f"{x[0]}:{x[1]}:{x[2]}: {x[3]} [{x[4]}]")
        ls.append("```\n")
open('warn_table.md','w').write(tab+'\n\n'+'\n'.join(dt)+'\n\n'+'\n'.join(ls)+'\nINSIDE PLACE SETS DIFFERING:\n'+repr(diff))
print(len(diff),diff[:3])
