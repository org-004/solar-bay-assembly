import json,sys,itertools
d=json.load(open(sys.argv[1]))
st={s['number']:s for s in d['steps']}
segs=d['arms'][0]['segments']
X=[];Y=[]
for s in segs:
  if s['kind']!='move': continue
  f,t=s['from'],s['to']
  if 'module_take' in f or ('module_take' in t):
    p=s['path']; X.append(abs(p[-1][-1]-p[0][-1])); Y.append(s['t1']-s['t0'])
n=len(X); mx=sum(X)/n; my=sum(Y)/n
b=sum((x-mx)*(y-my) for x,y in zip(X,Y))/sum((x-mx)**2 for x in X); a=my-b*mx
res=[y-(a+b*x) for x,y in zip(X,Y)]; sd=(sum(r*r for r in res)/(n-2))**.5
print(f"fit over {n} legs: dur = {a:.2f} + {b:.3f} s/m * d, residual sd {sd:.2f} s")
# bay 2 order and set track positions
sets={0:-3.58,1:-2.43,2:-1.29,3:-0.14,4:1.00,5:2.14,6:3.29,7:4.43,8:5.50}
order=[0,3,6,1,4,7,2,5,8]; rail9=5.50
def legs(takes):
  prev=rail9; out=[]
  for m,y in zip(order,takes):
    out.append(abs(prev-y)+abs(y-sets[m])); prev=sets[m]
  return out
base=legs([0.0]*9)
print("base track metres per cycle:", [round(x,2) for x in base], "sum", round(sum(base),2))
def report(name,takes):
  L=legs(takes); dm=[b*(x-y) for x,y in zip(base,L)]
  pre0=dm[0]; allm=sum(dm)
  ceil=min(allm, pre0+0.14)
  print(f"{name}: track {sum(L):.2f} m (base {sum(base):.2f}); module-arm chain saving {allm:.2f} s; saving before set 0 {pre0:.2f} s; interval ceiling min(that, pre0+0.14) = {ceil:.2f} s")
# (a) one dock anywhere along the track
best=min((sum(legs([y]*9)),y) for y in [i/100 for i in range(-360,551)])
for y in (-0.24,0.35,1.00,best[1]):
  report(f"(a) one dock at y={y:+.2f}",[y]*9)
# (b) two docks, each take from the better dock
bestb=None
grid=[i/10 for i in range(-36,56)]
for y1,y2 in itertools.combinations(grid,2):
  if y2-y1<2.6: continue  # two pallets 2.42 m long side by side, plus a gap
  t=[]; prev=rail9
  for m in order:
    c=min((y1,y2),key=lambda y:abs(prev-y)+abs(y-sets[m])); t.append(c); prev=sets[m]
  s=sum(legs(t))
  if bestb is None or s<bestb[0]: bestb=(s,y1,y2,t)
report(f"(b) two docks at y={bestb[1]:+.1f},{bestb[2]:+.1f}",bestb[3])
# ideal: a dock at every set
report("ideal: a module at every set",[sets[m] for m in order])
