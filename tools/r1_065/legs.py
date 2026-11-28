import json,sys
def legs(path,bay):
  d=json.load(open(path)); st={s['number']:s for s in d['steps']}
  out=[]
  for s in d['arms'][0]['segments']:
    if s['kind']!='move' or st[s['step']]['bay']!=bay: continue
    out.append((s['from'].replace(' retreat','').replace(' approach',''),s['to'].replace(' approach',''),round(s['t1']-s['t0'],2),s.get('budget'),s.get('seed')))
  return out
for bay in (0,1,2):
  A=legs(sys.argv[1],bay); B=legs(sys.argv[2],bay)
  tot=0
  print('bay',bay)
  for a,b in zip(A,B):
    if 'module' in a[0]+a[1] or 'wait' in a[0]:
      tot+=b[2]-a[2]
      print(f"  {a[0]:>16} -> {a[1]:<18} base {a[2]:5.2f} probe {b[2]:5.2f} diff {b[2]-a[2]:+5.2f} seed_same {a[4]==b[4]}")
  print('  module legs total diff', round(tot,2))
