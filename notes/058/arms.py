import json,sys,collections
d=json.load(open(sys.argv[1]))
steps={s['number']:s for s in d['steps']}
print('events',[(e['kind'],e['t0'],e['t1']) for e in d['events']])
for a in d['arms']:
    segs=a['segments']; tot=collections.Counter()
    for s in segs: tot[s['kind']]+=s['t1']-s['t0']
    print('ARM',a['name'],'end',round(segs[-1]['t1'],2),{k:round(v,2) for k,v in tot.items()})
    if a['name'] in sys.argv[2:]:
        cur=None
        for s in segs:
            st=steps.get(s['step'],{})
            print(f"  {s['step']:4d} {s['kind']:8s} {s['t0']:8.2f} {s['t1']:8.2f} {s['t1']-s['t0']:6.2f} {st.get('what','')[:40]:40s} {s.get('from','')[:22]}->{s.get('to','')[:22]} {s.get('reason','')[:90]}")
