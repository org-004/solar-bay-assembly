import json,sys,collections
d=json.load(open(sys.argv[1]))
steps={s['number']:s for s in d['steps']}
for a in d['arms']:
    if a['name'] not in sys.argv[2:]: continue
    print('ARM',a['name'])
    g=collections.OrderedDict()
    for s in a['segments']:
        g.setdefault(s['step'],[]).append(s)
    for n,ss in g.items():
        mo=sum(s['t1']-s['t0'] for s in ss if s['kind'] in('move','approach','retreat'))
        wk=sum(s['t1']-s['t0'] for s in ss if s['kind']=='work')
        wt=[(round(s['t1']-s['t0'],2),s['reason'][:70]) for s in ss if s['kind']=='wait']
        print(f" {n:4d} {steps[n]['what'][:34]:34s} {ss[0]['t0']:7.2f}-{ss[-1]['t1']:7.2f} mo {mo:5.2f} wk {wk:4.1f} {wt if wt else ''}")
