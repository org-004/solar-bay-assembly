import json, sys, ast
# gen.py cell.json feas out_name key=value ... : the all-+Y-facing KR 20 layout on the shifted cell.
#  split=3 (A 0-2, B 3-5, C 6-8; B pin tool only) or split=2 (A 0-3, C 4-8)
cell, feas, name = sys.argv[1], sys.argv[2], sys.argv[3]
p = dict(a.split('=', 1) for a in sys.argv[4:])
S = 1.0
d = json.load(open(cell)); d['name'] = name
yh = lambda i: -5.148 + S + 1.144 * (i // 4) + (0.024 if i % 4 < 2 else 1.144 - 0.024)
F = {}
for line in open(feas):
    i, mh, rest = line.split(' ', 2); i = int(i)
    F[i] = [t for t in ast.literal_eval(rest) if t < yh(i)]  # base at -Y of the hole only
split = int(p.get('split', 3))
if split == 3: mods = {'a': range(0, 3), 'b': range(3, 6), 'c': range(6, 9)}; bolts = {'a': '0-4', 'b': None, 'c': '5-9'}
elif split == 2: mods = {'a': range(0, 4), 'c': range(4, 9)}; bolts = {'a': '0-4', 'c': '5-9'}
else: mods = {'a': range(0, 9)}; bolts = {'a': '0-9'}
pick = p.get('pick', 'mid')
def choose(k, i):
    v = F[i]
    if not v: raise SystemExit('hole %d has no -Y pose' % i)
    first, last = min(mods[k]), max(mods[k])
    m = i // 4
    if m == last and k != 'c': return min(v)       # top module: base as low as it goes
    if m == first and k != 'a': return max(v)      # bottom module: base as high as it goes
    return v[len(v) // 2]
for k, s in (('s', 'x'),): pass
ov = {int(a[1:]): float(b) for a, b in p.items() if a[0] == 's' and a[1:].isdigit()}
bstop = [float(v) for v in p.get('bstops', '-5.0,-4.0,-2.9,-1.8,-1.2,1.1,1.8,2.8,3.8,4.8').split(',')]
arms = {a['name'][-1]: a for a in d['arms'] if a['name'].startswith('pin_arm')}
for k in [k for k in list(arms) if k not in mods]:
    d['arms'] = [a for a in d['arms'] if a['name'] != 'pin_arm_' + k]; del arms[k]
nest = {'a': -5.2, 'b': float(p.get('nestb', 0.0)), 'c': float(p.get('nestc', 3.5))}
for k, a in arms.items():
    st = [0.0] * 36; used = []
    for m in mods[k]:
        for h in range(4):
            i = 4 * m + h; t = ov.get(i, choose(k, i)); st[i] = t; used.append(t)
    a['stops']['locate'] = st; a['stops']['pin'] = list(st); a['stops']['swage'] = list(st)
    a['stops']['pins_take'] = [nest[k]]
    if bolts[k]:
        a['stops']['strap_bolt'] = bstop
        b0, b1 = map(int, bolts[k].split('-')); used += bstop[b0:b1 + 1]
    else: a['stops'].pop('strap_bolt', None)
    used.append(nest[k])
    wt = float(p.get('wt' + k, {'a': min(used), 'b': -0.3, 'c': max(used)}[k]))
    st_ = float(p.get('st' + k, {'a': -4.7, 'b': wt, 'c': 4.6}[k]))
    lo = min(used + [wt, st_]); hi = max(used + [wt, st_])
    tr = a['track']; tr['lo'] = round(lo, 3); tr['hi'] = round(hi, 3)
    blo, bhi = lo - 0.3, hi + 0.3
    tr['beam']['size'][1] = round(bhi - blo, 3); tr['beam']['pos'][1] = round((blo + bhi) / 2, 3)
    tr['mass_kg'] = round(21.94 * (bhi - blo), 1)
    a['wait_track'] = wt; a['stow_track'] = st_
    if 'wait' + k in p: a['wait_joints_deg'] = [float(v) for v in p['wait' + k].split(',')]
    if 'stow' + k in p: a['stow_joints_deg'] = [float(v) for v in p['stow' + k].split(',')]
    if 'ts' + k in p:
        for t in a['tools']: t['waits_at'][1] = float(p['ts' + k])
pts = {k: '%d-%d' % (4 * min(v), 4 * max(v) + 3) for k, v in mods.items()}
work = []
for w in d['work']:
    if w['arm'].startswith('pin_arm'):
        k = w['arm'][-1]
        if k not in arms: continue
        if w['kind'] in ('locate', 'pin', 'swage'): w['points'] = pts[k]
        if w['kind'] == 'strap_bolt': w['points'] = bolts[k]
        if w.get('points') is None: continue
    work.append(w)
d['work'] = work
if split == 2:
    # Magazine 1 is not used: pins_take points are the stock indices; keep C's on pins_2.
    for w in d['work']:
        if w['arm'] == 'pin_arm_c' and w['kind'] == 'pins_take': w['points'] = '2'
if 'order' in p: d['order']['modules'] = [int(v) for v in p['order'].split(',')]
json.dump(d, open('/root/scratch/' + name + '.json', 'w'), indent=1)
print(name, {k: (a['track']['lo'], a['track']['hi'], a['wait_track']) for k, a in arms.items()})
