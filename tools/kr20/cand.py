import json, sys
# cand.py name key=value... : the 3-arm KR 20 candidate
src = '/root/work/031-kuka-kr-20-pin-arms-for-the-r1/solar-bay-assembly/layouts/flat_rack_3_pin_arms_ordered.json'
d = json.load(open(src))
name = sys.argv[1]; p = dict(a.split('=', 1) for a in sys.argv[2:])
d['name'] = name
X = float(p.get('x', -0.35)); SH = float(p.get('shift', 0.4)); GL = float(p.get('grip', 0.2)); CY = float(p.get('coly', 0.1))
# Holder: saddles shortened along Y and moved toward the stretch's end away from the rail the pin arm reaches.
for g in d['holder']['grips']: g['length'] = GL
for b in d['holder']['boxes']:
    if not b['name'].startswith('stand_'): continue
    y0 = b['pos'][1]; s = -SH if y0 < 0 else float(p.get('shiftc', -SH))
    b['pos'][1] = round(y0 + s, 4)
    if 'column' in b['name']: b['size'][1] = CY
    else: b['size'][1] = GL
# Stops per hole (locate, pin, swage use the same), from the probes of run 031.
T = {}
def put(m, h, t): T[4 * m + h] = t
A = {(0,0): -3.7, (0,1): -3.5, (0,2): -5.3, (0,3): -2.4, (1,0): -5.3, (1,1): -2.4, (1,2): -4.2, (1,3): -4.5,
     (2,0): -4.25, (2,1): -4.5, (2,2): -3.1, (2,3): -3.3, (3,0): -3.1, (3,1): -3.3, (3,2): -1.9, (3,3): -2.2}
B = {(4,0): 0.8, (4,1): 0.8, (4,2): -0.8, (4,3): -1.0, (5,0): -0.8, (5,1): -1.0, (5,2): 0.4, (5,3): 0.1}
C = {(6,0): 3.0, (6,1): 3.3, (6,2): 1.4, (6,3): 1.3,
     (7,0): 1.5, (7,1): 1.3, (7,2): 2.7, (7,3): 2.3, (8,0): 2.6, (8,1): 2.4, (8,2): 3.7, (8,3): 3.5}
for k, v in p.items():
    if k[0] == 's' and k[1:2].isdigit():  # s3_2=-1.8 overrides a stop
        m, h = map(int, k[1:].split('_'))
        for tab in (A, B, C):
            if (m, h) in tab: tab[(m, h)] = float(v)
tracks = {'a': (-5.3, max(A.values())), 'b': (min(B.values()), max(B.values())), 'c': (min(C.values()), 5.3)}
for k in 'abc':
    if 'lo' + k in p: tracks[k] = (float(p['lo' + k]), tracks[k][1])
    if 'hi' + k in p: tracks[k] = (tracks[k][0], float(p['hi' + k]))
nest = {'a': -5.2, 'b': float(p.get('nestb', 0.0)), 'c': float(p.get('nestc', 3.5))}
wait = [float(v) for v in p.get('wait', '90,45,-100,0,40,0').split(',')]
stow = [float(v) for v in p.get('stow', p.get('wait', '90,45,-100,0,40,0')).split(',')]
for a in d['arms']:
    if not a['name'].startswith('pin_arm'): continue
    k = a['name'][-1]; tab = {'a': A, 'b': B, 'c': C}[k]
    lo, hi = tracks[k]
    a['model'] = 'kr20_r1810_2'
    a['base_pos'] = [X, 0.0, 0.798]
    a['base_rpy_deg'] = [0, 0, float(p.get('yaw', 0))]
    tr = a['track']; tr['lo'] = round(lo, 3); tr['hi'] = round(hi, 3)
    blo, bhi = lo - 0.3, hi + 0.3
    tr['beam']['size'][1] = round(bhi - blo, 3); tr['beam']['pos'] = [X, round((blo + bhi) / 2, 3), tr['beam']['pos'][2]]
    tr['mass_kg'] = round(21.94 * (bhi - blo), 1)
    st = [0.0] * 36
    for (m, h), t in tab.items(): st[4 * m + h] = t
    a['stops'] = {'pins_take': [nest[k]], 'locate': st, 'pin': st, 'swage': st}
    if p.get('nobb') == '1':
        bs = [float(v) for v in p.get('bstops', '-4.5,-3.0,-2.3,-2.3,-2.0,1.6,2.5,3.8,4.8,4.5').split(',')]
        if k != 'b': a['stops']['strap_bolt'] = bs
        if k == 'b':
            a['tools'] = [t for t in a['tools'] if t['name'] == 'pin_tool']
            a['first_tool'] = 'pin_tool'
    a['wait_joints_deg'] = [float(v) for v in p['wait' + k].split(',')] if ('wait' + k) in p else wait
    a['stow_joints_deg'] = [float(v) for v in p['stow' + k].split(',')] if ('stow' + k) in p else stow
    wt = float(p.get('wt' + k, {'a': -5.2, 'b': 0.0, 'c': 3.5}[k]))
    a['wait_track'] = wt; a['stow_track'] = float(p.get('st' + k, wt))
    a.pop('one_posture_per_module', None)
    ty = float(p.get('tsy', 0.65)) * {'a': -1, 'b': float(p.get('tsb', 1)), 'c': float(p.get('tsc', 1))}[k]; tz = float(p.get('tsz', 0.13))
    for t in a['tools']:
        dx = 0.08 if t['name'] == 'nutrunner' else -0.08
        t['waits_at'] = [dx + float(p.get('tsx', 0.25)), ty, tz]
# Work split: A modules 0-3, B module 4, C modules 5-8; bolts A 0-3, B 4-5, C 6-9.
pts = {'pin_arm_a': '0-15', 'pin_arm_b': '16-23', 'pin_arm_c': '24-35'}
bolts = {'pin_arm_a': p.get('bolta', '0-3'), 'pin_arm_b': p.get('boltb', '4-6'), 'pin_arm_c': p.get('boltc', '7-9')}
if p.get('nobb') == '1': bolts = {'pin_arm_a': '0-4', 'pin_arm_b': None, 'pin_arm_c': '5-9'}
for w in d['work']:
    if w['arm'] in pts:
        if w['kind'] in ('locate', 'pin', 'swage'): w['points'] = pts[w['arm']]
        if w['kind'] == 'strap_bolt': w['points'] = bolts[w['arm']]
d['work'] = [w for w in d['work'] if w.get('points') is not None]
# The pin magazine nests on the lane.
for it in d['items']:
    for b in it['boxes']:
        if b['name'] == 'nest_1': b['pos'][1] = nest['b']
        if b['name'] == 'nest_2': b['pos'][1] = nest['c']
for s in d['stocks']:
    if s['name'] == 'pins_1': s['pick_pos'][1] = nest['b']
    if s['name'] == 'pins_2': s['pick_pos'][1] = nest['c']
if 'order' in p: d['order']['modules'] = [int(v) for v in p['order'].split(',')]
json.dump(d, open('/root/scratch/' + name + '.json', 'w'), indent=1)
