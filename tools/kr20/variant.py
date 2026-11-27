import json, sys, copy
# usage: variant.py out_name key=value ...  (x z yaw tubez wait stow lo/hi per arm)
src = '/root/work/031-kuka-kr-20-pin-arms-for-the-r1/solar-bay-assembly/layouts/flat_rack_3_pin_arms_ordered.json'
d = json.load(open(src))
name = sys.argv[1]
p = dict(a.split('=', 1) for a in sys.argv[2:])
d['name'] = name
x = float(p.get('x', -0.38)); z = float(p.get('z', 0.798)); yaw = float(p.get('yaw', 0))
dz = z - 0.798; dx = x - (-0.38)
if 'tubez' in p:
    tz = float(p['tubez']); dtz = tz - d['tube']['axis_z']; d['tube']['axis_z'] = tz
    for b in d['holder']['boxes']:
        if 'column' in b['name']:
            b['size'][2] = round(b['size'][2] + dtz, 4); b['pos'][2] = round(b['pos'][2] + dtz / 2, 4)
        else:
            b['pos'][2] = round(b['pos'][2] + dtz, 4)
for a in d['arms']:
    if not a['name'].startswith('pin_arm'): continue
    a['model'] = 'kr20_r1810_2'
    a['base_pos'] = [x, a['base_pos'][1], z]
    a['base_rpy_deg'] = [0, 0, yaw]
    b = a['track']['beam']
    b['pos'] = [x, b['pos'][1], b['pos'][2] + dz]
    if dz != 0: b['size'][2] = round(b['size'][2] + dz, 4)
    if dz != 0: b['pos'][2] = round(b['pos'][2] - dz/2, 4)
    for k in ('wait', 'stow'):
        if k in p: a[k + '_joints_deg'] = [float(v) for v in p[k].split(',')]
    if p.get('opp') == '1': a['one_posture_per_module'] = True
    t = a['name'][-1]
    if 'lo' + t in p: a['track']['lo'] = float(p['lo' + t])
    if 'hi' + t in p: a['track']['hi'] = float(p['hi' + t])
if 'shift' in p:
    sh = float(p['shift'])
    for b in d['holder']['boxes']:
        if b['name'].startswith('stand_'):
            y = b['pos'][1]
            b['pos'][1] = round(y - sh if y < 0 else y + sh, 4)
json.dump(d, open('/root/scratch/' + name + '.json', 'w'), indent=1)
