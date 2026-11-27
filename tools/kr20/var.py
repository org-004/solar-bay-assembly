import json, sys
# var.py src.json out_name key=value ... : variants of 031's KR 20 candidate.
#  shift=S   the bay (tube, holder, module arm's set stops and track) moved S m along +Y
#  keep=2,4  keep only these holder stands (grips and boxes)
#  colx=X    stand columns moved to x=X (an arm from the column to the saddle under the tube)
#  nocol=1   stand columns removed (upper bound)
src, name = sys.argv[1], sys.argv[2]
p = dict(a.split('=', 1) for a in sys.argv[3:])
d = json.load(open(src)); d['name'] = name
S = float(p.get('shift', 0))
h = d['holder']
if 'keep' in p:
    keep = [int(v) for v in p['keep'].split(',')] if p['keep'] else []
    h['grips'] = [g for g in h['grips'] if g['hold'] in keep]
    h['boxes'] = [b for b in h['boxes'] if not b['name'].startswith('stand_') or int(b['name'].split('_')[1]) in keep]
if 'nocol' in p:
    h['boxes'] = [b for b in h['boxes'] if 'column' not in b['name']]
if 'colx' in p:
    X = float(p['colx']); W = float(p.get('colw', 0.08)); new = []
    for b in h['boxes']:
        if b['name'].startswith('stand_') and 'column' in b['name']:
            top = b['pos'][2] + b['size'][2] / 2; bot = b['pos'][2] - b['size'][2] / 2
            b['size'][0] = W; b['pos'][0] = X
            ah = 0.06  # the arm under the saddle, from the column to the tube's axis
            x0, x1 = min(X - W / 2, -1.08), max(X + W / 2, -1.08 + 0.11)
            new.append({'name': b['name'].replace('column', 'arm'), 'size': [round(x1 - x0, 4), b['size'][1], ah],
                        'pos': [round((x0 + x1) / 2, 4), b['pos'][1], round(top - ah / 2, 4)]})
        new.append(b)
    h['boxes'] = new
if S:
    d['tube']['plain_end_y'] = round(d['tube']['plain_end_y'] + S, 4)
    for b in h['boxes']: b['pos'][1] = round(b['pos'][1] + S, 4)
    m = d['arms'][0]
    MH = float(p.get('mhi', m['track']['hi'] + S))
    for k in ('module_set', 'rail_set'): m['stops'][k] = [round(min(v + S, MH), 4) for v in m['stops'][k]]
    lo = round(float(p.get('mlo', m['track']['lo'] + S)), 4); hi = round(float(p.get('mhi', m['track']['hi'] + S)), 4)
    m['track']['lo'], m['track']['hi'] = lo, hi
    bl, bh = lo - 0.85, hi + 0.55
    m['track']['beam']['size'][1] = round(bh - bl, 3); m['track']['beam']['pos'][1] = round((bl + bh) / 2, 3)
    for port in d['ports']:
        if 'zone' in port and port['name'] in ('tubes_in', 'bays_out'): port['zone']['pos'][1] = round(port['zone']['pos'][1] + S, 4)
    for it in d['items']:
        if it['name'] == 'transfer_cart':
            for b in it['boxes']: b['pos'][1] = round(b['pos'][1] + S, 4)
json.dump(d, open('/root/scratch/' + name + '.json', 'w'), indent=1)
