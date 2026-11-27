import json, sys
# mirror.py src.json out_name S : 031's candidate with the bay moved S m toward -Y, the stands' saddles 0.4 m toward +Y
# of their stretch's middle (031 had them 0.4 m toward -Y), the end unit at the tube's +Y end (bay_hold_10) with its
# column 0.7 m beyond that end at the deck's -X edge, joined to the head by a bar at the tube's axis height.
src, name, S = sys.argv[1], sys.argv[2], float(sys.argv[3])
d = json.load(open(src)); d['name'] = name
h = d['holder']; az = d['tube']['axis_z']
d['tube']['plain_end_y'] = round(d['tube']['plain_end_y'] - S, 4)
pe = d['tube']['plain_end_y']; ee = pe + 10.616
for b in h['boxes']:
    if b['name'].startswith('stand_'): b['pos'][1] = round(b['pos'][1] + 0.8 - S, 4)
col = next(b for b in h['boxes'] if b['name'] == 'end_column'); head = next(b for b in h['boxes'] if b['name'] == 'end_head')
head['pos'][1] = round(ee + 0.122, 4)
X, W, back = -1.185, 0.06, 0.7
bot = col['pos'][2] - col['size'][2] / 2; top = az + 0.04
col['size'] = [W, 0.1, round(top - bot, 4)]; col['pos'] = [X, round(ee + back, 4), round((top + bot) / 2, 4)]
y0, y1 = ee + 0.19, ee + back + 0.05; x0, x1 = X - W / 2, -1.04
h['boxes'].append({'name': 'end_bar', 'size': [round(x1 - x0, 4), round(y1 - y0, 4), 0.08], 'pos': [round((x0 + x1) / 2, 4), round((y0 + y1) / 2, 4), az]})
h['angle_pin_hold'] = 10
m = d['arms'][0]; LO = float(sys.argv[4]) if len(sys.argv) > 4 else -5.2
for k in ('module_set', 'rail_set'): m['stops'][k] = [round(max(v - S, LO), 4) for v in m['stops'][k]]
lo = LO; hi = round(m['track']['hi'] - S, 4); m['track']['lo'], m['track']['hi'] = lo, hi
bl, bh = lo - 0.85, hi + 0.55
m['track']['beam']['size'][1] = round(bh - bl, 3); m['track']['beam']['pos'][1] = round((bl + bh) / 2, 3)
for port in d['ports']:
    if 'zone' in port and port['name'] in ('tubes_in', 'bays_out'): port['zone']['pos'][1] = round(port['zone']['pos'][1] - S, 4)
for it in d['items']:
    if it['name'] == 'transfer_cart':
        for b in it['boxes']: b['pos'][1] = round(b['pos'][1] - S, 4)
json.dump(d, open('/root/scratch/' + name + '.json', 'w'), indent=1)
