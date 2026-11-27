import json, sys
# endbar.py in.json out_name back colx colw : the end unit's column 'back' m beyond the plain end (-Y) at x=colx,
# joined to the head at the plain end by a bar along Y at the tube's axis height.
d = json.load(open(sys.argv[1])); d['name'] = sys.argv[2]
back, X, W = float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])
pe = d['tube']['plain_end_y']; h = d['holder']; az = d['tube']['axis_z']
col = next(b for b in h['boxes'] if b['name'] == 'end_column')
bot = col['pos'][2] - col['size'][2] / 2; top = az + 0.04
col['size'] = [W, 0.1, round(top - bot, 4)]; col['pos'] = [X, round(pe - back, 4), round((top + bot) / 2, 4)]
y0, y1 = pe - back - 0.05, pe - 0.19
x0, x1 = min(X - W / 2, -1.12), -1.04
h['boxes'].append({'name': 'end_bar', 'size': [round(x1 - x0, 4), round(y1 - y0, 4), 0.08], 'pos': [round((x0 + x1) / 2, 4), round((y0 + y1) / 2, 4), az]})
json.dump(d, open('/root/scratch/' + sys.argv[2] + '.json', 'w'), indent=1)
