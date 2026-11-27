import json, sys
# mapvar.py src.json out_name [lo hi] : one pin arm A over the whole track doing all pin work
src, name = sys.argv[1], sys.argv[2]
lo = float(sys.argv[3]) if len(sys.argv) > 3 else -5.3; hi = float(sys.argv[4]) if len(sys.argv) > 4 else 5.3
d = json.load(open(src)); d['name'] = name
d['arms'] = [a for a in d['arms'] if a['name'] in ('module_arm', 'pin_arm_a')]
a = d['arms'][1]; a['track']['lo'] = lo; a['track']['hi'] = hi
a['wait_track'] = lo; a['stow_track'] = lo
for k in ('locate', 'pin', 'swage', 'pins_take', 'strap_bolt'): a.get('stops', {}).pop(k, None)
a['stops']['pins_take'] = [-5.2]
b = a["track"]["beam"]; b["size"][1] = hi - lo + 0.52; b['pos'][1] = (lo + hi) / 2
d['work'] = [w for w in d['work'] if w['arm'] in ('module_arm', 'pin_arm_a')]
for w in d['work']:
    if w['arm'] == 'pin_arm_a' and w['kind'] in ('locate', 'pin', 'swage'): w['points'] = '0-35'
    if w['arm'] == 'pin_arm_a' and w['kind'] == 'strap_bolt': w['points'] = '0-9'
    if w['arm'] == 'pin_arm_a' and w['kind'] == 'pins_take': w['points'] = '0-2'
json.dump(d, open('/root/scratch/' + name + '.json', 'w'), indent=1)
