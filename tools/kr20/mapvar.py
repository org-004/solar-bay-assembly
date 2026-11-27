import json, sys
# mapvar.py src.json out_name : one pin arm A over the whole track doing all pin work
src, name = sys.argv[1], sys.argv[2]
d = json.load(open(src)); d['name'] = name
d['arms'] = [a for a in d['arms'] if a['name'] in ('module_arm', 'pin_arm_a')]
a = d['arms'][1]; a['track']['lo'] = -5.3; a['track']['hi'] = 5.3
for k in ('locate', 'pin', 'swage', 'pins_take'): a.get('stops', {}).pop(k, None)
a['stops']['pins_take'] = [-5.2]
b = a['track']['beam']; b['size'][1] = 10.6 + 0.3; b['pos'][1] = 0.0
d['work'] = [w for w in d['work'] if w['arm'] in ('module_arm', 'pin_arm_a')]
for w in d['work']:
    if w['arm'] == 'pin_arm_a' and w['kind'] in ('locate', 'pin', 'swage'): w['points'] = '0-35'
    if w['arm'] == 'pin_arm_a' and w['kind'] == 'strap_bolt': w['points'] = '0-9'
    if w['arm'] == 'pin_arm_a' and w['kind'] == 'pins_take': w['points'] = '0-2'
json.dump(d, open('/root/scratch/' + name + '.json', 'w'), indent=1)
