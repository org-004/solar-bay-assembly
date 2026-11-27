"""shared.py out.json [key=value ...]: run 035's KR 20 pin arms on one shared beam.

Starts from run 034's layouts/flat_rack_1_kr20_pin_arm_ordered.json (one KR 20, every hole
from -Y, the stops of its probe) and splits its holes and strap bolts over pin arms A, B and C
on the same line of track (x -0.35, z 0.798, all facing +Y). B and C ride A's beam
(track.shares_beam_with, src/layout.h); the beam is drawn once for the whole line, as long as
the three tracks need, and every arm's track gives the same box. Keys (defaults in D below):
  mods.a=0-3 mods.b=4-6 mods.c=7-8   modules per arm (holes 4m..4m+3), a range
  bolts.a=0-4 ...                    strap bolts per arm ("" for none: pin tool only)
  wait.a=-5.7 ...                    wait track; nest.a=-5.2 ...: the arm's magazine nest (y)
  ts.a=-1.3 ...                      where the arm's waiting tools ride, along Y from the base;
                                     tz.a: their height, tx.a=<nutrunner>,<pin tool>: across
  ps.a=<track>                       the stop of the magazine take (default: the nest's y)
  wj.a=75,40,-90,0,20,0 ...          wait joints (deg); stow.a=-4.7: stow track
  plo.b=0.4 phi.b=0.4                track room below and above the arm's stops (A: none below)
  sNN=<track>                        override the stop of hole NN; bNN: of bolt NN
"""
import json, sys

ROOT = sys.argv[0].rsplit('/tools/', 1)[0] if '/tools/' in sys.argv[0] else '.'
out = sys.argv[1]
D = {'mods.a': '0-3', 'mods.b': '4-6', 'mods.c': '7-8',
     'bolts.a': '0-4', 'bolts.b': '5-6', 'bolts.c': '7-9',
     'wait.a': '-5.7', 'wait.b': '1.4', 'wait.c': '4.6',
     'nest.a': '-5.2', 'nest.b': '1.4', 'nest.c': '4.6',
     'ts.a': '-1.3', 'ts.b': '-1.3', 'ts.c': '1.3',
     'wj.a': '75,40,-90,0,20,0', 'wj.b': '75,40,-90,0,20,0', 'wj.c': '-90,40,-90,0,20,0',
     'stow.a': '-4.7', 'stow.b': '', 'stow.c': '',
     'name': 'flat_rack_3_kr20_pin_arms_shared_ordered'}
P = dict(D)
P.update(a.split('=', 1) for a in sys.argv[2:])
rng = lambda t: list(range(int(t.split('-')[0]), int(t.split('-')[-1]) + 1)) if t else []
d = json.load(open(ROOT + '/layouts/flat_rack_1_kr20_pin_arm_ordered.json'))
d['name'] = P['name']
base = next(a for a in d['arms'] if a['name'] == 'pin_arm_a')
hole_stop = list(base['stops']['locate'])
bolt_stop = list(base['stops']['strap_bolt'])
for k, v in P.items():
    if k[0] in 'sb' and k[1:].isdigit():
        (hole_stop if k[0] == 's' else bolt_stop)[int(k[1:])] = float(v)
arms = {}
KS = [k for k in 'abc' if P['mods.' + k] or P['bolts.' + k]]  # mods.b= bolts.b= : no arm B
for k in KS:
    a = json.loads(json.dumps(base))
    a['name'] = 'pin_arm_' + k
    mods, bolts = rng(P['mods.' + k]), rng(P['bolts.' + k])
    holes = [4 * m + h for m in mods for h in range(4)]
    for kind in ('locate', 'pin', 'swage'):
        a['stops'][kind] = [hole_stop[i] if i in holes else 0.0 for i in range(36)]
    a['stops']['pins_take'] = [float(P['nest.' + k])]
    if bolts:
        a['stops']['strap_bolt'] = list(bolt_stop)
    else:
        a['stops'].pop('strap_bolt')
        a['tools'] = [t for t in a['tools'] if t['name'] == 'pin_tool']
        a['first_tool'] = 'pin_tool'
    for t in a['tools']:
        t['waits_at'][1] = float(P['ts.' + k])
        if 'tz.' + k in P: t['waits_at'][2] = float(P['tz.' + k])
        if 'tx.' + k in P: t['waits_at'][0] = float(P['tx.' + k].split(',')[t['name'] == 'pin_tool'])
    if 'ps.' + k in P: a['stops']['pins_take'] = [float(P['ps.' + k])]
    used = [hole_stop[i] for i in holes] + [bolt_stop[b] for b in bolts] + a['stops']['pins_take']
    wt = float(P['wait.' + k]); st = float(P['stow.' + k] or wt)
    a['wait_track'] = wt; a['stow_track'] = st
    a['wait_joints_deg'] = [float(v) for v in P['wj.' + k].split(',')]
    lo, hi = min(used + [wt, st]), max(used + [wt, st])
    # Room for the planner: a move between two stops may run the carriage past them.
    lo -= float(P.get('plo.' + k, '0' if k == 'a' else '0.4'))
    hi += float(P.get('phi.' + k, '0.4'))
    a['track']['lo'] = round(lo, 3); a['track']['hi'] = round(hi, 3)
    m0, m1 = (min(mods), max(mods)) if mods else (0, -1)
    a['does'] = ('Drives strap bolts %s, then changes to its pin tool, takes its magazine and '
                 % P['bolts.' + k] if bolts else 'Takes its magazine and ') + \
        ('locates, pins and swages the %d holes of modules %d to %d; it stands 1.2 to 1.7 m '
         'toward -Y from each hole (the stops are run 034\'s probe\'s, tools/kr20).' % (len(holes), m0, m1))
    a['_mods'], a['_bolts'], a['_holes'] = mods, bolts, holes
    arms[k] = a
# One beam for the line: from the lowest track end less 0.3 m to the highest plus 0.3 m.
blo = min(a['track']['lo'] for a in arms.values()) - 0.3
bhi = max(a['track']['hi'] for a in arms.values()) + 0.3
for k, a in arms.items():
    tr = a['track']
    tr['beam']['size'][1] = round(bhi - blo, 3); tr['beam']['pos'][1] = round((blo + bhi) / 2, 3)
    if k == 'a':
        tr['mass_kg'] = round(21.94 * (bhi - blo), 1)
    else:
        tr['shares_beam_with'] = 'pin_arm_a'
        # The beam is pin_arm_a's; this arm adds its carriage (034's beam-and-carriage figure
        # per metre over the carriage and its 0.3 m ends).
        tr['mass_kg'] = round(21.94 * 1.1, 1)
        tr['source'] = ('Rides pin_arm_a\'s beam (run 035, shares_beam_with): the same box; its '
                        'mass is its carriage only, 21.94 kg per metre over 1.1 m.')
d['arms'] = [x for x in d['arms'] if not x['name'].startswith('pin_arm')]
for k in [k for k in P.get('armorder', 'abc') if k in KS]:
    a = arms[k]
    for f in ('_mods', '_bolts', '_holes'):
        a.pop(f)
    d['arms'].append(a)
# Magazines: one stock and one nest per arm.
pins = next(s for s in d['stocks'] if s['name'] == 'pins')
lane = next(i for i in d['items'] if i['name'] == 'pin_lane')
nest0 = next(b for b in lane['boxes'] if b['name'] == 'nest_0')
SK = [k for k in P.get('stockorder', 'abc') if k in KS]  # which arm takes which magazine stock
for n, k in enumerate(SK):
    y = float(P['nest.' + k])
    if n == 0:
        pins['pick_pos'][1] = y; nest0['pos'][1] = y
        continue
    s = json.loads(json.dumps(pins)); s['name'] = 'pins_%d' % n
    s['how'] = s['how'].replace('pin_arm_a', 'pin_arm_' + k)
    d['stocks'].append(s)
    b = json.loads(json.dumps(nest0)); b['name'] = 'nest_%d' % n
    lane['boxes'].append(b)
    s['pick_pos'][1] = y; b['pos'][1] = y
# Work: each pin arm its holes and bolts, its own magazine stock.
work = []
for w in d['work']:
    if w['arm'] != 'pin_arm_a':
        work.append(w); continue
    for k in KS:
        n = SK.index(k)
        a = next(x for x in d['arms'] if x['name'] == 'pin_arm_' + k)
        v = dict(w); v['arm'] = a['name']
        if w['kind'] in ('locate', 'pin', 'swage'):
            ms = rng(P['mods.' + k])
            if not ms: continue
            v['points'] = '%d-%d' % (4 * ms[0], 4 * ms[-1] + 3)
        elif w['kind'] == 'strap_bolt':
            if not P['bolts.' + k]: continue
            v['points'] = P['bolts.' + k]
        elif w['kind'] == 'pins_take':
            v['points'] = str(n)
        work.append(v)
d['work'] = work
d['title'] = 'Three KR 20 pin arms on one shared beam on the 40 ft flat rack (run 035)'
d['summary'] = ('Run 034\'s all-KUKA cell (bay 1.0 m toward +Y, the end unit\'s column 0.7 m beyond '
                'the plain end) with its holes split over three KR 20 R1810-2 pin arms on one line '
                'of track, every hole from -Y as in 034: A modules %s and bolts %s, B modules %s and '
                'bolts %s, C modules %s and bolts %s. B and C ride A\'s beam (shares_beam_with); '
                'carriage to carriage stays checked and the schedule keeps them apart.'
                % (P['mods.a'], P['bolts.a'] or 'none', P['mods.b'], P['bolts.b'] or 'none',
                   P['mods.c'], P['bolts.c'] or 'none'))
d['order']['note'] = ('The modules in bay order, 0 to 8: each pin arm follows the module arm along '
                      'its stretch of the bay. A module is taken once its own two rails are bolted, '
                      'and a pin arm changes to its pin tool and takes its magazine as soon as its '
                      'bolts are driven.')
if 'pintools' in P: d['order']['pin_tools'] = P['pintools']
json.dump(d, open(out, 'w'), indent=1)
print(out, {k: (a['track']['lo'], a['track']['hi'], a['wait_track']) for k, a in arms.items()},
      'beam', round(blo, 3), round(bhi, 3))
