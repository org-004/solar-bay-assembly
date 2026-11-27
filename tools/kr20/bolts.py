import sys, re, collections
# bolts.py probe.txt : per strap bolt, the tracks with a passing pose
ok = collections.defaultdict(set)
for line in open(sys.argv[1]):
    m = re.search(r' strap_bolt strap_bolt (\d+) track (\S+) fam (\d+) sec (\S+) margin (\S+) (\S+) clear (\S+) legs (\w+)', line)
    if not m: continue
    b, tr, fam, sec, mar, j, cl, legs = m.groups()
    if float(mar) >= 5 and float(cl) >= 0.015 and legs == 'pass': ok[int(b)].add(round(float(tr), 2))
for b in range(10): print('bolt', b, sorted(ok[b]))
