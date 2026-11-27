import sys, re, collections
# per hole (locate kind only): tracks with a passing pose, by family
ok = collections.defaultdict(lambda: collections.defaultdict(set)); seen=set()
for line in open(sys.argv[1]):
    m = re.search(r' (\w+) (\S+) (\d+(?:/\d+)?) track (\S+) fam (\d+) sec (\S+) margin (\S+) (\S+) clear (\S+) legs (\w+)', line)
    if not m: continue
    kind, k2, pt, tr, fam, sec, mar, j, cl, legs = m.groups()
    key = k2 + ' ' + pt; seen.add(key)
    if float(mar) >= 5 and float(cl) >= 0.015 and legs == 'pass':
        ok[key][int(fam)].add(round(float(tr), 2))
for key in sorted(seen, key=lambda k: (k.split()[0], [int(x) for x in k.split()[1].split('/')])):
    fams = ok.get(key, {})
    s = '; '.join('f%d: %s' % (f, ','.join('%.1f' % t for t in sorted(v))) for f, v in sorted(fams.items()))
    print(key, '|', s if s else 'NONE')
