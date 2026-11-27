import sys, re, collections
# feasible tracks per hole point index (module*4+hole), intersection over locate/pin/swage
ok = collections.defaultdict(lambda: collections.defaultdict(set))
for line in open(sys.argv[1]):
    m = re.search(r' (\w+) (\S+) (\d+(?:/\d+)?) track (\S+) fam (\d+) sec (\S+) margin (\S+) (\S+) clear (\S+) legs (\w+)', line)
    if not m: continue
    kind, k2, pt, tr, fam, sec, mar, j, cl, legs = m.groups()
    if float(mar) >= 5 and float(cl) >= 0.015 and legs == 'pass':
        ok[pt][k2].add(round(float(tr), 1))
res = {}
for pt, kinds in ok.items():
    if '/' not in pt: continue
    s = None
    for k in ('locate',):
        v = kinds.get(k, set()); s = v if s is None else s & v
    mm, h = map(int, pt.split('/')); res[mm * 4 + h] = sorted(s)
for i in range(36):
    print(i, '%d/%d' % (i // 4, i % 4), res.get(i, []))
