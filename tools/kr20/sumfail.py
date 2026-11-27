import csv, sys, collections
rows = list(csv.reader(open(sys.argv[1])))
hdr = rows[0]; body = [r for r in rows[1:] if len(r) == len(hdr)]
f = [r for r in body if r[7] != 'pass']
c = collections.Counter((r[0], r[2]) for r in f)
print(len(body), 'rows', len(f), 'fail', dict(c))
for r in f[:int(sys.argv[2]) if len(sys.argv) > 2 else 8]: print('  ', ','.join(r)[:260])
