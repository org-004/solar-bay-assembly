import sys
# sides.py feas [shift] : per hole, the passing base tracks below (-Y side) and above (+Y side) it
S = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
yr = [-5.148 + S + 1.144 * r for r in range(10)]
nneg = npos = 0
for line in open(sys.argv[1]):
    i, mh, rest = line.split(' ', 2); i = int(i); m, h = i // 4, i % 4
    tr = eval(rest)
    y = yr[m] + 0.024 if h < 2 else yr[m + 1] - 0.024
    neg = [t for t in tr if t < y]; pos = [t for t in tr if t > y]
    nneg += bool(neg); npos += bool(pos)
    print('%2d %s y=%6.3f  base<hole: %-26s base>hole: %s' % (i, mh, y, neg, pos))
print('holes reachable with the base at -Y of the hole: %d of 36; at +Y: %d of 36' % (nneg, npos))
