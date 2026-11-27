import sys
yr = [-5.148 + 1.144 * r for r in range(10)]
for line in open(sys.argv[1]):
    i, mh, rest = line.split(' ', 2); i = int(i); m, h = i // 4, i % 4
    tr = eval(rest)
    # hole y: holes 0,1 near rail m (+Y side of it), 2,3 near rail m+1
    y = yr[m] + 0.024 if h < 2 else yr[m + 1] - 0.024
    neg = [t for t in tr if t < y]; pos = [t for t in tr if t > y]
    print('%2d %s y=%6.3f  base<hole: %-22s base>hole: %s' % (i, mh, y, neg, pos))
