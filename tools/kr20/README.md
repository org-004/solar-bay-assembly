# Probes of KR 20 pin arm places on the 40 ft cell (run 031)

These are scratch scripts that work on copies of v10 in a scratch `LAYOUT_DIR` (`/root/scratch`
in run 031). They are kept on this branch so that a later run can use them again. They are not
on `main`.

- `variant.py`: v10 with KR 20 pin arms at a given x, z, yaw and tube height.
- `cand.py`: the three-arm candidate (stops per hole, tool stands, waits). `final.py` writes
  `layouts/flat_rack_3_kr20_pin_arms_ordered.json` from it.
- `mapvar.py`, `runmapc.sh`, `probe_map.py`, `feas.py`, `sides.py`: one pin arm over the whole
  track doing every hole, probed with `LAYOUT_CHECK_PROBE`. They list, for each hole, the track
  values where a passing pose exists, and on which side of the hole.
- `chk.sh`, `sumfail.py`: run `check_layout` on a scratch layout and count the failing rows.
