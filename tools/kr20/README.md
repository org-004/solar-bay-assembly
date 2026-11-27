# Probes of KR 20 pin arm places on the 40 ft cell (runs 031 and 034)

Scratch scripts that work on copies of `layouts/` in a scratch `LAYOUT_DIR` (`/root/scratch`) with
absolute paths of the run's machine; kept so that a later run can use them again
(docs/flat-rack-kr20-pin-arms.md). Run 031's candidate, the three-arm layout they start from, is
on branch `org/031-kuka-kr-20-pin-arms` (`layouts/flat_rack_3_kr20_pin_arms_ordered.json`).

- `var.py`: the bay moved along Y, stands kept, removed or moved; `endbar.py`: the end unit's
  column beyond the plain end with a bar at the axis height; `mirror.py`: the mirrored cell.
- `mapvar.py`, `probe.sh`, `feas.py`, `sides.py`, `bolts.py`: one pin arm over the whole track
  doing every hole, probed with `LAYOUT_CHECK_PROBE=pin_arm_a LAYOUT_CHECK_PROBE_WIDE=0`
  (about 1.5 min); for each hole the tracks with a passing pose, and on which side.
- `dg.sh`: one hole's clearance rows at one track (what blocks it).
- `gen.py`, `mk1.sh`: the layout from the probe (stops per hole), checked and planned (about
  1 min); `final.py` writes `layouts/flat_rack_1_kr20_pin_arm_ordered.json`.
- `sumfail.py`: count a checker CSV's failing rows.
