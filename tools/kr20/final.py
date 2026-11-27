import json, sys
# final.py : writes layouts/flat_rack_1_kr20_pin_arm_ordered.json from the scratch variant k1f (mk1.sh)
NAME = 'flat_rack_1_kr20_pin_arm_ordered'
d = json.load(open('/root/scratch/k1f.json')); d['name'] = NAME
d['title'] = "As flat_rack_3_pin_arms_ordered, every arm KUKA: one KR 20 R1810-2 pin arm along the whole bay, which stands 1.0 m further toward +Y"
d['summary'] = ("The cell on a 40 ft flat rack with no posts and no roof, every arm KUKA (the owner, 2026-11-26). One KUKA KR 210 R2700-2 on a floor track sets the ten rails and the nine modules with one combined gripper, as in v10. "
  "One KUKA KR 20 R1810-2 on a floor track under the bay's inner half, 0.73 m from the tube's axis, drives the ten strap bolts and then locates, pins and swages all 36 holes. "
  "A KR 20 cannot fold to reach a hole above its own base as the UR10e did: it works a hole from 1.2 to 1.7 m along the tube, its arm stretched under the bay, and it reaches every hole from the -Y side (its base at -Y of the hole), so it never swings from one side to the other under the bay. "
  "For that the bay stands 1.0 m further toward +Y than v10's (the tube's plain end at y -4.308, its other end 0.21 m beyond the deck, which the owner allows) so that rail 0 is reached from -Y with the base on the deck, and the end unit's column stands 0.7 m beyond the plain end at the deck's -X edge, joined to its head by a bar at the tube's axis height, so that the arm passes under the plain end. "
  "Two or three KR 20 on the one line of track would need overlapping stretches of track: arms that all reach from -Y need about 0.05 m of overlap at each boundary plus a carriage, and arms facing each other cannot pass (run 034, docs/flat-rack-kr20-pin-arms.md). "
  "The pallet, the rail rack, the ports and the cart are v10's; the order of work is the modules in bay order. Run 034.")
d['margins']['reason'] = "As v10: clearance 0.015 m, half again over the floor of 0.010 m, and joint margin at the floor of 5 degrees (run 034, check_layout)."
a = d['arms'][1]
a['does'] = ("Drives strap bolts 0 to 9, then changes to its pin tool, takes its magazine and locates, pins and swages the 36 holes of modules 0 to 8 in bay order; "
  "it stands 1.2 to 1.7 m toward -Y from each hole (the stops are the probe's, tools/kr20).")
a['track']['source'] = "Designed here as an envelope; a belt-driven floor axis standing on the deck, sized for a KR 20 R1810-2 (255 kg); mass 21.94 kg per metre of beam, v10's figure."
d['holder']['does'] = ("Four stands carry the tube in saddles that grip its lower half on the stretches between rails; an end unit pins the plain end's rivet holes and so fixes the angle. "
  "The saddles are 0.2 m long and stand 0.4 m toward -Y from the middle of their stretch (run 031). The end unit's column stands 0.7 m beyond the plain end at the deck's -X edge and carries its head on a bar at the tube's axis height, "
  "so that the pin arm reaching rail 0 from -Y passes under the plain end (run 034, docs/flat-rack-kr20-pin-arms.md).")
d['order']['note'] = ("The modules in bay order, 0 to 8: the one pin arm follows the module arm along the bay. A module is taken once its own two rails are bolted, and the pin arm changes to its pin tool and takes its magazine as soon as its bolts are driven.")
json.dump(d, open('/root/work/034-a-kr-20-40-ft-layout-that-r1/repo/layouts/%s.json' % NAME, 'w'), indent=2)
print('ok')
