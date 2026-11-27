import json, subprocess
P = "stow=0,10,-130,0,60,0 sta=-4.7 s6_3=4.4 s7_1=4.4 tsb=-1 tsy=1.3 tsz=0.0 nobb=1 wait=75,40,-90,0,20,0 wtc=5.3 stc=4.6 waitc=-90,40,-90,0,20,0 waitb=-160,40,-90,0,90,0"
NAME = 'flat_rack_3_kr20_pin_arms_ordered'
subprocess.run(['python3', '/root/scripts/cand.py', NAME] + P.split(), check=True)
d = json.load(open('/root/scratch/%s.json' % NAME))
d['title'] = "As flat_rack_3_pin_arms_ordered, with the three UR10e pin arms replaced by KUKA KR 20 R1810-2 that reach the holes along the tube from beside them"
d['summary'] = ("The cell on a 40 ft flat rack with no posts and no roof, every arm KUKA (the owner, 2026-11-26). One KUKA KR 210 R2700-2 on a floor track sets the ten rails and the nine modules with one combined gripper, as in v10. "
  "Three KUKA KR 20 R1810-2 on one floor line under the bay's inner half, 0.73 m from the tube's axis, drive the strap bolts and locate, pin and swage. A KR 20 cannot fold to reach a hole straight above its own base as the UR10e did, so each works a hole from 1.2 to 1.7 m along the tube from it, its arm stretched under the bay: "
  "pin arm A modules 0 to 3 (16 holes) and strap bolts 0 to 4, pin arm B modules 4 and 5 (8 holes, no bolts, the pin tool only), pin arm C modules 6 to 8 (12 holes) and strap bolts 5 to 9. "
  "The holder's four stands carry 0.2 m saddles near the end of their stretch away from the rail a pin arm reaches past them, and each pin arm's tools wait 1.3 m along the track from its base. "
  "The pallet, the rail rack, the ports, the cart and the order of work (the field \"order\") are v10's; the pin magazines' nests stand where each pin arm's track reaches. Run 031; docs/flat-rack-kr20-pin-arms.md.")
d['margins']['reason'] = ("As v10: clearance 0.015 m, half again over the floor of 0.010 m, and joint margin at the floor of 5 degrees. The tightest poses that are not near by design pass at 0.015 m or more (run 031, check_layout).")
for a in d['arms']:
    if not a['name'].startswith('pin_arm'): continue
    k = a['name'][-1]
    a['does'] = {
      'a': "Drives strap bolts 0 to 4, then locates the modules 0 to 3, puts in the pins and swages them, changing tools once; it stands 1.2 to 1.7 m along the tube from each hole.",
      'b': "Locates modules 4 and 5, puts in the pins and swages them, with its pin tool only (no bolts, so no tool change under the bay); it stands 1.2 to 1.7 m along the tube from each hole.",
      'c': "Drives strap bolts 5 to 9, then locates the modules 6 to 8, puts in the pins and swages them, changing tools once at its track's +Y end, beyond the bay; it stands 1.2 to 1.7 m along the tube from each hole."}[k]
    a['track']['source'] = "Designed here as an envelope; a belt-driven floor axis standing on the deck, sized for a KR 20 R1810-2 (255 kg). One rail with 3 carriages, drawn as 3 stretches; mass 21.94 kg per metre of beam, v10's figure."
    for t in a['tools']:
        t['source'] = t['source'] + " Its stand rides on the carriage 1.3 m along the track from the base (run 031): a KR 20 cannot reach down to a stand beside its base."
d['holder']['does'] = d['holder']['does'] + " The saddles are 0.2 m long and stand 0.4 m toward -Y from the middle of their stretch, so that a pin arm reaching a rail past a stand passes it where its arm is still low (docs/flat-rack-kr20-pin-arms.md)."
d['order']['note'] = ("v10's order of the modules, kept: pin arm A's modules 0, 1, 2, 3 are the 1st, 2nd, 4th and 8th set, pin arm B's 4 and 5 the 3rd and 5th, pin arm C's 7, 6 and 8 the 6th, 7th and 9th. "
  "A module is taken once its own two rails are bolted, and a pin arm changes to its pin tool and takes its magazine as soon as its bolts are driven (B has no bolts and starts with the pin tool).")
json.dump(d, open('/root/work/031-kuka-kr-20-pin-arms-for-the-r1/solar-bay-assembly/layouts/%s.json' % NAME, 'w'), indent=2)
