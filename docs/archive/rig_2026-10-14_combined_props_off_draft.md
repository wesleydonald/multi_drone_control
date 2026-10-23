# Wednesday 14 Oct: combined M1 props-off additions (DRAFT 11 Oct night; not in the sheet; Wesley's OK needed)

From the rig port's critic round 1 (`/home/wesley/.claude/jobs/50d19a1f/tmp/rigport_critic_r1.md`). Wesley decides
whether these join the 09:00 block. A combined props-off item with his nodes needs his OK. Run the combined items LAST:
the morning's W runs use the main tree's install, and these use the m1-rig tree (two install spaces).

## Terminals (critic F2; shortcuts.md is Wesley's, these are the lines to put there)
```bash
# in the mdc alias (or before it), so T1, T2, T3, RViz and Tejen's terminal share one local-only domain:
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=23
# the fleet helper: refuses unless the fleet manager answers from this terminal within 2 s
fleet() { python3 ~/multi_drone_control/tools/fleet_reach.py --send "$1"; }
```
`tools/preflight.py` and `tools/rig_flight.sh` refuse the same way. Tejen's laptop cannot see this graph: his
commands go in a terminal on this laptop (`ros2 topic pub --once /tejen/drone_command std_msgs/msg/String "{data: ARM}"`).
No `/drone_3/magnet` line in the Part B terminals (Part A's W4/W5 release on the week4 install still uses it): QUAD4's coil listens on `/drone_3/magnet_coil`, written only by
magnet_gate_3 (critic F4), and the RViz MAGNET D4 row is hidden.

TB16's orbit refusal and the rig release guard are on the candidate (m1-comove), not on the m1-rig 0210a27 install.
Neither can be checked props-off on Wednesday; a props-off guard check needs the candidate install (Tue 20).

## The measurements (props off, QUAD4 in hand with its pack in, ring on the floor; every value into the QUAD4 record)
| what | how | gives |
|---|---|---|
| weld_point_z | item 3 three times, the magnet re-seated each time | the mean and spread |
| contact radius and band | >= 10 hand catches: centre, +-2 cm radial and tangential, tilted +-15 deg with the tether slack; `ros2 topic echo /magnet/attach_proof` (distance) | band = max dz + 3 mm, radius = max dxy + 5 mm |
| speed gate / c0 (F7) | hand ramps of the face onto the plate at about 3, 5, 8 cm/s, 3 each, with the default (contact_still is on: type nothing) and once with `contact_still:=false` for comparison; the CONTACT line's c0 | pass if c0 is within 1 mm of the rest point |
| taut length and drop | QUAD4 hanging still; +-10 deg off vertical; a ~2 N pull on the caught magnet | proof_taut_len, magnet_drop_below_quad |
| d_ok | caught face, QUAD4 rocked 0-20 deg | drift spread |
| marker to face | the face flat on the floor | marker_to_face |
| coil | 10 holds and 10 releases, timed from the command | hold/release time |
| mocap | the puck on the magnet (24/6 swaps), QUAD4 at 2.3-2.9 m on a pole, the puck in the net, the face on plate 9 | gaps <= 0.1 s, no jumps > 1 cm |
| load and kill (F1) | the full combined graph (`partner_nodes:=true`) for >= 10 min, nothing armed but his MPC (props off); DISARM from RViz and from T3; kill his MPC alone; Ctrl-C T2 | the F1 bars; QUAD4's FC disarmed within 0.5 s of each |
| aux | Betaflight receiver tab with his stream forwarded | AUX 5, 7-10 at mid, no mode change |

## Item 7c: contact-lift check, props off (after items 3, 4, 7)
Drones 1-3 may stay unpowered: nothing is armed. R = the contact radius above, L = the taut length.
```bash
$A partner:=true partner_contact:=true weld_radius:=R proof_taut_len:=<L-0.003> c7               # T2
ros2 topic echo /drone_3/magnet_gate --once        # T3: partner
ros2 topic pub -t 3 /magnet/command std_msgs/msg/String "{data: 'ON'}"    # our span: the coil is ours
```
T2: `[magnet_gate] OURS from now (first ON from the ring-magnet manager)`. Watch `/magnet/attach_proof`.
- (a) **Caught**: face onto plate 9 (it catches), QUAD4 lowered ~1 cm (tether slack), still 1 s: `CONTACT`; c0's z
  should be ~0 (several mm = contact_still did not take c0 from a still window: F7). Lift straight up at ~2 cm/s
  until taut and the plate rises a few mm (< 1 cm), hold 1 s: `ATTACH: proven ...`, and `HOLDING approach authority`
  (expected: nothing flies). Three catches, one 1-2 cm off centre. OFF is then refused: Ctrl-C and relaunch.
- (b) **No catch**: relaunch (`c7nc`), QUAD4's pack out (coil dead). Touch, lift: `PROOF FAILED (drift)`, never an
  `ATTACH:`. Then rest the face uncaught, tether slack, and lift the ring's plate-9 side 5 mm by hand: no ATTACH (the
  taut check).
- (c) **Noise**: a caught face still 10 s, and the ring alone 10 s.
- Stop rule: any ATTACH in (b): the proof is not safe on the rig; stop the combined M1 and report.
- In flight his lift ends `proof_rise_max` (3 cm) above the measured contact; the first ~1 cm takes up the slack
  (his descent aims 1 cm below), so up to ~3 cm can pull the plate before our proof stops it. Note the plate rise the
  hand lift needs: if under ~1 cm, 0.02 is enough on the rig and tilts the ring less; if over ~1.5 cm, keep 0.03.
