# What changed in the controllers since the rig last flew (8 Oct, code b811d90 -> week5)

Nothing in the carry itself changed. Everything below is new on the rig; all of it has flown in the twin (the claim
batch R1638-R1681 and the 12 Oct batch R1682-R1713).

## Our nodes

| Node | Change | What it does on Wednesday | Default |
|---|---|---|---|
| Planner | LAND lands every drone | LAND before takeoff disarms; LAND in the creep ends it with no lift; LAND with QUAD4 unattached takes it back first | on |
| Planner | Freed drone disarmed alone (`departed_ground_idle`) | After a release, QUAD4 lands by itself, is pressed to its resting height and disarmed alone; the fleet's disarm no longer drops it | on |
| Planner | Newcomer fault lands the fleet (`newcomer_fault_land`) | A fault on an unattached QUAD4 disarms QUAD4 alone and lands the carriers (was: disarm all four in the air) | on |
| Planner | Take-back (`partner_contact`) | When Tejen gives up, or on LAND: QUAD4 goes to our tracker, rises clear of the ring and the carriers first, steps out to 1.2 m + rod, lands, disarms alone. On the circle the hold moves with the ring | on in the combined launch |
| Planner | Release guard (rig only) | Before the take-back moves: rise 5 cm above the tether's slack, wait up to 3 s for the tip to leave the plate. If plate 9 rises 2 cm (the magnet still holds): magnet OFF again, follow the ring down, LAND at the slack within 2 s | on, rig |
| Planner | LAND take-back settle (12 Oct) | The LAND take-back waits 0.1 s so a tilt cut's fault is seen first; a cut QUAD4 is never taken back | on |
| Planner | Contact span | Our ring magnet command starts only when QUAD4 rejoins (airborne, ring weld free); never starts an approach of ours | on in the combined launch |
| Tracker | Warm follow | From Tejen's takeoff our drone-3 tracker runs on QUAD4's measured body, so the hand-over is a reference switch, not a cold start | on in the combined launch |
| Tracker | Tilt cut | Disarms QUAD4 alone at 60 deg for 25 ms while Tejen flies it, with a glitch guard (3000 deg/s) | **off** |
| Fleet manager | Startup announce | Publishes its status at launch, so the fleet CLI, preflight and rig_flight see it without a LAND | on |
| Fleet manager | Per-drone disarm | Disarms a drone that reports "down alone" without aborting the fleet | on |
| Mux (QUAD4's radio input) | Take-back switch | On `/drone_3/take_back` the mux feeds QUAD4 from our tracker | on |
| Mux | Hold at idle (rig) | Tejen's throttle is held at armed idle until our planner flies the ring; his aux channels are zeroed | on, rig |
| Mux | Full mission start | With `partner_attached`, the mux is ours from boot and hands QUAD4 to Tejen on our release | only in test 7 |
| Mux | Cut | On a tilt fault: DISARM only, for good | off (with the tilt cut) |
| Magnet manager | Contact-and-lift proof | A catch is confirmed by contact, then a small lift (3 cm cap on the rig) with the tip following the plate; the tether must read taut (`proof_taut_len`) | on in the combined launch |
| Magnet manager | `contact_still` | The contact point is taken from the last 0.05 s of the dwell, with a vertical-speed gate (desk ramps: within 0.2 mm) | on |
| Magnet gate (new) | One coil writer | QUAD4's coil takes Tejen's commands until our span's first ON, then ours only. Between his pickup and his drop, his OFF passes only in his pickup and drop phases | on, rig |
| Launch | Refusals | The combined launch refuses without W (weld_point_z), the taut length, the rod length and his drop height (each within 1 cm of the taut length), his kT, `ROS_LOCALHOST_ONLY=1` and a domain id, or with his supervisor on | - |
| Tools | `load_check.py`, `fleet_reach.py`, `rig_night.py` | The 10-min load score; the fleet CLI refuses unless the fleet manager answers; the night pipeline | - |

## Tejen's stack (our fork, synced to his 8 Oct code; every edit in `src/tejen_mission/CHANGES_FROM_TEJEN.md`)

| Change | What it does | Default |
|---|---|---|
| Contact attach mode | His descent goes to contact (no hand-off at ATTACH_READY); LIFT_PROOF, RING_ATTACHED and TAKEN_BACK phases added; the magnet stays on through the proof | on in the combined launch |
| Give-up | 30 s after his first descent with no confirmed attach, or 3 failed proofs: he hands QUAD4 to our take-back | on |
| Supervisor | Refused on the rig; ARM, TAKEOFF and DISARM are typed | - |
| Magnet after the drop | After his drop, his TAKEN_BACK holds his object magnet OFF (it re-caught the puck in the net in the twin) | on |
| Drop check | Waits up to 2 s for the puck's pose to confirm the drop (`drop_puck_wait_s`; 0 restores his old behaviour) | 2.0 |
| Contact aim | His rig descent aims the face 1 cm below the plate (`contact_dz -0.01`); test 5 types +0.05 | - |
| Self-land after a take-back | Flies out and lands if our mux has not taken QUAD4 1 s after TAKEN_BACK | **off** |
| QUAD4's values | His QUAD2 values (body 12, marker 22) are not used; body 14, marker 24, drop 0.575, kT 22 are typed | - |

## Not changed
- The carry (mode mpc): measured rod pull, 60 deg rod plan, model integral, the affine thrust map, lift 0.15 m/s.
- The release path of tests 8-9: the 8 Oct one (detection 0.06 m, step out 0.5 m, drop on a second loss or 30 deg).
- The envelope: any fault on a carrier still disarms the fleet; no geofence.

## Twin evidence, in one line each
- Hover attach, hover take-back, no-catch hover, LAND during the proof: supported by the reviewer (R1638-R1655).
- R11a rehearsal, LAND in transit, full mission on hover: pass on week5 (R1682, R1684, R1685).
- The take-back race fix: 4 of 4 clean (R1702-R1705).
- Both release bands hold on week5 (R1686 4.6 deg, R1687 7.4 deg).
- Open: Tejen's controller tumbled QUAD4 twice in the twin near the mat (R1673, R1683); the circle reattach is marginal.
