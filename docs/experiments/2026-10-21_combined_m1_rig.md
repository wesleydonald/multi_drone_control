# Combined M1 on the rig: Tejen flies QUAD4 from its mat through the pickup, drop and a contact attach; our motion capture confirms; bumpless hand-over (DRAFT, 11 Oct)

DRAFT for Wesley. Not frozen. The bars below are written before any combined flight; they freeze when Wesley signs this
card. Twin claim: card 2026-10-10_combined_m1_contact (rev 5, m1c_* runs, the m1-final tree). Rig port: branch m1-rig
(dd33dca + the critic round 1 items built 11 Oct night). Critic round 1 on the port:
`/home/wesley/.claude/jobs/50d19a1f/tmp/rigport_critic_r1.md` (verdict for this flight: NO-GO until the items below).

## 1. Question
On the rig, does Tejen's controller take QUAD4 from its mat through the pickup, the high transit and the drop into the
ring's net, bring the magnet onto plate 9 and prove the catch with a small lift, so that our motion capture alone
confirms the attach and our tracker takes QUAD4 over without a jolt; and when no attach happens, does our tracker take
QUAD4 back and land it?

## 2. Before the first combined flight (each a gate; any one open = no combined flight)
| # | item | state 11 Oct night |
|---|---|---|
| F5 | rig tree = m1-final merged with m1-rig; twin claim launch dumps identical; one m1c_hover + one m1c_nocatch_hover on the merged tree; the reviewer passes the m1-final table first; the contract's rig lift paragraph rewritten | waits for m1-final |
| F2 | every terminal in the fleet's domain (`fleet`, preflight, rig_flight refuse otherwise); the kill path repeated from T3 and from his terminal | built (tools/fleet_reach.py); sheet lines drafted |
| F3 | the mux holds his throttle at armed idle until our planner flies the ring; ARM FAILED says "his ARM first" | built |
| F4 | QUAD4's radio latch on a private topic only magnet_gate_3 writes; RViz MAGNET D4 hidden; no `/drone_3/magnet` line on a combined-day sheet | built |
| F9 | after the pickup his OFF reaches the coil only at the pickup site (his retry) or in DROP_OBJECT / CLEAR_DROP_ZONE; gate switch and span wait for the puck released; the drop ends on the measured puck (stuck = give-up); his supervisor refused | built |
| F7 | contact_still (c0 from the dwell's last 0.05 s + vertical speed gate) decided with Wednesday's hand ramps | built; on by default since 0210a27 (Wesley 11 Oct); Wednesday's hand ramps check the default |
| F8 | QUAD4's body-to-marker, marker-to-face, taut length, weld_point_z, contact radius (Wednesday) and his kT with and without the puck (Thursday R10a) typed from one QUAD4 record | values open |
| F1 | the full combined graph at wall clock on the rig laptop, >= 10 min, passes the load bars (section 5.1) | Wednesday props-off, if Wesley OKs the combined item |
| F6 | warm-tracker faults and his stuck TAKEN_BACK handled (the tilt-cut card), or accepted in writing with the sheet line "T2 says 'taken back, but our tracker is not flying yet': clear the cage, DISARM, QUAD4 falls" | **accepted in writing** (Wesley 11 Oct): the DISARM sheet line; warm_fault_not_live stays off |
| F1-new | new authority, NOT built (needs a card + critic): the mux takes QUAD4 back when his stream is silent > 0.2 s while our tracker is live; our planner takes it back when his MPC latches its reference fault | listed only |

## 3. The flights, in order (one variable each)
1. **R11a no-catch rehearsal (F10)**: the whole mission with `partner_args:='contact_dz:=0.03 ...'`: his descent holds the
   marker 3 cm above plate 9, so no catch is possible. If R10a showed his height error over 2 cm, use
   `contact_dz:=0.05` (critic round 2 A8). It exercises the pickup, transit, drop, the coil hand-over at
   his rejoin, the 30 s give-up and our take-back, and measures his hold error over the moving plate and plate 9's noise
   with QUAD4 overhead. Ring hovering.
2. **R11b contact, hover**: `contact_dz` -0.01 (1 cm below the face); twice.
3. **R11c contact, circle 0.125 m/s**: only after 2 of 2 R11b pass, and only with `contact_rig_orbit:=true`.
   - That needs Wesley's word after the reviewer passes the final table with the co-moving take-back (TB6 passed 11 Oct).
   - His TAKEOFF comes before the orbit starts. Twin R1625: otherwise the ring passes 0.12 m over QUAD4.
Between flights: relaunch T2, his nodes come with it (`partner_nodes:=true`).

## 4. The command (QUAD4 values from the record; R = contact radius, L = taut length)
```bash
$A partner:=true partner_contact:=true partner_nodes:=true weld_radius:=R proof_taut_len:=<L-0.003> \
   weld_point_z:=<W> attach_cable_len:=<L-0.04> \
   partner_args:='magnet_drop_below_quad:=<L> mpc_thrust_ratio:=<kT> [contact_dz:=0.03] [drop_puck_wait_s:=0]' r11a
```
**The values the launch now refuses untyped** (critic round 2 A3):
- `weld_point_z` = W, the contact height from Wednesday 09:00 item 3.
- `attach_cable_len` = L - 0.04 (QUAD4's rod; the pivot sits 4 cm below the body). The release guard's tip-to-body
  check allows 3 cm either side of it: typed wrong by more than that, every real release times out and the fleet lands.
- `drop_puck_wait_s:=0` only if Wednesday's item 11 showed the puck is not seen in the net.
Order: his ARM (`/tejen/drone_command ARM`, QUAD4 armed idle on its mat) -> `fleet arm` -> `fleet takeoff` -> the ring
at 1.0 m (T2: `our planner flies the ring`) -> his TAKEOFF. End of every flight: `fleet disarm` and his DISARM.

## 5. Bars (per flight; frozen when signed; scored from the rig bag and the logs)
### 5.1 Load (F1; the first R11a, and any flight whose laptop load is new)
- **Confirms**: max gap < 0.15 s on `/drone_3/ELRSCommand_tejen`, `/drone_3/ELRSCommand` and each `/drone_i/ELRSCommand_diss`
  from his TAKEOFF to the switch or the take-back; < 0.3 s on `/join_planner/reference`; zero "External-reference fault
  latched"; zero pose timeouts. **Falsifies**: any one; the combined flights stop for the day.

### 5.2 Contact (his descent start to `/magnet/contact`)
- **Confirms**: within 10 s, or by the third try. **Reported**: 10 to 30 s. **Falsifies**: no contact by his give-up (the
  take-back must still pass, 5.6). R11a: zero contacts expected (a contact at +3 cm falsifies the contact check).
- Logged: c0 against the rest point (the F7 measure), contact distance, tries, his alignment aborts.

### 5.3 Proof
- **Confirms**: proven within 3 s of the last contact; QUAD4's body-to-marker >= the taut length at the proof; plate 9
  rise <= 2 cm (his lift ends `proof_rise_max` above the measured contact, 3 cm on the rig, after ~1 cm of slack:
  Wesley may set 0.02 after Wednesday's hand lifts).
- **Falsifies**: any `ATTACH:` in R11a; a proof on a contact whose c0 is more than 3 mm above the rest point.

### 5.4 Hand-over (QUAD4's applied command, last 0.1 s before against the first 0.3 s after the switch)
- **Confirms**: the mux switches 0 to 0.2 s after `/magnet/object_attached`, and only then; throttle step <= 10 % of
  hover, roll/pitch <= 2 deg, yaw rate <= 10 deg/s, heading change over 1 s <= 3 deg; QUAD4 tilt at the proof <= 3 deg.
- **Falsifies**: > 25 %, > 5 deg, > 30 deg/s, > 8 deg.

### 5.5 Ring and clearance
- Ring tilt peak contact..switch+8 s **confirms** <= 5 deg, **falsifies** > 5 deg; ring dip **confirms** <= 1 cm.
- QUAD4's body to any carrier body, his TAKEOFF to the switch: **confirms** >= 0.35 m, **falsifies** < 0.30 m (the
  rig stop rule). QUAD4's body and tether to any carrier rod until contact, and the puck while carried: **confirms**
  >= 0.15 m, **falsifies** < 0.10 m.

### 5.6 Take-back (R11a always; any flight without an attach)
**The path as built** (combined card rev 10 §3b):
- **Release guard first** (`real:=true`): the rise waits until the tip has left plate 9 and the plate has not followed.
- **Hover:**
  - It rises first if the tip is down at the ring: to the higher of the rim clearance and the highest carrier + 0.4 m.
  - Then it steps out to 1.2 m from the ring centre (plus the tether while the puck is on).
  - Then it comes down and is disarmed alone.
- **Circle** (only under `contact_rig_orbit`): the zone rule and the move with the ring, then the decouple and the path
  clear.

**Confirms:**
- `/drone_3/take_back` is true within 0.1 s of his give-up or our LAND, and the mux reads ours within 0.2 s.
- The coil goes OFF only after our span began; before it, the puck stays on.
- The TB4 bars:
  - rod-segment clearance >= 0.15 m and body clearance >= 0.35 m to every carrier;
  - attitude step <= 2 deg at the take-back and at the decouple;
  - ring tilt <= 5 deg;
  - QUAD4 down within 60 s.

**Falsifies:**
- "taken back, but our tracker is not flying yet" for more than 1 s (F6);
- the coil OFF before the span;
- rod < 0.10 m or body < 0.30 m;
- an attitude step > 5 deg or ring tilt > 5 deg;
- QUAD4 not down within 60 s;
- a RELEASE GUARD failure with plate 9 rising 2 cm (the coil held it). The fleet lands, with QUAD4 following the ring
  down at the tether's slack.

**Reported, not a stop** (Wesley 11 Oct, critic round 2 A1): a guard timeout with plate 9 still. QUAD4 follows down
and lands, and the run is scored on the other bars.

**Reported, no bar:** the overflight time and the minimum tip-to-carrier gap. Rotor wash is a rig-only question.

### 5.7 Authority (every flight)
- Zero `/drone_3/magnet_coil` writers but magnet_gate_3; zero his OFFs reaching the coil in the transit or over the
  ring once the puck is taken (`[magnet_gate] his OFF held` lines are fine); zero "HOLDING approach authority" with QUAD4 airborne.

## 6. Stop rules (the sheet's, plus)
- To stop his attach, publish his give-up (`/join_planner/give_up` true); the ring stays at hover. LAND only as an
  emergency, because a coil that holds plate 9 under LAND tilts the ring (critic round 2 A2).
- After LANDED, if drone 4's mux still reads partner, clear the cage and press DISARM. This is accepted in writing by
  Wesley, 11 Oct (F6).
- Known, not fixed for 21-22 Oct (with A2): right after a failed guard's LAND, the slack target sits up to 1.7 cm
  above taut for ~0.5 s while the ring steps to its landing speed. A coil that still holds briefly tugs plate 9.
- Any falsifier in 5.1, 5.6 or 5.7: no more combined flights today.
- QUAD4 frozen more than 5 s outside a hold phase: LAND (F1).
- A pose gap over 0.1 s on 14 / 24 / 8 with QUAD4 overhead ends M1 for the day.

## 7. Repeats
R11a once; R11b twice; R11c twice. The stop rules are in section 6 (the twin's "two fails" rule was removed 11 Oct).

## 8. The QUAD4 record (F8; not built)
One YAML per measuring day, e.g. `configs/rig/quad4_2026-10-14.yaml`, signed by Wesley, read by one launch argument
(`quad4_record:=`) that sets our `weld_point_z`, `weld_radius`, `proof_taut_len` and his `magnet_drop_below_quad`,
`mpc_thrust_ratio` from it, so the two stacks cannot disagree:
```yaml
quad4:                       # QUAD4 = /drone_3, Motive body 14, marker 24, /dev/QUAD4
  measured: '2026-10-14 09:30, Wesley; bags c7_*, tip_*'
  weld_point_z: 0.0xx        # marker above the ring-origin plane, face on plate 9 (mean of 3 re-seated)
  contact_band_m: 0.0xx      # max dz of the hand catches + 3 mm
  contact_radius_m: 0.0xx    # max dxy of the hand catches + 5 mm (-> weld_radius)
  body_to_marker_m: 0.5xx    # tether hanging (-> magnet_drop_below_quad); taut under ~2 N pull: 0.5xx
  proof_taut_len_m: 0.5xx    # body_to_marker - 0.003
  marker_to_face_m: -0.0xx   # face flat on the floor
  d_ok_noise_m: 0.00x        # drift spread, caught face rocking 0-20 deg
  kt_his: 2x.x               # Thursday R10a, his kT on QUAD4 (with the puck: 2x.x)
  z_error_hover_m: 0.0x      # his steady height error (with the puck: 0.0x)
```

## Critic

### Round 1 (rig port m1-rig dd33dca, 10 Oct night), as written

Verdict: Wednesday props-off GO WITH CHANGES (add the measurements; a combined props-off item needs Wesley's OK);
21-22 Oct flight NO-GO on dd33dca until: F5 merge m1-final + reviewer; F2, F3, F4, F9 in code; F7 with Wednesday's
numbers; F8 values typed from Wed/Thu; F1 load bars pass; F10 as the first flight; F6 built through the critic or
accepted in writing.

F1 High: his stream quiet >= 0.5 s -> elrs_interface (328-336) disarms QUAD4 mid-air; his MPC FAULT_LATCHED_HOLD on a
  reference > 0.5 s old (main.py 232-235, 1041-1069) hovers until DISARM; the full graph never ran at wall clock on one
  laptop. Wed props-off: full combined graph >= 10 min, bars frozen first: max gap < 0.15 s on /drone_3/ELRSCommand_tejen,
  /drone_3/ELRSCommand, each /drone_i/ELRSCommand_diss; < 0.3 s on /join_planner/reference; zero "External-reference
  fault latched". Fail -> no combined flight. New code (critic): mux takes QUAD4 back when his stream silent > 0.2 s and
  our tracker live; our planner takes it back when his MPC latches its reference fault. Sheet: "QUAD4 frozen > 5 s
  outside a hold phase: LAND".
F2 High: DDS isolation checked only in T2 (graph_attach 57-63). T3 / his command terminal / any ros2 topic pub on the
  default domain -> fleet disarm goes nowhere. Export both in mdc; fleet, preflight.py, rig_flight.sh refuse unless
  `ros2 topic echo --once /fleet/manager_status` answers within 2 s; repeat item 5 kill path from T3 and his terminal;
  sheet note: Tejen's laptop cannot see this graph.
F3 High: his TAKEOFF before fleet takeoff -> take-back cannot switch; LAND before TAKEOFF only service-disarms our
  trackers (fleet_manager 462-469); his MPC has no LAND (only ARM/DISARM/TAKEOFF). Mux clamps his throttle at idle
  (armed idle allowed) until our planner's phase is planner/network. Fix the ARM FAILED message (433-435): "his ARM first".
F4 High: elrs_interface listens on plain /drone_3/magnet (168); the RViz MAGNET D4 row publishes there (arm_panel.cpp
  239-241, 473-487); Wednesday's W4/W5 `ros2 topic pub -t 3 /drone_3/magnet OFF` from shell history drops QUAD4 / the
  puck. Remap drone 3's radio magnet input to a private topic only magnet_gate_3 writes; hide the D4 row; desk
  `ros2 topic info -v` one publisher; no /drone_3/magnet lines on a combined-day sheet.
F5 High blocker: rig tree lacks m1-final (lift from target, no pickup retry, 23f8873). Rig tree = m1-final merged with
  m1-rig; twin claim launch dumps identical; one m1c_hover + one m1c_nocatch_hover on the merged tree; reviewer passes the
  m1-final table first; rewrite the contract's rig lift paragraph (108-112).
F6 Medium-high: warm drone-3 tracker runs the pose watchdog and envelope while not flying (tracker 1545-1561): a fault
  latches, the mux holds his stream; his TAKEN_BACK ignores LAND, his MPC has no LAND -> hovers until DISARM. His MPC's
  own disarm never lands the fleet (fleet manager 314-322; /drone_3/fc_arm_state has the signal). Behind switches, card +
  critic (the tilt-cut card): his planner flies out 1.2 m and lands if the mux does not read ours 1 s after a take-back;
  warm tracker fault = not live, latches only with authority; FC disarmed after his takeoff -> fleet LAND. Until built,
  sheet line: "T2 says 'taken back, but our tracker is not flying yet': clear the cage, DISARM, QUAD4 falls" (Wesley
  accepts in writing).
F7 Medium-high: c0 = mean over the 0.15 s dwell opened at <= 0.05 m/s (contact_proof.py 167-190): at 0.05 m/s ~7 mm of
  travel, c0 ~5 mm above rest -> every proof times out; at 0.08 m/s 0.9 mm margin left; his end-of-descent overshoot.
  1 cm sphere vs 6 cm plate / his 2.5 cm gate / slack tilt. c0 from the last 0.05 s of the dwell + vertical speed
  <= 0.02 m/s (or re-anchor until the plate rises); desk ramps 0.03/0.05/0.08 m/s pass if c0 within 1 mm of rest;
  ring-frame vertical band + horizontal radius from Wednesday. Thresholds: Wesley.
F8 Medium-high: his IRL values are QUAD2's (body 12 / marker 22: irl_commissioning.yaml:28, m1_ground_ring.yaml:11):
  pickup values, marker-to-face -0.0225, kT 22 (twin 22 -> 18.1 still falling, R1476), drop 0.575 (twin 0.59);
  contact_ball_mass 0.1 vs puck 0.060 kg; object_mass_kg 0 + no z integral. kT and z error with/without the puck from
  Thursday R10a; body-to-tip and marker-to-face from Wednesday; refuse rig defaults for magnet_drop_below_quad and
  mpc_thrust_ratio unless from one QUAD4 record file shared with weld_point_z and proof_taut_len; contact_ball_mass 0.06.
F9 Medium: before our span every OFF of his reaches the coil; his LANDING sends OFF over the ring
  (mission_definitions 204-210); supervisor land_now leads there; DROP_OBJECT exits on its timer with the puck attached
  (online_join_planner 7010-7020); RViz ATTACH before the drop takes the coil with the puck. Gate forwards his OFF only
  in DROP_OBJECT / CLEAR_DROP_ZONE or before the puck is attached; gate switch and our span wait for
  /tejen/object_attached false; drop timeout with the puck attached = his give-up -> our take-back; supervisor stop
  publishes give-up, not land_now; refuse supervisor:=true on the combined rig until then.
F10 Process: first combined flight cannot catch: partner_args:='contact_dz:=0.03' (holds 3 cm above plate 9); exercises
  the mission, coil handover, 30 s give-up, take-back; measures his hold error over the moving plate and plate-9 noise
  with QUAD4 overhead. Then contact runs, hover first. A rig card with bars before the flight.
Low: magnet_gate restores its state from its latched topic on restart; take-back steps out with the tip on the plate
  (watch); two install spaces on Wednesday: run the combined item last.
Wednesday measurements: weld_point_z x3 re-seated; contact radius from >= 10 hand catches (centre, +-2 cm radial and
  tangential, tilted +-15 deg slack; band = max dz + 3 mm, radius = max dxy + 5 mm); speed gate / c0 hand ramps
  3/5/8 cm/s x3 (c0 within 1 mm of rest); proof_taut_len and drop (hanging, +-10 deg, ~2 N pull); d_ok (rocking
  0-20 deg); marker-to-face (face flat on the floor); coil 10/10 hold/release timed; mocap (puck on the magnet 24/6
  swaps, QUAD4 at 2.3-2.9 m on a pole, puck in the net, face on plate 9: gaps <= 0.1 s, no jumps > 1 cm); load and
  kill (full graph 10 min, F1 bars; DISARM from RViz and T3; kill his MPC alone; Ctrl-C T2: FC disarmed within 0.5 s);
  aux in Betaflight (AUX 5, 7-10 mid, no mode change).

### Round 2, part A (code on the candidate m1-comove 3a259fe / 986ac0a, 11 Oct evening), condensed
**Verdict:** GO WITH CHANGES for the hover flights R11a and R11b on 21-22 Oct.
- **Before R11a:** A1 (constants), A3 and A4 done, and A2's sheet line on the sheet.
- **The card's own gates still stand:** F1 (Wednesday load test), F5 (claim and reviewer on this SHA), F6 (written
  acceptance, or warm_fault_not_live through its reviewer) and F8 (values from the QUAD4 record).

**Round 1 state on this tree:**
- **F1 open:** the radio still disarms after 0.5 s of silence; the mux has no silence takeover.
- **Closed:** F2, F3, F4 and F9.
- **F5 partly:** the release guard has never flown in Gazebo. It only runs with real:=true, and TB6 flew on c53ae0c,
  before the guard merge.
- **F6 open:** one QUAD4 pose gap over 0.25 s latches our drone-3 tracker. A later take-back then hovers until
  DISARM.
- **F7:** the code is closed. The vertical-speed gate is a slope over 0.15 s, so its noise is ~0.004-0.008 m/s against
  the 0.02 limit. The values come from Wednesday.
- **F8 partly:** `mpc_cable_length` 0.531 and `anchor_offset_z` -0.030 still default to his values, unrefused.
- **F10:** closed in the card.

**Findings, worst first:**

| # | Severity | Finding | Fix | New flight-loop code? |
|---|---|---|---|---|
| A1 | high | The guard height uses `btt` = 0.59 (the carriers' rod) and the ring-origin plane. QUAD4's body-to-plate is 0.58, and R11b aims 1 cm low. The tip rises only ~5 cm against the 5 cm pass, with a 0.67 s carrot tail. A no-contact give-up with the tip on the plate then times out: a false RELEASE GUARD failure that the card scores as a stop. | For 21-22 Oct: rise +0.05 above slack (not +0.02) and `GUARD_WAIT_S` 3 s. The 2 cm plate-rise check still catches a held plate. Add a rig-geometry unit test. Later: the height from the measured tip0 + `proof_taut_len`. Split the falsifier: plate rising counts; a timeout with the plate still is reported. | constants: no; measured version: yes |
| A2 | high consequence | LAND with a coil that doesn't release. LAND starts the ring down at once. The guard's plate-rise test is in world z, so it never fires on a descending ring; only the 2.5 s timeout catches it. The ring tilts far past 5 deg. | Sheet line now: stop his attach with his give-up (`/join_planner/give_up`), LAND only as an emergency. Code later: hold the ring's LAND descent while the guard is armed (at most 2.5 s), or measure plate 9 relative to the ring. | yes (later) |
| A3 | medium-high | `weld_point_z` 0.0 reaches both stacks unrefused, and it is not a launch argument. `attach_cable_len` silently takes the carriers' `cable_len`. | On the combined rig, refuse `weld_point_z == 0` and an untyped `attach_cable_len`. Make `weld_point_z` a launch argument (or `quad4_record:=`). | no (launch) |
| A4 | medium | The guard reads `net.rho[3]`, the gap-centre placeholder. With the carriers at 270/30/150 the three gaps tie; argmax picks plate 9 today only because gaps 0 and 2 are equal to the last bit and argmax takes the first. Another order or offset could switch the guard off silently. | A pytest with the rig azimuths now. Then give the guard the weld point (the attach offset, as `_reserved_plate_dir` does). | test: no; fix: small yes |
| A5 | medium-low | The guard passes on a single tip sample. Its ring pose has no freshness check, and `plate_z0` is one sample. A tip-puck marker swap or a stale ring pose would give a false pass, then a full rise with the plate held. | Pass only after 0.1 s of consecutive fresh samples; ring pose within DETECT_STALE_S; tip-to-body within taut + 0.03; `plate_z0` averaged over 0.2 s. | yes |
| A6 | gate | F6 above: Wesley's written acceptance of the DISARM line, or warm_fault_not_live through its reviewer. | | |
| A7 | low-medium | His drop check trusts the release when the puck is unseen. The puck can ride to plate 9. | Wait up to N s for a fresh puck pose, then give up (his code, our fork). | his code, small |
| A8 | low | R11a's +3 cm is relative to his MPC (no z integral; R1503 flew 11 cm low). | If R10a's height error is over 0.02, use +0.05. Score §5.2 on the measured tip height. | no |
| A9 | low | `contact_rig_orbit` is honoured from a params file. | Honour it only when typed on the command line. | no (launch) |

**Checks:**
- **TB16:** no orbit reaches the rig launch without `contact_rig_orbit`, apart from A9.
- **Tilt cut with its switches off:** inert. No subscription, parameter or publisher is created. The only new default
  fleet action on the rig is the guard's LAND.
- **The §4 launch line:** missing `weld_point_z`, QUAD4's `attach_cable_len`, `mpc_cable_length` and
  `anchor_offset_z`.

### Round 2 follow-up: the hardening diff (m1-harden b8af32e + ac85597, 11 Oct evening), condensed
**Verdict:** GO WITH CHANGES; merge into m1-comove. Findings 1, 2 and 5 land before 21-22 Oct; 3 and 4 are tests
before Friday's claim. Each change does what A1/A3/A4/A5/A7/A9 asked, and nothing new cuts QUAD4. The twin is unaffected.
`drop_puck_wait_s:=0` restores his old path exactly. A give-up at the drop is safe: his magnet goes NONE, our span has
not begun, and our take-back flies the hover path.

| # | severity | finding | fix |
|---|---|---|---|
| 1 | medium | The tip-to-body band is centred on `attach_cable_len` + 0.04 (derived), never checked against the measured taut length (`proof_taut_len` + 0.003). A mismatch over ~2 cm makes every no-contact take-back with the tip on the plate time out and land the fleet. | Launch only: refuse when \|attach_cable_len - pivot_offset_z - (proof_taut_len + 0.003)\| > 0.01. Also check against `magnet_drop_below_quad` when it is in partner_args. |
| 2 | low-medium | Every timeout logs "tip has not left plate 9", whether the tip really held, the ring or the tip pose was stale, or the band missed. | Count ticks per cause and print them in the error line. |
| 3 | low-medium | The follow-down's slack is now the designed 5 cm (it was ~7 cm by accident). At LAND the reference can lag the ring's step by up to 0.2^2/(2*0.25) = 8 cm, so a held coil goes taut for a moment. The follow test only checks after 2.7 s. | Assert the reference stays at or below plate + taut over the whole follow, with the lag model. If that fails, ask Wesley for GUARD_SLACK_M ~0.08. |
| 4 | low | A held coil may lift plate 9 ~4.5-5 cm before the 2 cm check reacts (was ~3-3.5 cm): the reference now aims 0.10 m over taut. | A held-coil test that bounds the peak rise. Over 5 cm: slow the guard stage's speed, don't lower the height. |
| 5 | low | `tip0` is latched at the take-back, before the hold. | Reset it when the guard starts. |

**Nits:** GUARD_PASS_S and GUARD_PLATE_AVG_S are 2-3 planner ticks at 10 Hz. DETECT_STALE_S equals one planner
period. Twin params.json carries NaN for the weld point. weld_point_z refuses only 0, with no range check.
