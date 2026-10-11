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
| F6 | warm-tracker faults and his stuck TAKEN_BACK handled (the tilt-cut card), or accepted in writing with the sheet line "T2 says 'taken back, but our tracker is not flying yet': clear the cage, DISARM, QUAD4 falls" | another helper |
| F1-new | new authority, NOT built (needs a card + critic): the mux takes QUAD4 back when his stream is silent > 0.2 s while our tracker is live; our planner takes it back when his MPC latches its reference fault | listed only |

## 3. The flights, in order (one variable each)
1. **R11a no-catch rehearsal (F10)**: the whole mission with `partner_args:='contact_dz:=0.03 ...'`: his descent holds the
   marker 3 cm above plate 9, so no catch is possible. It exercises the pickup, transit, drop, the coil hand-over at
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
   partner_args:='magnet_drop_below_quad:=<L> mpc_thrust_ratio:=<kT> [contact_dz:=0.03]' r11a
```
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
- a RELEASE GUARD failure. The fleet lands, with QUAD4 following the ring down at the tether's slack.

**Reported, no bar:** the overflight time and the minimum tip-to-carrier gap. Rotor wash is a rig-only question.

### 5.7 Authority (every flight)
- Zero `/drone_3/magnet_coil` writers but magnet_gate_3; zero his OFFs reaching the coil in the transit or over the
  ring once the puck is taken (`[magnet_gate] his OFF held` lines are fine); zero "HOLDING approach authority" with QUAD4 airborne.

## 6. Stop rules (the sheet's, plus)
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
