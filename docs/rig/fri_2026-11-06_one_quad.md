# Fri 6 Nov: one-quad tests (QUAD4 alone)

Every terminal:
```bash
source ~/ros2_humble/install/setup.bash && cd ~/multi_drone_control && source install/setup.bash && export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=23
```

## Read first (after q4f / q4f_2 crashed, 17:18-17:21)
- **One stack at a time.** q4f and q4f_2 were both running: two controllers drove QUAD4's radio at once, and a
  `ros2 topic pub --once` reaches only the first subscriber it finds, so the TAKEOFF reached one controller and the
  DISARM the other: QUAD4 flipped 0.6 s after leaving the floor and its flying controller stayed ARMED on the floor
  (and when it was picked up) until the stack was stopped. Every script now refuses while any flight stack is up.
- **Fly from the RViz panel** (it opens with every script): **ARM**, **TAKEOFF**, **LAND**, and **SPACE = DISARM**
  (click the RViz window once first so it has the keyboard). A bridge started by each script turns the panel's commands
  into his topics; its DISARM goes to his controller 10 times over 1 s and puts the radio itself to disarmed for 2 s.
  Never use `ros2 topic pub --once /drone_command ...` again. Terminal fallback (it reaches the bridge, the only
  subscriber): `ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: DISARM}"` (or ARM / TAKEOFF / LAND).
- **LAND lands beside the magnet** (17:55): straight down until the magnet is on the floor (QUAD4 ~1.1 m up), then
  0.3 m sideways while it comes down: q4f away from the ring, q4p toward its takeoff mat. q4f and q4p now run his planner
  from the overlay copy ~/mdc_m2one_ws (that change and the one-quad M2 changes; src/ untouched).
- **His controller shuts down after any DISARM or landing.** Ctrl-C T1 and rerun the script before the next flight.
- Props-off magnet check: panel ARM, then in a terminal
  `ros2 topic pub -t 10 -r 10 /magnet/command std_msgs/msg/String "{data: ON}"` (q4m2: `/drone_0/magnet/command`); then
  SPACE, Ctrl-C T1, rerun.

## Funnel attach (q4f), first
QUAD4 alone onto plate 9 of the ring resting on the floor, funnel v4 on plate 9, Te-Jen's ground-ring trial in mission
floor_join: TAKEOFF straight up, level over plate 9 (magnet 0.30 m above it), match within 8 cm, descend to 3 cm below
the seated magnet (5 Nov's contact_dz -0.03), magnet ON in the descent, then ATTACH_READY holds with the magnet ON (no
automatic proof: judged by eye). LAND: magnet OFF, rise until the face is 0.16 m over the ring plane (clear of the
85 mm funnel), fly back over the takeoff spot, land there. Values: W 0.0491, L 1.034, Y 0.2293 (5 Nov; re-measured ones
go in as `W=.. L=.. Y=.. bash ...rig_q4f.sh`).

Setup: the ring flat on the floor; QUAD4's mat ~1 m out from plate 9 on the line from the ring centre through plate 9
(its path to the plate is straight, with no avoidance); Motive streams **8, 14, 24** only and is **not recording**; our
rig stack down (the script refuses otherwise).

```bash
python3 tools/mocap_stream_check.py --expect 8,14,24          # T1 first: must say OK, then stop it
bash results/rig/2026-11-06/scripts/rig_q4f.sh                # T1: q4f, q4f_2, ... (bag + log in results/rig/2026-11-06/)
# then the RViz panel: ARM, TAKEOFF; LAND after the catch; SPACE = DISARM. The phase shows on the panel.
```

**Expected phases:** TAKEOFF, APPROACH_ABOVE_TARGET, MATCH_VELOCITY, DESCEND_TO_ATTACHMENT, ATTACH_READY (holds). A
descent more than 10 cm off goes back up and retries on its own; LAND if it keeps hunting. **DISARM** on a drift toward
the cage (no position limit in his stack), a growing swing, or the magnet dragging the ring. Watch: does the magnet slide
down the cone onto the plate, does the cable clear the rim.

## Pickup (q4p)
QUAD4 alone picks up the small object (body 6) with Te-Jen's pickup stack: its pickup miss with no other quad flying,
to compare with 4-5 Nov's pickups (k1m, k1d, fm9y_2) made while the carriers flew. The same pickup settings as 5 Nov's
k1d (capture window 6 cm, proof bounds 5/8 cm, his throttle bound 0.8, the battery term), the 1 m cable (L 1.034), the
contact height 0.038. Run it after the funnel test, with that stack stopped (the script refuses otherwise).

Setup: the object on its mat in the open, at least ~1 m from the ring and from QUAD4's mat; Motive streams **6, 14, 24**
only and is **not recording**; body 6 as defined for k1d (k1o_2 failed on a re-defined body 6).

```bash
python3 tools/mocap_stream_check.py --expect 6,14,24          # T1 first: must say OK, then stop it
bash results/rig/2026-11-06/scripts/rig_q4p.sh                # T1: q4p, q4p_2, ... (bag + log in results/rig/2026-11-06/)
# then the RViz panel: ARM, TAKEOFF; LAND after the hold; SPACE = DISARM.
```

**Props off, before the first flight:** the magnet check in "Read first". Optional check of body 6 (props off, flight
stack down): `ros2 launch tejen_mission m1_irl_pickup.launch.py measure:=true quad_body_id:=14 magnet_body_id:=24
pickup_body_id:=6` in T1, the magnet face flat on the object's centre, `python3 tools/pickup_geometry.py` in T2: the
contact height should read ~0.038 (a new value goes in as `PC=<value> bash ...rig_q4p.sh`). Stop T1 after.

**Expected phases:** TAKEOFF, APPROACH_ABOVE_PICKUP (the magnet ~0.35 m over the object), a high then a low tier
(magnet ON in the low tier), MAGNET_ATTACH_WAIT, LIFT_OBJECT (his proof), then **the hold** with the magnet ON, the
object ~0.40 m up. A miss or a stalled descent retries on its own (up to 5); after that it holds with the magnet OFF.

**LAND** comes straight down until the object and magnet are on the floor, then QUAD4 moves 0.3 m toward its takeoff
mat while it lands, beside them. Keep that side clear.

**Stop rules:** DISARM on a growing swing, a drift toward the cage (his stack has no position limit), or the object
knocked off its mat; LAND if the lift sags without lifting the object. 3-5 flights, one per pack if the pack sags.
After each flight tell the lab chat its name: it scores every descent's miss from the bag against 4-5 Nov's.

## Attach with the 15 deg proof (q4m2)

Te-Jen's M2 real-world attach, flown by QUAD4 alone onto his plate 3 = **our plate 9** (funnel v4) of the ring lying on
the floor. After contact his proof moves QUAD4 **out along plate 9's radial until the cable leans 15 deg** with the magnet
held on the plate (body ~0.26 m out); that is the visual proof of the attach. It then holds there (attached hold) until
your LAND. His code runs from an overlay copy (`~/mdc_m2one_ws`, one-vehicle changes; `src/` untouched); config
`results/rig/2026-11-06/scripts/m2_one_quad4.yaml` (1 m cable, takeoff hover 1.33 m, plate 3). Critic waived (Wesley).
**Never flown in this form, and no simulation of it**: his M2 real-world path last flew 25-30 Sep on 0.55 m cables.

**Setup**
- The ring flat on the floor (Motive body 8), funnel v4 on plate 9, nothing else on the ring.
- QUAD4's mat about **1 m out from plate 9 on the line from the ring centre through plate 9**: the transit, the 15 deg
  hold (further out on that line) and the return all stay on that side, never over the ring.
- Motive streams only **8, 14, 24**, **not recording**. Our rig stack and the q4f / q4p stacks down (the script refuses).
- QUAD4 powered and bound before the launch: readiness needs its radio telemetry and link statistics.

**Commands** (every terminal: `source ~/ros2_humble/install/setup.bash && cd ~/multi_drone_control && source install/setup.bash && export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=23`)
```bash
python3 tools/mocap_stream_check.py --expect 8,14,24                      # T1 first: must say OK; then stop it
bash results/rig/2026-11-06/scripts/rig_q4m2.sh                           # T1: the flight (bag, log, his logs)
# then the RViz panel: ARM (once the phase is M2_WAIT_FOR_ARM), TAKEOFF; LAND (the bridge sends it to
# /m2d/operator/land_request in M2_ATTACHED_HOLD, else /drone_0/join_planner/land_now); SPACE = DISARM.
ros2 topic echo /m2d/status --field data                                  # T3 (optional): the supervisor state
```
- Wait for the phase **M2_WAIT_FOR_ARM** and `/m2d/status` state **ACTIVE_0** before ARM (his controller compiles ~10 s at
  launch; the assignment waits for QUAD4 still on the mat, disarmed, the ring and the radio's telemetry fresh). ARM is
  refused until then.
- **A DISARM ends his controller for that launch** (it shuts down): Ctrl-C T1 and relaunch before the next flight
  (also after the props-off check).

**Before the first flight, props off:** the magnet check in "Read first" (`/drone_0/magnet/command`).

**Phases** (T3): M2_WAIT_FOR_ARM -> (TAKEOFF) M2_VERTICAL_TAKEOFF -> M2_TAKEOFF_HOVER (body 1.33 m over the mat, waits
for the swing under 5 deg) -> M2_CPP_TRANSIT_CAPTURE (his C++ planner, level, to the magnet 0.30 m over plate 9) ->
M2_SETTLE_CAPTURE -> M2_ATTACH_APPROACH_HIGH -> M2_ATTACH_APPROACH_LOW (magnet ON near the plate) ->
M2_ATTACH_CAPTURE_WAIT (contact, 2 cm overtravel) -> **M2_ATTACH_PROOF** (~10 s: QUAD4 moves out until the cable leans
15 deg; his detector must see the magnet stay on the plate within 4 cm) -> **M2_ATTACHED_HOLD** (`/m2d/status`
M2D_SUCCESS_HOLD). A miss or a lost proof releases, rises and retries the same plate (3 tries, then it waits for you:
LAND).
LAND from the hold: magnet OFF, a slow rise (up to 0.20 m) until the magnet is seen off the plate, then back over the
mat at >= 1.33 m and down. If the detach is not seen within 1.5 s it goes to M2_FAULT and holds: send the land_now line.

**Stop rules**
- **DISARM** on a growing swing, a drift toward the cage (his stack has **no** +-1.75 m position limit), the magnet
  dragging the ring across the floor, or the proof pulling plate 9 up off the floor by more than a couple of cm.
- **LAND** if the takeoff hover never settles (the 1 m cable swings slowly; 25 Sep his M2 hover stalled with a ~1 Hz
  bounce) or the transit stalls (backend waiting) for more than ~30 s.
