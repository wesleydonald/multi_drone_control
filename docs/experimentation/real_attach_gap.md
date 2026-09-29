> **SUPERSEDED (2026-09-29).** `real_attach_launch.py` now refuses to start (its newcomer took ARM/TAKEOFF from the
> /fleet/command broadcast). The rig flies `three_attach_launch.py real:=true` (Wesley 2026-09-28); the current rig sheet
> is the WED section of tests.txt and the ladder is docs/g7_real_world_ladder.md. Kept for the history below.

# Real-rig attach: the hardware launch and what the rig must still supply (2026-09-10)

`real_attach_launch.py` is the hardware twin of `three_attach_launch.py` (same 69
arguments; `tools/param_diff.py three_attach_launch.py real_attach_launch.py` shows only
the intended differences: measured kT 24 / takeoff 0, real-rig takeoff pace, the sim-only
watchdog budgets and `sil`/`attach_pose_index` absent, the magnet radio args added). The measured-force throttle (`vel_indi_*`, branch
`measured-force-velocity-loop`) is sim-only until its IMU filter is checked on the airframe.
Brought up whole on a desk on 2026-09-10 against `tools/fake_mocap.py --num-drones 3
--attach` (static poses): all four trackers at kT 24, the dissipative node, mux, approach
MPC, join planner, target publisher and magnet manager come up, and the mux forwards the
newcomer's radio command at 30 Hz. Without any mocap only ONE tracker appears -- each
holds the acados compile lock until its first pose -- so bring terminal 1 up first, as
the run order already says. Before it existed the hardware dissipative launch lacked all 31 attach
arguments and would have flown the demo config with every one of them at its node
default.

## What the rig has to provide (the sim gets these for free)

| Item | Where | Status |
|---|---|---|
| Newcomer rigid body | `RIGID_BODY_TO_DRONE` in `motion_capture_publisher_node.py` (id = `num_drones`) | add the body id |
| Magnet-tip rigid body | `MAGNET_TIP_RIGID_BODY_ID` (same file) → `/magnet_tip_pose` PoseStamped | None until set; the weld trigger and tip-based weld capture read the TIP |
| Magnet aux channel | `elrs_magnet_channel` / on / off values; `elrs_mux` merges the channel into every forwarded command | channel index and polarity unverified against the receiver |
| Weld detection | the manager declares `/magnet/object_attached` on tip distance < `weld_radius` and low speed; it cannot feel a real weld | `weld_radius` must sit inside the magnet's real capture range |
| Rim geometry | `attach_azimuths_deg` (tethers), `attach_x/y_offset` (4th magnet), `cable_len`, `load_mass` | measure, then `tools/preflight.py --real` reads them back |
| Velocity-mode gains | `vel_kp_pos` 2 / `vel_kv` 4 / `vel_k_att` 8 | sim values; unmeasured on the airframe |
| Approach controller kT | `mpc_thrust_ratio` = the fleet's 24 | solo hover is a lighter operating point; fixed by decision |

## Order of proof at the rig

1. `real_dissipative_launch.py` hover and circle with three drones (nothing new).
2. `real_attach_launch.py enable_approach:=false`: same flight through the new launch.
3. Newcomer hovering solo on its approach MPC, magnet OFF, `weld_radius` small: watch
   `/magnet_tip_pose` and `/attach_target/pose` agree in RViz (role letters T T T W).
4. ATTACH with the fleet on a 0.2 m/s circle; the banner shows HOLD, then N for the
   newcomer; `attach_traj_hold_s` gives 10 s before the circle resumes.
5. DETACH 3 (magnet OFF through the manager) once the four-drone circle is steady.

## Rig checklist additions (2026-09-24, from the 2026-09-23 non-lift)

| Item | What | Why |
|---|---|---|
| Payload rigid body in Motive | origin at the RING CENTRE on the attach plane (plate tops) | it read 4.3 cm off on 2026-09-23: the per-rod estimate was 6 cm out and `measure_rod_len` refused it; the rest references and tilt read wrong |
| Packs | 6S, fresh, ≥ 24 V at the preflight (bar 22.8 V = 3.8 V/cell); all voltage constants are 6S since 2026-09-24 | drone 1 started at 21.2 V and sagged to 17 V; 4S constants never warned |
| Rod length | type `cable_len:=0.47` (disc fit); `measure_rod_len` (default on) refines it at the creep handover | typed 0.50 vs real 0.46 alone cost 22 cm of hover in SIL |
| Floor start | `start_taut:=false handover_elev_deg:=45 handover_settle_s:=2.0` on every rig launch | the OCP references from tick 1 sit 18–22 cm inward of a resting drone |
| Preflight | `tools/preflight.py --drones N --real --planner load_planner --max-ground-z 0.5` | rigid rods rest the drones at ~0.38 m |
| Adapters | `drone0_serial:=/dev/QUAD1 … drone3_serial:=/dev/QUAD4` explicitly (udev names 1–4, launch default 0–3), `num_drones` in both terminals | a default QUAD0 does not exist |
