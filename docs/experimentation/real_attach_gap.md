# Real-rig attach: the hardware launch and what the rig must still supply (2026-09-10)

`real_attach_launch.py` is the hardware twin of `three_attach_launch.py` (same 69
arguments; `tools/param_diff.py three_attach_launch.py real_attach_launch.py` shows only
the intended differences: measured kT 24 / takeoff 0, real-rig takeoff pace, the sim-only
watchdog budgets and `sil`/`attach_pose_index` absent, the magnet radio args added).
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
