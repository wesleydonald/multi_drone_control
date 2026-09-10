# Real-rig attach: what the hardware launch is missing (2026-09-10)

`tools/param_diff.py three_attach_launch.py real_dissipative_launch.py` — the sim attach
demo declares 69 arguments, the hardware dissipative launch 38. The hardware launch is a
carry/detach twin of `dissipative_launch.py`; it has **no attach machinery at all**. Before
the first rig attach a `real_attach_launch.py` (twin of `three_attach_launch.py`) is
needed. The gap, sorted by what it costs on the day:

## Must exist before the first rig attach (node falls back to a default that disables it)

| Argument (sim default) | Why the rig needs it |
|---|---|
| `reserved_attach` (1) | 0 on the real launch = no attach topics, no 4th-drone slot |
| `control_mode` (velocity_after_handover) | the working demo config; node default is `mpc` |
| `diss_ki_load` (1.0), `diss_a_i_load_max` (2.0) | common-mode trim; off (0) the 3-drone hover sags and the weld capsizes in SIL |
| `diss_balanced_tensions` (true) | moment-balanced shares for the 3/12/9 rim; default even split tilts |
| `attach_azimuths_deg` ('0,90,180' vs '') | the rig's 9/12/3 o'clock tethers; '' means an even ring |
| `attach_handout` (true), `attach_t_handout` (12) | soft hand-out; instant join collapses the newcomer |
| `attach_elev_deg` (65) | newcomer settle elevation; 45 collapses every time |
| `attach_traj_hold_s` (10), `handover_blend_s` (3) | trajectory hold through the reconfiguration; bumpless tension step |
| `attach_x_offset`/`attach_y_offset` (0, −0.25) | 6 o'clock rim target in the payload frame |
| `weld_radius` (0.08) | magnet trigger radius; must match the real magnet's capture range |
| `enable_approach`, `enable_approach_mpc` | the newcomer's approach controller and its kT (`approach_kt_*`) |
| `vel_kp_pos`/`vel_kv`/`vel_ki`/`vel_k_att` | velocity-mode tracker gains; hardware values unmeasured |

## Sim-only, must NOT be copied

`thrust_ratio`/`takeoff_thrust_ratio` `auto` (Gazebo secant gain; hardware keeps the
measured 24 / 0), `pose_timeout_s` 1.0 and `safety_ref_timeout_s` 2.0 (sim CPU-load
budgets; hardware watchdogs stay at 0.25 / 1.0 s), `sil`, `enable_obstacle_avoidance`.

## Same argument, different default — decide per flight

`load_mass` (0.6 vs 0.1: weigh the ring), `num_drones` (3 vs 2), `handover_elev_deg`
(45 vs 0), `lift_ramp_vel`, `traj_speed` (0.6 vs 0.4: the attach demo flies 0.2),
`payload_rest_z` (−0.1 vs 0.05: the rig's stand height).

## Not a launch argument, but blocks the campaign

- Mocap rigid bodies for the 4th drone AND the magnet tip (`attach_target_publisher`
  reads the tip pose; the weld capture is tip-based).
- The magnet manager's ELRS aux channel (`elrs_magnet_on/off_value`) on the newcomer's
  radio, and a `detach_when_magnet_off` release path (the round trip depends on it).
- `tools/preflight.py --real` reads back every item in the first table from the live
  nodes, so a launch that silently lost one fails the flight card, not the flight.
