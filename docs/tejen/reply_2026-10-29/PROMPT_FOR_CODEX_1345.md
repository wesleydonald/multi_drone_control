# For Codex: decisions on the 29 Oct rig follow-ups (Tejen's controller in Wesley's rig)

You reviewed Wesley's rig report and evidence earlier today (your `REPLY_TO_WESLEY.md`, `EVIDENCE_REQUEST.md`,
`UPSTREAM_NOTES.md` and the follow-up on timing). This asks for your decisions on what comes next, item by item. Use the
evidence you already have: `rig-evidence-20261029.zip` (sq5_2, k1_4, sq2_5) and `cable-geometry-20261029.md` (below as
item 4's data). Keep to your own suggested order: the timing correction first, then the swing, then narrow descent and
capture changes. No wholesale merge, no broad threshold relaxation.

Names: Wesley's fork renames your packages. `src/tejen_mpc` = your MPC (`controller_mpc_payload` upstream),
`src/tejen_mission` = your mission/planner (`drone_communication`), `src/tejen_dynamic_planner` = your C++ backend
(`dynamic_planner_ros`). `partner_mission.launch.py` and `ring_bridge.py` are Wesley's (his launch and his ring bridge
subclassing your `fake_cooperative_transport_world.py`).

Setup flown on 29 Oct: three carriers (Wesley's controller) in a triangle on plates 3/11/7 of a 1.15 kg ring (0.51 m,
~100 x 85 mm slots); QUAD4 (your controller) on a 1 m metal cable (bends, no stretch once taut) joins plate 9; mocap
markers on body and magnet; rig wall clock; `mpc_profile` oct10 on every run.

## 1. The timing port: review it (queued to fly next, after a twin check)
What you recommended: the elapsed-interval command correction under oct10, without the oct14 weights or bounds. In the
fork the send path was tied to the profile (`main.py`: `self.mpc_partial_step_execution = self.mpc_profile == 'oct14'`).
Wesley's side added a parameter that can override it (default `profile` = unchanged):

```diff
@@ main.py, after the profile log line
+        command_execution = str(self.declare_parameter('mpc_command_execution', 'profile').value).strip()
+        if command_execution not in ('profile', 'partial_step', 'endpoint'):
+            self.get_logger().fatal(
+                f'Unknown mpc_command_execution {command_execution!r}; expected profile, partial_step or endpoint')
+            raise ValueError(f'mpc_command_execution {command_execution!r}')
+        if command_execution != 'profile':
+            self.mpc_partial_step_execution = command_execution == 'partial_step'
+            self.get_logger().warn(
+                f'MPC command execution {command_execution} (mpc_command_execution), '
+                f'the {self.mpc_profile} weights and bounds unchanged')
```
It runs before `generate_payload_ocp_controller`, so `mpc_command_stage_duration_s = time_steps[0]` and the send-path
block (`command_at_elapsed_time(control_state, u, elapsed_s, self.mpc_command_stage_duration_s)`, the history storing
the rounded command sent, `command_execution_last_publish_time` from the node clock) follow the flag as under oct14. The
metadata's `command_execution` then reads `first_stage_partial_step`.
**Question:** is `partial_step` under the oct10 OCP the correction you meant? Anything in that path that assumes the
oct14 OCP (stage duration, input-rate bounds, state layout)? Anything to check in the twin before the rig?

## 2. Descent reapproach (after the timing test)
Contact mode, rig values: `contact_abort_xy_threshold` 0.08 m (= max(match_xy 0.08, 0.08)),
`contact_abort_xy_velocity_threshold` 0.25 m/s, checked every 30 Hz tick, no dwell:
```python
too_far = (xy_error > self.contact_abort_xy_threshold
           or xy_relative_velocity > self.contact_abort_xy_velocity_threshold)
```
`tracking_errors()`: `xy_error` = measured magnet tip (`use_measured_magnet_tip` true) minus tip target;
`relative_velocity` = body velocity minus target velocity (target velocity zero with attach_twist zero).
sq5_2: 6 descents, 6 aborts, every one on position (8.1-9.5 cm at the abort), after 0.0-2.7 s of descent.
**Questions:** a sustained-loss dwell (how long; reset rule; fresh-evidence rule)? Should the speed term use
magnet-to-plate speed instead of body speed? What stays distinct as final contact/proof evidence? A narrow patch against
your upstream paths if you'd do it.

## 3. Pickup capture and MATCH (after the timing test)
k1_4 flew your tiered pickup with: capture 0.025 m / 0.5 s / 0.07 m/s (yours), LOW 0.10 m with a 1.0 s dwell and HIGH
0.12 m (Wesley's 14 Oct rig values), `pickup_lift_xy_dwell_s` 1.5, `pickup_lift_xy_tolerance` 0.04, overtravel 0.036.
26 s in SETTLE_ABOVE_PICKUP: magnet-object xy median 4.07 cm (mean (-1.39, +0.31) cm, oscillation 5.09 cm rms), inside
2.5 cm 16.8 % of samples. Typed for the next try, not flown: capture 0.06 m, relative speed 0.15 m/s. The whole flown
profile is in `runs/k1_4/launch_resolved_params_his_nodes.yaml`.
MATCH, sq2_5: 5 cm / 1 s unbroken took 12.3 s and then never held; sq5_2 with 8 cm / 0.5 s / zero twist passed in 1.7 s.
**Questions:** safe capture values (as an above-object capture trial, not contact limits), reviewed with LOW, HIGH and
proof together; your view on MATCH 8 cm / 0.5 s; anything you would change in how the LOW tier or the bottom dwell
treat a swinging magnet.

## 4. The swing (after the timing test): data
Anchor-to-magnet geometry from the bags, your IRL pendulum math (anchor = body + R(q) (0, 0, -0.030),
phi = atan2(r_x, -r_z), theta = atan2(r_y, -r_z)), per phase window; your MPC's logged phi/theta over the same windows:
| run, window | chord (median) | mean phi / theta | rms phi / theta | dominant | your MPC (mean; rms) |
|---|---|---|---|---|---|
| sq5_2 approach (6.8 s) | 1.009 m | -0.42 / +0.14 deg | 1.88 / 1.85 | 0.73 Hz | -0.42 / +0.14; 1.88 / 1.89 |
| sq5_2 MATCH (1.7 s) | 1.009 | -0.53 / -1.15 | 2.56 / 2.36 | 0.59 | -0.47 / -1.27; 2.61 / 2.26 |
| sq5_2 descent (1.8 s) | 1.009 | -0.54 / +0.25 | 2.00 / 0.66 | 0.56 | -0.56 / +0.16; 2.01 / 0.70 |
| sq5_2 approach (3.2 s) | 1.009 | -0.74 / +0.02 | 5.07 / 2.45 | 0.63 | -0.86 / +0.11; 5.03 / 2.41 |
| k1_4 settle (26.3 s) | 1.008 | -0.64 / +0.36 | 1.75 / 1.33 | 0.59-0.70 | -0.64 / +0.35; 1.76 / 1.35 |
Props-off taut chord ~1.011 m. Reading: straight and taut in flight, small mean lean, a 1.6-5 deg rms swing at 0.5-0.9
Hz, and the MPC received the measured swing. Limit: a kink at the anchor shows only as the mean angle (bounded at ~2 deg).
**Questions:** are your swing states and cost active under oct10 on the rig (which parameters, which weights), and is
damping this swing in reach once the timing is corrected? Which logged channels should the next flight be judged on?

## 5. C++ transit: decisions
- **Hand-over bounds.** Rig now 0.12 m/s and 0.30 m drift (your defaults 0.08 / 0.15); QUAD4 hovered at 0.08-0.09 m/s
  (sq3). You accepted the speed and called the drift a separate concession. What drift?
- **Carrier tether model.** For a low transit end (tip 0.30 m above plate 9) your box model of the carriers' tethers
  refused the end at its margins (twin, the T layout: separation -0.137 m at swing 0.17 rad and inflation 0.05 + 0.05;
  accepted only near 0.02 rad); in the triangle the neighbouring cables are 60 deg either side of plate 9. Which
  component relation would you change (capsule tethers, the swing allowance, the end height), if any?
- **The start_skew race.** Your `fake_cooperative_transport_world.timer_callback` refreshes the shared commitments
  before it publishes that tick's carrier states; your backend refuses a commitment starting after a carrier's latest
  state (tolerance 1e-6 s) and latches off. Wesley's workaround, typed on the rig (`commit_backdate:=0.04`):
  ```python
  def refresh_shared_commitments(self, valid_from_s):            # RingBridge (Wesley's subclass)
      back = max(0.0, float(self._param('shared_trajectory_backdate_s', 0.0)))
      FakeCooperativeTransportWorld.refresh_shared_commitments(self, valid_from_s - back)
      self.last_shared_commitment_publish_s = float(valid_from_s)
  ```
  The spline's first control point is the measured ring (the bridge's `compute_payload_state` returns the measured ring
  for knots within 0.05 s of now, the planned trajectory beyond). Is 40 ms of backdating acceptable as a description of
  the actual trajectory, or would you rather reorder your node (states before commitments)?
- **Search floor.** Your backend waits below `minimum_search_z_m` for ever (sq4_5: QUAD4 at 2.33 m under a 2.36 m floor,
  21 s). Wesley's launch now refuses a typed floor within 0.10 m of the climb end. Anything on your side?

## 6. Small
QUAD4's `object_vehicle_mass_kg` is 0.55; it weighs 0.583 kg with the 1 m cable. Change it or not, after tracing how it
enters your model?

## What to send back
One `.md` with a section per item above: your decision or recommendation, what evidence supports it, and what would
need a flight to settle. For any code change, a narrow unified diff against your upstream paths with the fork mapping
noted, and the tests you'd run. Mark anything provisional.
