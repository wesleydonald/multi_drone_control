# tejen_dynamic_planner - R6.3C.1b/C.1c

This increment is **passive commissioning only**. It connects the validated C++
planner to live `/motion_capture_state`, runs suspended-mode 9^3 Octopus shadow
plans at a nominal 10 Hz request cadence, and publishes a 30 Hz **shadow**
`MultiDOFJointTrajectory` to `/dynamic_planner/shadow_reference`.

It deliberately refuses to use `/join_planner/reference`, so it cannot take MPC
authority in C.1b/C.1c.

Key contracts:

- 30 Hz reference sampling, 20 MPC stages, skip_steps=3, 61 samples / 2.0 s.
- 10 Hz planner request cadence, at most one solve in flight; busy triggers coalesce.
- ROS time is trajectory/world time. `steady_clock` remains runtime benchmarking time.
- measured acceleration uses the already-approved first-order derivative filter,
  tau = 0.15 s by default. The derivative/freshness epoch uses this node's ROS-time
  receive stamp so a producer on a different clock cannot make fresh state appear stale;
  the producer header offset is still logged when clock domains are comparable.
- planning is gated to a stabilized hover (defaults 0.10 m/s speed and 0.50 m/s^2
  filtered acceleration).
- each passive solve creates a **fresh stationary hover incumbent** from the latest
  measured state. A previous shadow candidate is never used as the next planning
  start, so passive mode cannot advance a fictional executed incumbent.
- the most recent observed TRAJECTORY-clock elapsed time is carried only as the
  next fresh planner's future-splice lookahead seed. CPU runtime remains a separate
  steady-clock diagnostic, so Gazebo real-time factor does not distort splice lead.
- an optional artificial delay is inserted inside the planning-world snapshot for
  C.1c. If the first candidate is late it is rejected and the stationary hover
  incumbent remains unchanged; the trajectory-time-seeded next fresh solve can then use a
  longer future splice.
- a self-subscription to the shadow reference measures ROS/DDS/executor message age.
- diagnostics report ROS-time reference period, wall-time reference period, message
  age, planner/Octopus/qpOASES runtime, coalesced triggers, late candidates, and the
  first accepted future-C2-splice check.

No MPC controller code, MPC cost/dynamics, Octopus algorithm, MINVO, qpOASES
refinement, collision geometry, planning radius, or B.4 commissioning limits are
changed here.

## C.1b/C.1c v3 stationary/disarmed commissioning

The first airborne passive run showed that flying the vehicle was unnecessary for
this integration gate and introduced avoidable controller/reference risk. v3 therefore
runs C.1b/C.1c with the vehicle stationary and disarmed.

Key defaults:

- `use_sim_time: false`, matching the inspected `commandsForPayloadSimClean*` stack,
  which does not establish a `/clock` bridge.
- `planner_rate_hz: 5.0`, based on measured accepted suspended-system ROS runtimes
  around 118-169 ms in the first C.1b run.
- `stationary_shadow_state: true`, so the measured position is retained but the
  private shadow seed uses zero velocity/acceleration after the stationary gate.
- `minimum_search_z_m: -0.05`, allowing a passive ground/spawn commissioning state
  instead of inheriting the old airborne-only 0.20 m lower-z clamp.

The node remains PASSIVE ONLY and still rejects `/join_planner/reference` as its
shadow output topic. No MPC/controller source is modified here.

Before C.1d active authority, inspect the actual current controller/MPC and command
workflow again rather than relying only on this README or a handoff summary.

## C.1d active clear-space commissioning (v0.4.0)

C.1d adds a separate `dynamic_planner_active_commissioning` executable. The
validated passive C.1b/C.1c executable is intentionally left intact.

C.1d is the first bridge allowed to publish `/join_planner/reference`, and it must
be the only publisher on that topic during commissioning. The active bridge keeps
one persistent `RecedingHorizonPlanner`; the planner object itself is moved into the
single asynchronous solve worker and returned when the solve finishes, while the
30 Hz reference timer samples a separate immutable copy of the current committed
trajectory. This avoids blocking the reference timer on the ~130 ms suspended
Octopus solve and avoids unsynchronised concurrent access to planner state.

Authority phases:

1. `GROUND`: measured-position 61-sample stationary reference. C.1d0 stops here.
2. `ASCENT_C3`: 4 s seventh-order rest-to-rest vertical reference to z=1.20 m.
3. `HOVER_SETTLE`: stationary target hold until vehicle + pendulum conditions are
   satisfied continuously for 1.0 s.
4. `ACTIVE_PLANNING`: persistent 5 Hz suspended 9^3 planner, initially targeting
   +1.0 m X at constant settled-hover altitude.
5. `FAULT_HOLD`: local controlled-stop/hold containment if this reference bridge
   itself loses a valid authority representation. This is explicitly not claimed
   to be an obstacle-safety certificate.

The seventh-order normalized step is

`s(tau) = 35*tau^4 - 84*tau^5 + 70*tau^6 - 20*tau^7`,

which has zero endpoint velocity, acceleration, and jerk. The persistent planner is
seeded with the same certified stationary hover used by the validated core. Its
initial splice timing seed defaults to 160 ms based on the measured C.1b normal-run
maximum (~153 ms), avoiding the old first-attempt 83 ms bootstrap lookahead.

C.1d writes a 1 Hz bridge diagnostics CSV by default to `/tmp/r6_3c1d_active.csv` so authority/phase/timing history is preserved for post-run analysis.

C.1d does not change standalone planner math, MINVO, Octopus, qpOASES, planning
radius, B.4 suspended geometry, or Check/Recheck semantics.

## C.1e static suspended-obstacle commissioning (v0.5.0)

C.1e keeps the passed C.1d authority, timing, MPC interface, persistent-planner,
and fault-hold behavior. It selects a new configuration that adds one physical
yaw-rotated cuboid and raises only the commissioning collision envelope from 10
to 15 degrees. The 5-degree hover-settle gate is unchanged.

The canonical scene is `config/c1e_static_sim.yaml`. Both the C++ node and
`scripts/generate_c1e_world.py` consume its obstacle centre, half extents and
yaw. The generated `simulation_assets/tejen/world_drone_env_detach_c1e.sdf` therefore
uses the same geometry as the planner rather than an independently entered box.

C.1e sets `spline_time_factor: 3.0` while C.1d remains explicitly at its
validated `2.5`. The C.1e solver diagnostic showed that 2.5 produced a qpOASES
infeasibility for the suspended-obstacle route, while 3.0 solved the same case
without relaxing jerk limits, separator margin, or qpOASES working-set limits.

Before TAKEOFF is accepted and again before `ACTIVE_PLANNING`, C.1e requires:

1. the complete suspended assembly is safe at the intended hover start;
2. the complete suspended assembly is safe at the goal;
3. the body-only straight sweep is safe; and
4. the same straight sweep is blocked by cable or magnet geometry.

The C++ planner publishes `/dynamic_planner/markers` with the physical obstacle,
component C-space vertices, direct witness, authoritative reference, committed
route, Octopus and selected control polygons, splice/local/global goals, measured
trace and status text. The current RViz config contains a transient-local display
for this topic while retaining the old `/join_planner/markers` display.

C.1e records 1 Hz commissioning diagnostics in `/tmp/r6_3c1e_active.csv` and a
30 Hz sample-zero measured-versus-reference trace in
`/tmp/r6_3c1e_reference_trace.csv`. Swing above 10 degrees is latched as a warning;
swing above 15 degrees latches `geometric_claim_invalid` for the run but does not
introduce an unapproved autonomous flight response.
