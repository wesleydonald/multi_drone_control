# Targeted upstream notes — not a patch bundle

Reviewed source HEAD: `73e7b7fed86a584c7e6642dfc473c0fe9fe1a8a6`.
Receiving fork's last documented upstream sync: `fd495db0`.
These notes do not assert that every subsequent commit should be ported.

## Existing completion-collection fix

Commit: `1cfc7a90e8e86fe3a20dcb00403e7d986ccabecc`

Subject: `planner: collect completed ROS plans before splice deadlines [NAV2]`

Relevant production/test paths:

| Upstream | Receiving fork |
|---|---|
| src/dynamic_planner_ros/include/dynamic_planner_ros/planner_completion_waitable.hpp | src/tejen_dynamic_planner/include/tejen_dynamic_planner/planner_completion_waitable.hpp |
| src/dynamic_planner_ros/src/transfer_backend_node.cpp | src/tejen_dynamic_planner/src/transfer_backend_node.cpp |
| src/dynamic_planner_ros/CMakeLists.txt | src/tejen_dynamic_planner/CMakeLists.txt |
| src/dynamic_planner_ros/test/test_planner_completion.cpp | src/tejen_dynamic_planner/test/test_planner_completion.cpp |

Map package/include namespaces consistently with your fork; the table describes
the intended mapping, not a verified current checkout of your local headers.

The fix wakes the ROS executor when a completed worker result becomes available,
instead of relying only on the ordinary collection timer. It makes the result
available before notification. Installation remains executor-owned; existing
version, epoch, authority and deadline checks are retained. Existing timers remain.

Additional diagnostics include completion_to_collection_ms,
result_collection_time_s, installation_attempt_time_s,
installation_splice_margin_ms and installation_world_version.

Inspect the commit diff against your renamed baseline, preserve local changes,
and run its completion tests and your handoff/manual-authority regressions before
any trial. Do not replace your current backend wholesale or assume this alone
fixes every install race. No executable patch is included in this package.

## Current-source checks supporting the reply

- `src/drone_communication/drone_communication/online_join_planner.py`:
  DESCEND_TO_ATTACHMENT still has an immediate generic relative-alignment
  reapproach. Ground-ring tiered HIGH has a separate hold-and-correct exception;
  do not silently transplant that static-target policy to your moving ring.
- `src/drone_communication/config/m1_measured_tip_pickup.yaml`:
  capture XY0.025m, dwell0.5s, relative speed0.07m/s, LOW XY0.025m and HIGH0.05m.
  Your 0.10/0.12m LOW/HIGH overrides are not the same profile.
- `src/dynamic_planner_ros/config/c1f_irl_m1_virtual_ring.yaml`:
  cooperative_tracking_safety_enabled=false and
  execution_tracking_safety_enabled=false. These are separate enforcement flags;
  turning them off does not make advertised futures physically exact.
- `src/dynamic_planner_ros/src/transfer_backend_node.cpp`:
  cooperativeSharedBlocker retains start/coverage/freshness checks;
  tryActivate waits below minimum_search_z_m. No automatic recovery fix for
  those behaviours is supplied here.
- `src/controller_mpc_payload/controller_mpc_payload/dynamics.py`:
  payload_dynamics uses the linear straight-cable pendulum equations
  phi_ddot=-(g/L)*phi-a_x/L and theta_ddot=-(g/L)*theta-a_y/L. A stiff bent cable
  is outside that shape model; current tip position still matters geometrically.
- `src/controller_mpc_payload/controller_mpc_payload/command_execution.py`:
  command_at_elapsed_time interpolates the command states over elapsed time,
  capped at the first-stage duration. Your oct10/oct14 fork selection must be
  checked in its own main.py; we have not inspected that source or its new logs.

No benchmark workspace, flight settings, gains, collision margins or gates have
been modified for this review package. Reported rig outcomes remain
**Handoff-reported** until the requested artifacts are independently inspected.
