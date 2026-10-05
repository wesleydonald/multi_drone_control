# C1F.6 Supervised Simulation Test

The normal research loop is user-operated. Codex prepares a change and its focused
tests; the user runs the GUI simulation, reports what they observed, and Codex analyses
the newest evidence bundle together with that feedback.

From the repository root, run:

```bash
tools/sim_test/run_c1f6_manual.sh
```

The command performs preflight checks, starts Gazebo and RViz, and waits for manual
arming and takeoff. After `TAKEOFF`, it stops at either a stable two-second
`ATTACH_READY`/`handoff_ready` success condition or the 120-second mission timeout. It
then requests disarm, shuts down only the processes it launched, copies the existing
CSV logs, and writes an analysis bundle under:

```text
logs/c1f_experiments/c1f6_manual_<timestamp>/
```

Press Ctrl-C to abort safely. The runner still collects and analyses the available
artifacts. When asking Codex to diagnose a run, include what you saw or heard in Gazebo
and RViz; Codex can locate the newest bundle automatically.

The normal handoff does not require sending the raw CSVs. After cleanup the command
prints paths to:

- `analysis/summary.md`, containing the verdict, first abnormal event, phase durations,
  data quality, and essential subsystem metrics;
- `analysis/dashboard.png`, a six-panel whole-mission overview; and
- `analysis/key_events.csv`, a TAKEOFF-relative cross-system event timeline.

Tell Codex the behavior you expected, the first abnormal phase or approximate time,
what you observed, and any manual intervention or unusual Gazebo/RViz indication.
Codex will inspect those three compact artifacts first.

## M1 environment stress cases

The same supervised M1/C1F.6 mission can be run against checked-in environment cases.
Cases change only fake-target motion, fake cooperative-drone layout, and static obstacles;
they cannot change mission gates, planner tuning, safety tubes, or controller settings.

List the registered cases:

```bash
tools/sim_test/run_m1_case.sh --list
```

Run one case:

```bash
tools/sim_test/run_m1_case.sh static_two
tools/sim_test/run_m1_case.sh blocking_center
tools/sim_test/run_m1_case.sh narrow_gap
tools/sim_test/run_m1_case.sh figure8
tools/sim_test/run_m1_case.sh three_attached
tools/sim_test/run_m1_case.sh three_attached_yawing
```

Registered case files live under `tools/sim_test/cases/v1/`. The case YAML is copied
into every evidence bundle as `configuration/case.yaml`, and non-baseline bundle names
include the case, e.g. `c1f6_static_two_manual_<timestamp>/`.

`tools/sim_test/run_c1f6_manual.sh` remains the baseline alias. Its behavior is
equivalent to selecting the `baseline` case.

Current M1 stress semantics:

- `blocking_center` is an intentionally infeasible negative case.
- `narrow_gap` uses explicit companion offsets of +/-0.47 m and is intended to be a
  tight positive case.
- `mixed` is a positive combined case. Its close companion uses +0.45 m lateral
  offset; the older `blocking_left` +0.25 m placement made the final suspended
  rendezvous structurally infeasible.
- `three_attached_yawing` is the pre-M2 rigid-transform diagnostic built as an A/B delta from
  `three_attached`: the basket keeps the same circular translation but adds a 0.20 rad/s yaw.
  The three companion bodies stay fixed in the ring frame, so they orbit the basket centre at
  the same angular velocity while translating with it. Visualization intentionally stays the
  same as the ordinary scenarios. The case does not enable diagonal cooperative cable collision
  geometry and is not an M1 pass criterion.
- `three_attached` is an M2-preview environment with four drones total: ego at plate 0
  plus three fake companions associated with plates 3, 6, and 9. Companion positions are
  derived from `RingNetGeometry` and an ideal 0.50 m straight cable pulled radially
  outward at 45 deg from vertical. This gives about 0.354 m outward and 0.354 m upward
  cable components, placing each companion about 0.604 m radially from ring centre.
  The case YAML stores the cable length/angle, not hand-written XYZ offsets. The C++
  backend is provisioned with three cooperative state/trajectory topic pairs for this
  case only; ordinary M1 cases remain at two. This remains body-only preview collision
  geometry, not the final diagonal cooperative-cable model. Because the current M1 C++
  delivery target remains the ring centre, this case is not an M1 must-pass criterion;
  M2 will define ego plate-0 assignment/approach before cable collision geometry is added.
- Case-defined `online_join_planner` static obstacles are relevant to Python
  diagnostics/local phases. The current C1F.6 C++ transfer configuration keeps its
  own static scene disabled, so `static_two`/the static part of `mixed` should not be
  treated as proof of C++ static-obstacle transit avoidance.

## Other commands

Run a GUI preflight without starting the stack:

```bash
tools/sim_test/run.sh preflight --scenario c1f6 --gui
```

Reanalyse the newest C1F.6 bundle:

```bash
tools/sim_test/run.sh analyze --scenario c1f6
```

Reanalyse a specific bundle:

```bash
tools/sim_test/run.sh analyze --scenario c1f6 \
  --bundle logs/c1f_experiments/c1f6_manual_<timestamp>
```

If the compact evidence is insufficient, generate a derived drill-down around a phase:

```bash
tools/sim_test/run.sh analyze --scenario c1f6 --detailed \
  --focus-phase TRANSIT_TO_REATTACH
```

Or around a time in seconds from observed `TAKEOFF`:

```bash
tools/sim_test/run.sh analyze --scenario c1f6 --detailed --focus-time 52.0
```

Focused output is stored under `analysis/drilldown/<focus>/` as `window.csv` and
`window.png`. When `--detailed` has no explicit focus, the analyzer selects the first
hard failure, then the first warning, then the terminal phase. The large
`plot_join_planner_log.py` output remains available only as a later, explicit
join-planner-specific investigation.

For infrastructure qualification only, the runner also supports explicit headless
automatic control:

```bash
tools/sim_test/run.sh run --scenario c1f6 --headless --auto-arm
```

Automatic arming is restricted to a scenario marked `simulation_only` and to a stack
launched and owned by this runner. It is not the default planner-development workflow.

## Verdicts and exit codes

The analyzer reports infrastructure, mission, and safety gates independently. Any
`UNCERTIFIED_*` execution outcome fails the safety gate even if the mission state
machine reaches its success phase. Numerical comparison remains `NOT_BASELINED` until
a baseline and thresholds are deliberately frozen.

- `0`: all hard gates passed.
- `1`: mission or safety failure.
- `2`: preflight, launch, collection, or analysis infrastructure failure.
- `130`: user interruption after cleanup and artifact collection.

Machine-readable results remain in `supervisor/result.json`, `analysis/verdict.json`,
and `analysis/metrics.csv`. Canonical subsystem logs remain in their existing locations
and are copied into `raw/`; compact analysis does not alter them. `--no-plots` remains
available for environments where images must be suppressed.

## M2 hand-over bench (multi_drone_control, G5)

Starts where `drive_m2_handover.py` hands over: Tejen's four X3s welded to the dynamic
0.86 kg ring on plates 3/0/6/9 (ring yaw 20 deg, resting on its legs at z 0.100), rods at
the T0019 hold at the mux switch (t=104.5 s sim, 73.5-74.4 deg at the pivot), each X3 held
by a detachable hanger. No join, no
mission: about 4 min instead of 20-25.

```bash
source ~/ros2_humble/install/setup.bash && source install/setup.bash
python3 tools/sim_test/make_m2_bench_world.py        # -> simulation_assets/tejen/bench_m2/
./tools/run_experiment.py configs/experiments/m2_bench.yaml
```

- `make_m2_bench_world.py` runs his `m2c_ground_spawn.generate_m2c_assets` (read-only) in a
  temp dir and keeps his world, ring and joint plugins. It applies the runner's 0.86 kg ring
  patch and re-makes the X3 copies at the hold pose. It then adds `bench_hanger_i` (a link
  welded to the world + DetachableJoint to `x3_i::X3/base_link`) and checks the written SDFs: rod tip
  <5 mm from its plate, rod 0.450 m, and `gz sdf -k`. `--evidence <his run dir> --t <sim s>`
  takes the hold from another run's `attachment.csv`. `--hanger static` (static model) and
  `--hanger kinematic` (the `m2a_x3_support.sdf` pattern) are kept for comparison only:
  dartsim welds a root link into the parent's skeleton and leaves it there on detach, so a
  static parent likely keeps the X3 frozen after release, and dartsim ignores `<kinematic>`,
  so that support likely does not hold. `--weld-gap-m` lifts the magnets
  (T0019 welded 6-7 mm above the plates; the default is 0).
- Both kinds of DetachableJoint attach on the first step. Nothing sends
  `/drone_i/magnet/detach`, so the magnets stay welded. The hangers release on
  `std_msgs/Empty` to `/bench/hanger_i/detach` (bridged), and their state is on
  `/bench/hanger_i/state` (String).
- `sim_m2_bench_launch.py` (bringup) starts his sim plumbing as his M2D launch
  does: the unthrottled `/clock` bridge, the ring pose bridge, the magnet joint bridge, and
  per drone the pose/motor bridge, `tejen_motion_capture_emulator` and
  `tejen_betaflight_communication`. `rate_ki` defaults to M2_RATE_KI, else SIM_RATE_KI,
  else 5. It also bridges the hangers. `start_gz:=false` is for run_experiment, which
  starts Gazebo itself. Standalone, it starts `gz sim -s -r` on the bench world.
- `m2_bench.yaml` runs our stack with the driver's args: ARM, then `FLEET_HANDOVER`
  (`/fleet/handover`), then TAKEOFF. `WAIT_THRUST` waits until all four muxes forward our
  armed, above-idle command. `HANGER_RELEASE` follows 0.05 s later, then `WAIT_LIFT 0.3`,
  a 15 s hover and LAND.

What the bench does NOT reproduce:
- his join transients: the approach, capture, proof excitation and the 15 deg hold
  dynamics. The fleet starts at rest on the hangers instead of flying under his MPC.
- his MPC's final state: its commands, integrators and the rate-loop I-term of his
  bridges at the switch. Our tracker takes over from zero motor speed on a held body,
  not from his hover throttle.
- tether swing and ring motion from the join. The ring rests exactly on the floor, and
  the rods are still (within 1.1 deg of radial, as logged).
- the magnet weld offset. The magnets sit on the plate centres, while his welds are
  1-3 mm off-centre and 6-7 mm high. The bodies are therefore 6-7 mm lower than in T0019
  (z 0.595-0.598).
- his pendulum/observer/planner CPU load. The bench wall time and RTF are not M2's.
