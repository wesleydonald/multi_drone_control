# Design note — software-in-the-loop bench (`tools/sil_bench.py`), THESIS_PLAN §9.3

**Status: DRAFT, awaiting approval. No code written yet.**
Written after reading `controller_mpc.py`, `dissipative_node.py`, `mini_plant.py`,
`verify_dissipative.py`, `payload_betaflight_comm.py`, `dynamics.py`,
`three_attach_launch.py` and `three_attach.sdf` / `x3_drone3_magnet.sdf` — not from
the plan summary. (`fleet_safety.md §6` records what happens when a note is written
from a summary instead of the code.)

---

## 1. The problem

Every control decision in this project is currently made by launching Gazebo, watching,
and inferring. Gazebo runs at RTF ~0.4, leaks DDS segments between runs, and needs a
human at the RViz buttons. That is the documented dominant failure mode: **iterating
blind**.

The offline harness (`verify_dissipative` + `mini_plant`) is fast but drives a **PD
tracker proxy**, and `CLAUDE.md §8` and the harness's own Test I/J comments record,
with evidence, that it *cannot reproduce the Gazebo divergences*. Fixes have shipped on
its evidence and been walked back.

So there is a hole between "2 seconds, cannot see the failure" and "10 minutes of
wall time and a human, can see the failure". Everything in Parts A and B of the plan
lives in that hole.

**What the bench is:** the real controller nodes, unmodified, closed around a fast
numerical plant over real ROS topics, with no Gazebo and no human.

**What the bench is not:** a Gazebo replacement. It has no contact physics, no
aerodynamics, no rotor dynamics and no mocap noise. Its output is a *discriminator*,
not a validation. Gazebo and the rig remain the authorities.

## 2. Why the existing harness cannot do this, mechanically

Three properties of the real tracker are the runaway mechanism, and a PD proxy has
none of them:

| Real tracker (`controller_mpc.py`) | PD proxy (`mini_plant`) |
|---|---|
| **No integrator.** A sustained force above the modelled feedforward becomes a sustained position error that nothing ever removes. | `kp*(p_ref - p)` grows without bound until the error is corrected. |
| **Caps the cable feedforward** — `CABLE_ACCEL_CAP = 6.0` on the measured path; on the model path it only ever sees the planner's `t·s/m`, which was logged at ~1.66 while the true pull reached 17. | The proxy is handed `a_ff` directly and applies it exactly. |
| **Solver dynamics** — 20-node horizon, `u_dot` bounds (throttle 0.5/s), terminal cost, warm start, failure/hold path. | Instantaneous algebraic force. |

And a fourth that I think matters as much and is not in the plan's list:

| **The real drone must tilt to push sideways.** Thrust acts along body **z** only; horizontal force costs vertical force, and the attitude loop has lag (`tau_rate = 0.12 s`) plus a rate limit. A drone being dragged outward sinks. | `mini_plant`'s drone is a **point mass with omnidirectional force**. It never tilts, never sinks while accelerating sideways, and has no attitude at all. |

The 2026-07-28 record for the 45° case is *"ERR y grows REF 0.30 → ACT 1.06, |aCm| → 20,
tilt 10 → 33 → 72°"* — a drone being physically hauled outward while its thrust vector
cannot follow. A point mass cannot express that failure. **Adding per-drone attitude
dynamics is, in my judgement, the single most load-bearing fidelity item, and §9.3
does not mention it.** If the bench fails its acceptance test, this is the first
thing I would suspect, not the last.

## 3. Architecture

```
                            tools/sil_bench.py  (one ROS node + a process supervisor)
   ┌──────────────────────────────────────────────────────────────────────────┐
   │  SilPlant (pure numpy, no ROS)          scripted events (sim-time)        │
   │    n quadrotors, 6-DOF                    ARM · TAKEOFF · WELD · LAND     │
   │    payload rigid body                                                     │
   │    rods, ground contact                                                   │
   └──────────────────────────────────────────────────────────────────────────┘
        │ /clock                                                    ▲
        │ /drone_i/motion_capture_state   /payload/motion_capture_state
        │ /drone_i/imu                                              │
        ▼                                                           │
   ┌─────────────────────────┐        /drone_i/reference_trajectory  │
   │ dissipative_controller  │ ───────────────────────────────────┐  │
   │  (REAL node, unmodified)│                                    ▼  │
   └─────────────────────────┘                    ┌──────────────────────────┐
   ┌─────────────────────────┐                    │ controller_i × (n+1)     │
   │ central_controller      │  /fleet/step       │ (REAL acados trackers)   │
   │  (REAL fleet manager)   │───────────────────▶│                          │
   └─────────────────────────┘                    └──────────────────────────┘
                                                        │ /drone_i/ELRSCommand
                                                        └────────────────────┘
```

Every box marked REAL is the same executable the Gazebo launch starts, with the same
parameters (§3.2). The bench replaces exactly three things: **Gazebo**, the
**betaflight/motor bridge**, and the **magnet/approach chain**.

### 3.1 Lockstep sim time — the timing decision

The bench owns `/clock`. It advances sim time **only after every tracker has published
a fresh `ELRSCommand` for the current step**:

```
publish /clock(t), poses(t), imu(t)
wait until all n+1 trackers have published a command stamped after t   (wall timeout)
apply commands, integrate the plant DT=20 ms
t += DT
```

Why lockstep rather than free-running:

- The project has already lost A/B comparisons to this exact effect — `controller_mpc.py:141`
  records two runs of the same trajectory at RTF 0.40 and 0.60 giving payload radius
  errors of −16% and −34%, *"which made every sim A/B silently incomparable."*
  Lockstep makes the result independent of machine load **by construction**, not by
  hoping the machine is idle.
- Repeats become genuinely repeatable, which is what E1/E2/E7/E11 need.
- It self-paces: a slow solve slows the bench instead of silently dropping a control cycle.

**Constraint this creates, stated plainly:** the tracker's pose watchdog
(`POSE_TIMEOUT_THRESHOLD = 0.25 s`) and the envelope's `safety_ref_timeout_s = 1.0 s`
are **wall-clock** and are not parameters (the first is a module constant). So one
bench step must complete in < 0.25 s wall and the planner must publish within 1.0 s
wall. Four acados tracker solves are a few ms each and the 10 Hz planner OCP is the
slow one; I expect ~1–3× real time, but **this is a measurement, not a claim** — the
first thing the bench will print is its achieved wall-time-per-sim-second, and if the
margin is thin the answer is to raise the two thresholds *as parameters* rather than
to abandon lockstep.

### 3.2 Node parameters: one source of truth

The bench does **not** restate the controller parameters. It imports the real launch
file's `launch_setup`, runs it, and **filters the returned node list by package**:

| Package | In the bench? |
|---|---|
| `controller_quad_load` (`controller`, `main`) | **kept** |
| `controller_dissipative` (`dissipative`) | **kept** |
| `ros_gz_bridge`, `simulation_communication` | dropped — the bench is the simulator |
| `drone_magnet`, `controller_mpc_payload` | dropped — see §4 |

So `three_attach_launch.py` stays the single authority for every controller parameter,
and a launch-file change lands in the bench automatically. This is architecture
principle 1; the alternative (a parallel parameter list in a bench YAML) is exactly the
drift F9 exists to close.

Two remappings need handling and are the only place the bench deviates:
drone 3's `ELRSCommand` → `_diss` (the mux) is **removed**, since there is no mux; and
`/drone_3/command` → `/fleet/command` is kept.

## 4. The weld harness (decision D5)

The plan's D5 asks for *"a simple deterministic weld harness so reconfiguration can be
tested independently while [Tejen] is still developing."* The bench is that harness.
Neither `online_join_planner`, `magnet_attachment_manager` nor the approach MPC runs.
Instead:

- **Pre-weld**, the bench flies drone 3 with a plain stand-in position controller (a
  clearly-labelled internal PD-on-attitude loop, *not* the real tracker) to its weld
  pose above the target ring point. It is dynamically consistent — it is the same
  6-DOF quad model as everyone else — so its state at the weld instant is a real state,
  not a teleport.
- **At the scripted sim-time weld**, the bench simultaneously: engages the rod between
  drone 3 and the payload weld point in the plant, and publishes
  `/magnet/object_attached: true`. That single Bool is precisely the signal
  `dissipative_node._magnet_attached_cb` consumes in the real stack, so the controller
  side of the handover is exercised **exactly** as it is in Gazebo — control authority
  transfers to the real tracker at the weld, with the real reference discontinuity.

This is what makes sub-claim **N2** (handover without loss of stability) testable
independently of another person's schedule (risk R4/R10).

**Weld geometry, from the SDF:** `x3_drone3_magnet.sdf` is
`payload =weld(fixed)= tip =BALL= arm(0.5 m) =BALL= drone`. After the 2026-07-28 ball-joint
fix, both ends are balls, so the newcomer's connection is *structurally a 0.5 m rod
anchored at the weld point* — the same element as a tether, with a different anchor and
length. The plant models it that way, which is why no new joint type is needed.

## 5. The plant — `SilPlant`

Pure numpy, no ROS, unit-testable, in `tools/sil/plant.py`.

### 5.1 Per drone (this is the new part)

State: `p, v, q, ω`. Inputs: the four ELRS channels, decoded with the **same** mapping
`payload_betaflight_comm` uses, so what the bench feeds the plant is what the sim
feeds Gazebo:

```
throttle u₂ = (channel_2 + 1)/2
ω_cmd       = betaflight_rates(channel_0, channel_1, −channel_3)      [dynamics.py:123]
ω̇           = (ω_cmd − ω) / τ,            τ = 0.12 s                   [tau_rate]
q̇           = ½ Ω(ω) q
a_thrust    = R(q) · [0, 0, c·u₂²]                                     [quadratic plant]
v̇           = a_thrust − g ẑ + a_cable + drag
```

Two deliberate choices:

- **Quadratic thrust `a = c·u²`, c = 88.6.** This is the SDF motor model, and it is
  what makes the tracker's assumed-linear `kT` wrong away from its operating point —
  the whole reason `_scheduled_kT`, `AIRBORNE_MARGIN` and the kT estimator exist. A
  linear plant would delete that error term and flatter the tracker.
- **Attitude, not omnidirectional force.** §2. Rate loop only; no motor mixing, no
  rotor inertia — the mixer/ESC layer is below the timescale that matters here.

**IMU:** the tracker's `measured_cable_accel()` needs `/drone_i/imu`, and `|aCm|` is
half the acceptance criterion. Published as body-frame specific force
`a_body = Rᵀ(a_thrust_world + a_cable_world)`, which reads `[0,0,+9.81]` at rest —
matching the contract in `controller_mpc.py:552`.

### 5.2 Payload and rods

Reused conceptually from `mini_plant` (rigid body, diagonal inertia, stiff two-way
rods, one-way ground penalty), because that part is already gate-proven. Semi-implicit
Euler at 1 ms, 20 substeps per 20 ms control step.

### 5.3 Why a new module rather than extending `mini_plant`

`mini_plant` is load-bearing for gate stage 5 (tests A–K). Adding attitude to it would
change the plant under eleven passing tests and I would not be able to tell a real
regression from a fidelity change. `SilPlant` is new; `mini_plant` is **frozen**.

This does create two plants, against architecture principle 1. I think that is correct
and want it recorded as a decision: they are two *fidelity levels*, not two copies —
the PD point-mass one is the fast reference-generator gate, the 6-DOF one is the
controller-in-the-loop bench.

**Correction (2026-08-05).** An earlier draft of this section claimed the shared physics
(rods, payload, ground) was "factored into one module both import", so that only the
drone model was duplicated. That is not what was built. Freezing `mini_plant` and
factoring code out of it are mutually exclusive, and freezing won: the rod, payload and
ground-penalty code is **duplicated** between `mini_plant` and `tools/sil/plant.py`.
The cost is real — a fidelity fix to one will not reach the other, and the two can drift
apart silently. Accepted deliberately in exchange for not disturbing gate stage 5, but
it is duplication, not factoring, and should be read as a known debt.

## 6. Acceptance test (§9.3, non-negotiable)

Config, from the 2026-07-28 record: gap-centre weld `attach_x_offset = −0.08` (az 180),
`diss_balanced_tensions = true`, `attach_central = false`, `attach_handout = true`,
`attach_elev_deg = 45`.

**Bar 1 — it must diverge at 45°.** All three, measured on drone 3 in the 20 s after weld:

| Signal | Gazebo, 2026-07-28 | Bench must show |
|---|---|---|
| drone 3 tracking error ‖p_ref − p‖ | 0.30 → 1.06 m, growing | > 0.5 m and monotonically growing |
| measured cable accel ‖aCm‖ | → 20 m/s² | peak > 15 m/s² |
| payload tilt | 10 → 33 → 72° | > 45° |

**Bar 2 — it must NOT diverge at 65°.** Same config, `attach_elev_deg = 65`, which
Gazebo records as *stable but parked at ~25° tilt*: bench tilt bounded < 40°,
‖aCm‖ < 5, drone 3 error < 0.15 m.

**Bar 2 is not optional and is not in the plan.** A plant that diverges for everything
reproduces the runaway by accident and is worth nothing. The claim being made is
*discrimination* — the bench separates the configuration that failed from the one that
did not — and only Bar 1 + Bar 2 together support it.

**What falsifies this design:** if Bar 1 fails with attitude dynamics present, the
mechanism is not in the tracker/plant coupling and the working hypothesis in
§12.2/A1 is wrong. That is a *result*, and it should be written down as one rather
than tuned around. Per risk R1 the fallback is +3 days of fidelity (solver timing
jitter, IMU noise, rod compliance, magnet-arm inertia) and then headless batch Gazebo
as the discriminator.

## 7. Blind spots — the honest list

To be printed by the bench itself on every run, so a result can never be quoted
without them:

- No contact/collision physics; no aerodynamics or prop wash; no motor mixing or ESC
  saturation asymmetry; no mocap noise, dropout or latency; no DDS timing jitter
  (removed on purpose by lockstep — which means the bench also cannot show a failure
  *caused* by timing jitter).
- The magnet arm is a massless rod: its swing inertia is not modelled, so a
  pendulum-mode instability would not appear.
- The payload inertia is a guess (`0.004 kg·m²` diagonal, inherited from `mini_plant`),
  and payload rotation is central to the attach failure. This number should be
  measured against the SDF before Bar 1 is trusted — it is on the implementation list.
- Bench agreement is **necessary, not sufficient**, in exactly the way `CLAUDE.md §8`
  says of `mini_plant`. Nothing ships to the rig on bench evidence alone.

## 8. Deliverables and how it is verified

| File | What |
|---|---|
| `tools/sil/plant.py` | `SilPlant` + shared rod/payload/ground physics. Pure. |
| `tools/sil/bench_node.py` | The ROS node: clock, poses, IMU, command capture, lockstep. |
| `tools/sil/scenario.py` | YAML scenario → launch filter + scripted sim-time events. |
| `tools/sil_bench.py` | CLI entry. Writes a `results/` run directory (§4.1 layout). |
| `tools/test/test_sil_plant.py` | Unit tests, below. |
| `configs/sil/attach_ring_45.yaml`, `attach_ring_65.yaml`, `carry_hover_n3.yaml` | Scenarios. |

Unit tests on the pure plant, each named for the property it protects:

1. A level drone at hover throttle holds altitude (thrust/gravity balance is right).
2. A commanded roll produces the betaflight rate curve within tolerance of `dynamics.py`'s
   own `betaflight_rates` — i.e. plant and MPC model agree on the actuator mapping.
3. A tilted drone at constant throttle **loses altitude** (the cos-θ term exists at all).
4. IMU at rest reads `[0, 0, +9.81]`; free-fall reads `[0,0,0]`.
5. A taut rod at rest carries `m_L·g / n` per drone (static tension is right).
6. Energy does not grow in an undriven, undamped 1 ms integration over 10 s (integrator sanity).

Gate wiring: a **SIL smoke scenario** (3-drone hover, 10 s sim) added as gate stage 6,
and the two attach scenarios as an on-demand acceptance target — not in the gate,
because they are ~40 s of sim each and the gate must stay fast enough to be run.

## 9. Scope check against §9.3

§9.3 says *"mini_plant (extended)"*. I am proposing **not** to extend it (§5.3), and to
add attitude dynamics it does not call for (§2), and a second acceptance bar it does
not call for (§6). Those three are the deviations; everything else is as written. They
need your approval before I write code.
