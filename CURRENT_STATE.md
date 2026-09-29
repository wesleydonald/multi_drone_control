# CURRENT STATE — multi_drone_control

Updated 2026-09-27 (Tejen integration, M1/M2 claims and the adopted rejoin defaults: §1, §2, §3,
§4.11, §7, §10–12; the rest as of 2026-09-16/25). This is the one document to read first. It condenses the chronological
logs that used to live in `learning.txt`, `update.txt`, `progress.txt` and `controllers.txt`
(removed 2026-09-16; still in git history). Every number cites a run id; runs live in
`results/<date>/R####_*/`, the index is `results/registry.csv`.

Companions: `general/THESIS_PLAN.md` (the plan, Aug 2026, partly superseded),
`general/TERM3_STRATEGY.md` (schedule re-baseline, 9 Sep), `docs/design/*.md` (design
notes with derivations), `docs/experimentation/*.md` (findings), `docs/decisions.md`
(dated decisions, OPEN lines wait on Wesley), `README.md` (install, build, launch),
`docs/GOALS.md` (the 2026-09-26/27 loop's plan: G1–G6 status lines, Parked, Questions, Tejen review
items), `authority.md` (Tejen integration notes; §6 lists every change to his forked code).

---

## 1. What this is

A fleet of 2–4 Betaflight quadrotors carries a cable-suspended payload cooperatively.
Drones can **detach** from the payload mid-flight and a free drone can **attach** to it
mid-flight, so the fleet reconfigures without putting the load down. Mid-flight physical
attach is the thesis contribution. Submission Thu 27 Nov 2026.

| Piece | Source | Role |
|---|---|---|
| Centralized OCP planner | Sun et al. 2025, arXiv 2501.18802 (`docs/papers/`) | trajectory following, takeoff (ground break-off), detach via fleet resize |
| Dissipative virtual-node network | Quan et al. 2025, arXiv 2509.03563 (`docs/papers/`) | hold and reconfigure; a member leaving or joining is its native mode |
| Physical attach: rendezvous, weld, authority handover, reconfiguration | this thesis | a free drone with a swung electromagnet welds onto the carried payload and joins |

The approach drone, its magnet and its approach controller are a collaborator's (Tejen).
This project's controllers take over at the weld.

Since 2026-09-25 his code is forked into this tree (base `a7524bae` of his `drone_cage_control`,
branch `SwungPayload2026_Collaborative`; `authority.md` §1): `src/tejen_mission` (mission planner
`online_join_planner`, magnet manager, supervisors, launches), `src/tejen_mpc` (his payload-pendulum
MPC), `src/tejen_dynamic_planner` (C++ transfer planner, needs GLPK), `src/tejen_utility_objects`,
his worlds under `simulation_assets/tejen/`. Two integrated demos, both claims SUPPORTED in sim
(§4.11): **M1** (four carriers start attached, orbit, drone 3 detaches, flies his pickup/drop
mission, is handed back and rejoins by weld + OCP resize, LAND) and **M2** (his four-drone ground
join, hand-over to our OCP, lift, hover, LAND). Git: branch `integration` (from
`week1-ready-for-testing` 35c2363, first commit d8aea49) holds `src/` and `simulation_assets/` only;
`tools/`, `docs/`, `configs/` stay untracked (backup `results/backups/pre_loop_2026-09-26.tgz`).

Payload: Te-Jen's M2A ring (`simulation_assets/m2a_ring_fixture.sdf`): a 500 mm ring of
24 segments carrying 12 magnet plates every 30 deg on its top face (plate 0 on +x), so
attach azimuths are clock positions on the plates. Official mass 0.86 kg (2026-09-23).
Every world carries this model since 2026-09-23 (`generate_rigid_world.py`, ring inertia
Ixx 0.0273 / Izz 0.0545 at 0.86 kg); the fixture's bench legs are left out because the
flying payload rests on its ring (mocap origin 5 cm off the floor). RViz draws the same
ring and plates (`fleet_viz`). `params.py`, every launch default, the Gazebo and SIL
configs and the SIL plant moved to 0.86 kg the same afternoon (Wesley's word); the
inertia now follows `load_mass` through `params.load_inertia` and
`tools/sil/plant.PayloadParams`, so a `load_mass` arg carries its ring inertia. Robustness
arms that deliberately mis-tell the mass carry `mis_seed: true`;
`tools/run_experiment.py` refuses any other config whose `load_mass` differs from its
world.

---

## 2. Architecture

Two reference generators feed one unchanged per-drone tracker over one wire format.

```
mocap (/drone_i, /payload motion_capture_state)
        |                                   |
  OCP PLANNER (10 Hz)              DISSIPATIVE NETWORK (10 Hz)
  controller_load_mpc              controller_dissipative (subclasses the planner)
        +----> /drone_i/reference_trajectory <----+
               [n_nodes, dt, p(3), v(3), a_ff(3), a_cable(3)] x (N+1)
                          |
             PER-DRONE TRACKER (50 Hz) controller_quad_load
             control_mode: mpc | velocity | velocity_after_handover
                          |
                   /drone_i/ELRSCommand  ->  sim betaflight emulator | real ELRS
```

- **Phase dispatch.** The dissipative node runs the OCP for creep, lift and carry, and
  switches to the network on a detach, a weld, or `auto_network_handover`. Takeoff is
  always the OCP; the network cannot break the load off the ground.
- **Tracker modes.** `mpc` is the original cable-aware acados tracker (no integrator,
  cable feedforward capped at 6 m/s²). `velocity` is the paper's architecture: a
  velocity loop with attitude and rate layers (`velocity_loop.py`), which cannot fly the
  lift. `velocity_after_handover` flies MPC through the lift and the velocity loop after
  the network engages. This is the default in `three_attach_launch.py`.
- **Common-mode load trim** (`diss_ki_load`): one z-only integrator on measured load
  height, distributed identically (or by tension share with
  `diss_trim_share_weighted`) to every attached node. Per-drone integrators tilt the load.
- **Measured-force throttle (INDI-lite) was removed on 2026-09-23** (Wesley): exact at the
  drone, never beat the classic loop on the moving load (§4.4 table 2, §6). History in
  `docs/design/measured_force_loop.md`; the runs stay in the registry.
- **kT starts typed and is trimmed in flight (since 2026-09-23).** Sim launches derive the
  start value from the operating point (`thrust_ratio:=auto`, `thrust_model.py`); hardware
  starts from the measured 24. A wrong kT lands in payload height, not throttle; the
  per-drone trim of §4.10 removes it in tethered hover.
- **Detach** has two modes: `reconfig_mode:=network` (network redistributes, then hands
  back to a pre-built OCP for the new n) and `reconfig_mode:=ocp` (the OCP resizes
  directly; attach geometry is a runtime parameter so the survivors keep their true
  azimuths).
- **Attach chain** (`drone_magnet`): join planner, target publisher, magnet manager,
  `elrs_mux` (forwards the approach controller's commands until the weld, then ours;
  never hands authority to a stream that is not commanding thrust).
- **Partner paths (2026-09-26/27, §4.11).** M1: `three_attach_launch.py partner:=true
  partner_attached:=true`; drone 3 starts welded on plate 3, the detach is an OCP resize,
  our tracker steps it out, `/partner/release` starts his mission, `/join_planner/handoff_ready`
  hands it back at ATTACH_READY, our approach, weld, OCP resize n=3→4. M2: his join, then
  `dissipative_launch.py partner_m2:=true sim_interface:=false`; per-drone ELRS muxes switch each
  drone to ours on the first flying command after `/fleet/handover`; `airborne_start` creep holds
  live until TAKEOFF, then sweeps each rod from its measured elevation to 45° (T0008).

---

## 3. Status by capability

| Capability | State | Evidence |
|---|---|---|
| OCP carry: takeoff, hover, line, circle, fig-8, n=2–4 | works in sim; the baseline | gate SIL smoke R0232 settles 0.598 m on a 0.60 target |
| Detach, OCP fleet resize (`reconfig_mode:=ocp`) | works; the stronger mode | R0111–R0114 |
| Detach, network then OCP hand-back | works; larger transient | R0108, R0109 |
| Dissipative trajectory tracking | resolved by `velocity_after_handover` + common-mode trim | §4.4 table 1 |
| Attach during a circle, 3/12/9 layout (the demo) | works in sim, repeatable | R0196, R0197, R0234, R0243 |
| Attach round trip (attach, resume, detach the newcomer) | clean | R0231 |
| Attach on a fig-8 | clean | R0226 |
| Attach on the 4/12/8 layout | level hover, rolled circle; layout OPEN | R0266–R0272 |
| Attach during a circle, measured-force throttle | REMOVED 2026-09-23; best weld transient, aborts on the moving load at gain 1 | R0244, R0247, R0257 |
| Robustness to parameter mis-seed | asymmetric envelope, table in `docs/experimentation/robustness_table.md` | R0215–R0222 |
| Hardware carry, detach, attach | **not started**; launches and rig checklist exist; takeoff bugs fixed on the rig 2026-08-03 | — |
| M2: his four-drone ground join → our takeover, lift, hover, LAND | claim SUPPORTED in sim (reviewer), rate_ki 5; GUI watch owed | T0018, T0019 |
| M1: attached start, detach, his pickup/drop (0.01 kg ball), rejoin, LAND | claim SUPPORTED in sim (reviewer); Wesley watched R0687 | R0685, R0686 |
| Combined M1: moving rejoin, 0.1 kg object, height integral in the orbit | claim SUPPORTED in sim (reviewer); canonical defaults re-flown | R0696, R0697; R0699 |

The attach rows above the M rows are the network attach of Aug–16 Sep. Since 2026-09-25 the attach
is an OCP resize n→n+1 (decisions.md), and the rejoin that carries the thesis claim is M1 (§4.11).

---

## 4. Results

### 4.1 Detach

One-flag A/B, n=4 → 3 during a circle, `detach_ocp_n4` vs `detach_network_n4`:

| metric | OCP resize | network hand-off |
|---|---|---|
| settled load tilt | ~1° | 26–48° |
| survivor tracking error | ~1 mm | 60–910 mm |
| runs | R0111–R0114 | R0111–R0114 |

Network-then-hand-back sequence (hold trajectory, redistribute, prime the new solver, hand
back): definitive runs R0108/R0109 held mid-sweep, resized to n=3 on azimuths
[90, 180, 0], resumed. Minimum payload height after the detach 0.46 m (was 0.07 m before
the two fixes: do not re-zero the feedforward soft-start on an airborne hand-back, and
prime the new solver twice before handing over). Known: LAND with a parked detached drone
aborts after the trajectory completes. A 3 → 2 detach in the SIL bench capsizes because
two support points leave the CoG outside the support line; Gazebo 3 → 2 survives because
survivors re-space first.

### 4.2 Attach during a circle (the demo)

Sequence: three drones fly the circle at 0.2 m/s, ATTACH freezes the load target, the
approach welds the stationary payload at 6 o'clock, bumpless handover to the network, the
load levels, the trajectory resumes with four drones, LAND.

| run | config | pre-weld hover | weld transient | four-drone circle | notes |
|---|---|---|---|---|---|
| R0196, R0197 | first two clean runs, 3/12/9, hold 15 s | 27° tilt (geometry) | 39° → 4° in 11 s | 4–15° tilt, z 0.60 | LAND 78 s |
| R0203, R0204 | fast: hand-out 8 s, hold 10 s | — | level in ~8 s | completed | ~50 s takeoff to finish |
| R0234 | classic loop baseline, kT auto | 0.569 m | 47° peak, 24° mean | 3.7° mean, z 0.601 | the registry baseline |
| R0243 | `attach_traj_hold_mode: settle` | — | hold ended itself at 13.6 s, tilt 1.0° | 3.6° | resume window 1.2° mean vs 2.5° timed |

Fixes that got it there (each from a run): newcomer tracker in velocity mode; weld point
captured from the magnet tip, rotated by payload attitude; ATTACH implies ARM in the
approach node; hold arms once per approach and lasts until the weld; bumpless xy
handover (datum shifts to the measured load at network entry); settle elevation 65° (45°
collapses at hand-out end); the network ring built from the physical tethers plus a
gap-centre placeholder (it was a 0/90/180/270 ring against 0/120/240 tethers); symmetric
hand-out (incumbents' tension solution blends over the same ramp as the newcomer);
`handover_blend_s` for the incumbents' cable feedforward.

Three drones at 3/12/9 hang ~27–34° tilted by geometry (CoG on the 3–9 chord). The fourth
drone is what makes level flight possible. n=2+1 rim attach is not flown: a disc on two rim
cables is a pendulum about their chord.

### 4.3 Layout: 3/12/9 vs 4/12/8 (OPEN)

| layout | three-drone hover | weld | four-drone circle | run |
|---|---|---|---|---|
| 3/12/9, newcomer 6 | 33° tilt | 47° peak, 12 s to settle | 3.7° mean, level | R0234 |
| 4/12/8, newcomer 6, classic loop | 2.0° | 10° mean first 4 s | rolls 20–37°, low side at 12 o'clock | R0266 |
| 4/12/8, INDI 0.5 + swing | level | 7° first 4 s | 16 → 22° mean, abort +30 s | R0268 |
| 4/12/8, INDI 0.5 + swing + share trim | level | 4.9° at +10–16 s | 21–23° | R0272, first completed run on this layout |

Mechanism: on the 12/4/6/8 rim the 12 o'clock drone carries ~40 % of the load and the
other three 20 % each. A no-integrator tracker sags in proportion to force error, so the
common-mode trim cannot level it. Neither layout gives a level hover and a level circle today.

### 4.4 Tracker variants on the attach demo

Table 1, Part A architecture study on `diss_circle_n3` (n=3 drones, circle r=0.5 at 0.6 m/s,
medians over 5 and 3 runs):

| metric | MPC tracker | velocity loop, `vel_ki 0` | + common-mode trim |
|---|---|---|---|
| payload radius ratio | 0.816 | 1.001 | unchanged |
| payload phase lag | 0.676 s | 0.514 s | unchanged |
| payload RMSE (sweep) | 0.272 m | 0.215 m | 0.215 m |
| payload height | 0.498 m | 0.498 m | 0.600 m |
| post-sweep tilt | 1.4° | 5.4° | unchanged |

Table 2, measured-force throttle (INDI) on the attach demo, 3/12/9:

| config | weld +0–4 s mean/max | weld +4–10 s mean | four-drone circle mean/max | outcome | run |
|---|---|---|---|---|---|
| classic loop | 28.8 / 47 | 28.8 | 3.7 / 9.4 | completes | R0234 |
| INDI gain 1 | — | 7.1 | — | ~1 Hz tilt growth, abort +18 s | R0244 |
| INDI 0.5 | 32.8 / 38 | 17.4 | 8.5 / 16 | completes | R0247 |
| INDI 0.5 + anti-swing 0.3 | — | 18.2 | 5.7 / 11.1 | completes; best compromise | R0257 |
| INDI 1 + anti-swing, mass +25 % mis-seed | — | — | 21–22 mean, 50 max | first full run of this mis-seed | R0255 |

Mechanism at gain 1: INDI makes each drone a stiff position servo, the load hangs from four
fixed points on rods whose lengths the network only approximates, rods go slack, a load
on intermittently slack rods is a pendulum. The compliant MPC tracker kept rods taut by
sagging into the load. The newcomer's reference leads the physical load by 0.4 m after the
resume in every run (network pins its payload node to the desired position); measured
rod length (0.490 m) and a horizontal leash both failed to fix it (§6).

### 4.5 Thrust ratio and hover height

The OCP hover parked the load 12–18 cm high at 0.6 kg because the fixed sim kT (32.9) was
the secant at the 0.4 kg operating point. SIL A/B on `carry_hover_n3`, identical throttle
0.391:

| kT | settled load z (target 0.60) | run |
|---|---|---|
| 32.9 | 0.782 | R0230 |
| 33.0 | 0.773 | R0229 |
| 34.6 | 0.632 | R0228 |

`thrust_model.py` derives kT per launch from load mass, fleet size and cable elevation and
reproduces both measured hover points to 1e-3. Gate smoke now settles at 0.598 (R0232).

### 4.6 Attach faults and the weld-mechanics ladder (Aug 2026)

Three stacked faults, each masking the next, all fixed: the mux handed authority to an
armed-idle stream so the newcomer's motors cut at every weld; the approach drone spawned on
a corridor through drone 0; the pose watchdog ran on the wall clock in a 0.3× sim.

Weld ladder, welded runs, payload tilt after the weld:

| variant | survival s | tilt +2 s | tilt +8 s | runs |
|---|---|---|---|---|
| ball baseline | 10.1–10.2 | 46–49 | 31–34 | R0126, R0128 |
| rod (proper arm inertia) | 10.4 | 47 | 36 | R0129 |
| seg2 (extra mid-span joint) | 10.0–10.3 | 40–44 | 26–35 | R0132, R0133 |
| rigid (moment-transmitting) | 1.7–1.9 (5/5) | 83–106 | — | R0135, R0138–R0141 |

Compliance does not help; a genuinely rigid weld capsizes 5/5. Decision (9 Sep): the
attachment is modelled as a ball joint after the weld, the `rod` world is canonical, and
the rigid-weld control problem is out of scope. [Partly superseded 2026-09-27: the partner
drone's rod is on universal joints at both ends in sim, a DART workaround; §4.11.]

### 4.7 Robustness to a wrong controller belief (E10)

Attach-during-circle demo, world true, controller mis-seeded, two runs each:

| belief | result | runs |
|---|---|---|
| mass −25 % | clean 2/2 | R0215–R0222 |
| mass +25 % | abort 2/2, load over-lifted to 0.96 m | |
| cable −10 % | clean 2/2 | |
| cable +10 % | abort 2/2, load dropped to 0.17 m | |

Under-estimates are absorbed by the common-mode trim; over-estimates are fatal within ~10 s
of the weld. Rig rule: weigh the payload and measure the rods, err low.

---

### 4.8 Rod-length tolerance: `measure_rod_len` (2026-09-23, SIL)

The rig rods are 0.46 m against a typed `cable_len` 0.50. The tautness gate
(`CABLE_TAUT_LO_FRAC` 0.85) scales the tension feedforward by measured/typed length, so a
4 cm short rod is not "close enough": the fleet flies 47 % of the cable feedforward and
the load hangs 22 cm low. `measure_rod_len:=true` makes the planner measure each rod
(drone body to its own rim point from mocap) at the OCP handover, with the rods flat on
the floor, and use per-drone lengths for the gate and the OCP. Guards: ±15 % of typed,
rods within 3 cm of each other; on a guard failure it keeps the typed length and logs why.
Frozen tree (f8e2ef4 + 26 dirty), solver prebuilt, hover-only window, 2 repeats agreeing
to 1 mm, target 0.60:

| geometry error | feature off | feature on | measured rods | runs |
|---|---|---|---|---|
| none | 0.634 | 0.634 | 0.500 | R0308/R0309, R0314/R0315 |
| rod 0.46 typed 0.50 | 0.416 | 0.631 | 0.460 | R0310/R0311, R0316/R0317 |
| rod 0.50 typed 0.46 | 0.569 | 0.634 | 0.500 | R0312/R0313, R0318/R0319 |
| rim 0.27 typed 0.25 | — | 0.642 | 0.514 | R0320/R0321 |
| rim 0.23 typed 0.25 | — | 0.626 | 0.486 | R0322/R0323 |
| load mass +20 % | — | 0.540 | 0.500 | R0324/R0325 |
| rod 0.46 + rim 0.23 + mass +20 % | — | 0.549 | 0.474 | R0326/R0327 |

Rod errors of ±4 cm and rim errors of ±2 cm are absorbed to within 1 cm; the no-op is
exact. Mass error is not addressed: the plant was 20 % heavier than the typed `load_mass`,
which sets both the OCP model mass and the derived sim kT, so the sag cannot be attributed
to either one here (open problem 9). The feature-off short-rod runs (R0310/R0311) also fail
the peak cable-acceleration bar and show a 2e-3 tension-share spread; every other run is
below 7e-4. Reviewer verdict on the matrix: weak as first worded (baseline citation, mass
wording, tension bound), supported after the corrections above. Card:
`docs/experiments/2026-09-23_geometry_autocal.md`. Gazebo half not yet flown; the
launch default is `true` on every OCP launch since 2026-09-24 (Wesley's word; the
typed `cable_len` is the initial estimate, 0.47 on the rig from the disc fit).

### 4.9 Unmodelled load force as an augmented state (built and deleted, 2026-09-23)

Supervisor's option 3, built in the formal form at Wesley's choice: a Kalman filter on the
load's vertical motion with the unmodelled force as a random-walk state, its input the
OCP's planned load acceleration, fed to the OCP as a runtime parameter and to the tension
regulariser. SIL matrix, hover-only, target 0.60, two repeats:

| arm | est off | est on | verdict |
|---|---|---|---|
| correct mass, kT | 0.629 | 0.580 | partial (2 cm low) |
| kT typed +10 % | 0.498 | 0.501 | not supported, adds 5–6 cm p-p oscillation |
| kT typed +15 % | 0.462 | 0.483 | not supported |
| plant mass +20 % | 0.511 | 0.592 | partial (8 of 9 cm) |

The filter settles at its own fixed point, the force reconciling the planned node-0
acceleration with a stationary load, which is not the target height; the deficit from a
wrong kT does not appear at the load level. Deleted the same day (Wesley). Card, critic
and reviewer: `docs/experiments/2026-09-23_effective_load_force.md`, runs R0331–R0366.
Found on the way: the SIL bench's ARM/TAKEOFF race (TAKEOFF 2 s after ARM was rejected
when arming took longer; three "completed" runs never flew), now failed loudly.

### 4.10 Per-drone thrust-gain trim (`kt_trim`, 2026-09-23/24)

The "kT is fixed" lock was lifted by Wesley on 2026-09-23 for this: each tracker estimates
its own throttle-to-acceleration gain from what it is flying, `kT = (a_meas_z + g −
a_static_z)/(u R33)`, where `a_static_z` is the drone's STATIC vertical cable share
published by the planner (`/drone_<k>/cable_static_z`: the planned distribution
normalised to the load's weight; the planned tension itself carries climb/descend intent
and made the loop drift), first-order filtered at 1.5 s, bounded ±25 %, applied only after
the takeoff spool, while tethered, a centimetre clear of the stand/floor, at hover
(|vz| < 0.08), and frozen from the first 5 cm of the landing descent. Leaving tethered
flight resets it to the typed gain (a newcomer welds with the typed value). Always
computed in shadow and logged (`kt_hat`); fed back only with `kt_trim`. SIL matrix, last
8 s of hover, target 0.60, true gain 36.75:

| arm | trim off | trim on | kT_hat at landing |
|---|---|---|---|
| correct | 0.629 | 0.614 | 36.97–37.01 |
| kT typed +10 % | 0.499 | 0.614 | 36.98–37.00 |
| kT typed +15 % | 0.461 | 0.614 | 36.98–37.01 |
| plant mass +20 % | 0.511 | 0.619 | 35.75 (the effective gain the heavier load needs) |
| 3/12/9 uneven hang | 0.626 (1 of 2 lifts aborted at 66° tilt) | 0.617–0.618 (1 of 2 aborted on a re-fly, as off) | spread 0.4 % across unequal shares |
| Gazebo, kT typed +10 %, floor start | 0.175 (drones parked at 0.53 m) | 0.609 (lifts at TAKEOFF, estimate 40.4 → 36.1 in 12 s) | 36.25–36.30 |

(matched runs R0405–R0420, R0429–R0432, one tree, two repeats agreeing to 1.5 mm.) The
correct arm moves 1.5 cm: the fixed 36.75 is the sim plant's secant at a 45° hover the
fleet does not fly (42.6°, secant 36.92), and the trim finds the real operating point;
on the rig's linear gain that term is absent. The OFF sags are partly the fixed-gain
tracker's own takeoff/airborne gain switch chattering across the 4 cm margin (kT +15 %
off never left takeoff mode); the trim replaces that switch after the spool.
Settles 3–9 s after TAKEOFF, no overshoot, no oscillation. A floor-contact detector (`kt_trim.in_contact`: the TYPED model's predicted acceleration vs the measured one, outside −2…+5 m/s² means something else carries the drone; no update and a leak back to the typed gain) is what makes the floor start work: without it the estimate read the floor's support as strong motors, railed to +25 % and held the fleet down for 14 s (Wesley's Gazebo run, 2026-09-24 12:21). Card, critic and
reviewer: `docs/experiments/2026-09-23_kt_trim.md`. Three earlier attempts the same day
are recorded there and in §4.9: the load-level Kalman filter (deleted), the per-drone
vertical integrator (critic: cannot fix a multiplicative error; not built) and the
removed INDI throttle. Gazebo and the rig: not yet flown.

### 4.11 Tejen integration: M1 and M2 (2026-09-25/27)

How they run (plan and status `docs/GOALS.md`; changes to his code `authority.md` §6):
- **M1** `python3 tools/run_experiment.py configs/experiments/partner_attached_orbit.yaml` (canonical,
  on the current defaults). World `three_attach_partner_attached.sdf`: carriers at 150/270/30 plus drone 3
  welded on plate 3 at 90° (plates 1/3/5/9), orbit r 0.5 at 0.125 m/s. Events: LAUNCH
  `tejen_mission partner_mission.launch.py plate:=3 ... start_gate_topic:=/partner/release` → ARM → TAKEOFF →
  WAIT_LIFT → DETACH 3 (~lift+10 s) → WAIT_REWELD → LAND at weld+8 s → stop 5 s after landed; ~100–135 s sim,
  11–13 min wall. Variants: `_g3_uu` (M1 claim), `_full` (combined claim: `three_attach_partner_attached_m100.sdf`
  0.1 kg object, `object_mass_kg:=0.1`), `_full_direct`, `_g1b`, `_g1b_zki`, `_moving`; `_g3`, `_g3_vs`,
  `_g3_ds`, `_g3_stop` are the yank hunt. A partner run is VOID unless the launch log has 'drone 3 welded at
  start: plate at 90' (`metrics.json` m1.start_weld_ok; R0676v).
- **M2** `tools/clean_slate.sh; python3 tools/sim_test/drive_m2_handover.py LOGDIR false 20` (headless, 20 s sim
  hover; GUI `true 20`). His simultaneous M2D join with the `/m2d/transit_owner` token, operator hold, our
  takeover (§2), 45° creep, OCP lift to 0.60, hover, LAND; ~21 min wall (his join 12–15). `M2_RESUME=1` resumes at
  his success hold. Scored by `summarise_m2` into `metrics.json`.

M2 claim (card `docs/experiments/2026-09-27_m2_handover.md`; bars: join 4/4, 0 faults, ring 0.55–0.65 m,
ring tilt ≤ 3°, drone tilt ≤ 25°, landed):

| run | rate_ki | join | ring z mean (min–max) | ring tilt peak lift/hover/descent | drone tilt max | verdict |
|---|---|---|---|---|---|---|
| T0015c | 10 live at the hold | 4/4 | 0.604 | 1.78 | 15.5 | exploratory |
| T0016 | 10 | 4/4 | 0.5996 (0.597–0.606) | 2.33 / 0.89 / 1.87 | 15.2 | pass, not counted |
| T0017 | 10 | 4/4, 1 fallback | 0.628 (0.617–0.641) | 4.16 / 4.36 / 4.74 | 16.2 | FAIL (tilt) |
| T0018 | 5 | 4/4 | 0.601 (0.597–0.615) | 1.55 / 0.59 / 2.38 | 15.0 | claim 1 |
| T0019 | 5 | 4/4 | 0.5996 (0.597–0.606) | 1.83 / 0.77 / 1.23 | 15.4 | claim 2 |

Reviewer SUPPORTED on ki 5 (T0018 + T0019). The PI-zero mechanism is a hypothesis: ki 5 is not claimed to cure
the rocking. Ring overshoots to 0.648 / 0.650 in the 5 s after the target (outside the window). One plate
assignment (3/0/6/9), hover only; Wesley's GUI watch owed.

**The claim is WEAK since T0020; cause found 2026-09-27.** After the claim, 4 of 5 completed runs rocked (T0020, T0024,
T0026, T0027: 7–21°). Ruled out: tethers (T0021–T0024), gate loop, rim height (T0026/27), stored motion from his
welds (M-spin). The M2 hand-over bench (`configs/experiments/m2_bench.yaml`, world from
`tools/sim_test/make_m2_bench_world.py`: his four X3s start welded at the T0019 hold on releasable hangers, ~2 min
wall) is calm unloaded (R0707/R0708) and rocks only under CPU load. Cause: his sim Betaflight bridge
(`tejen_betaflight_communication.py`) differences **unstamped** mocap poses, so bunched or skipped samples at M2's
RTF 0.15–0.17 turn into rate spikes. Opt-in fix `rate_source imu` (X3 gyro; `M2_RATE_SOURCE=imu` on the full M2 path).

| arm (bench under CPU load) | ring peak > 3° | peaks |
|---|---|---|
| pose (default), R0709–R0719 | 3/6 (one abort) | 7.46 / 1.55 / 34.9 / 5.99 / 2.23 / 1.80 |
| gyro, R0710–R0720 | 0/6 | 1.05 / 1.39 / 1.40 / 0.67 / 2.26 / 1.44 |

Full M2 on the gyro: T0028 (3.44 at lift-off / 0.94 hover / 2.81 LAND) and T0029 (0.82 / 0.39 / 3.33 at LAND
onset). No sustained rock, but both miss the 3° bar on 0.3–0.5 s transients that the bench does not show (M-trans).
Gyro made the default on every sim bridge, ours and his (Wesley, 2026-09-28; `rate_source` / `M2_RATE_SOURCE` pose for the old arm). Waiting on Wesley: whether the bar is per sample or windowed (GOALS question 5).

M1 claim (card `2026-09-27_m1_rejoin.md`, config `_g3_uu`; bars: detach ≤ 8°, orbit mean ≤ 3°, rejoin ≤ 5°,
dip ≤ 0.03 m, < 2° within 3 s, ball in net, TAKEOFF→landed ≤ 140 s):

| run | detach | orbit mean | rejoin | dip m | ≥ 2° after weld | ball at LANDED | T→landed s |
|---|---|---|---|---|---|---|---|
| R0646 (baseline) | 5.4 | — | 10.4 | 0.064 | censored | n/a | 114.7 |
| R0653 (ki 5, before fixes) | 3.0 | — | 21.1 | 0.136 | censored | in net | 250 |
| R0685 | 2.8 | 0.62 | 1.7 | 0.017 | never | in net | 115.2 |
| R0686 | 2.8 | 0.83 | 1.6 | 0.018 | never | in net | 133.0 |

Reviewer SUPPORTED. Wesley watched R0687 in the GUI (rejoin 2.2, 100.4 s) and adopted the rod model and flags.

Combined M1 claim (card `2026-09-27_m1_full.md`, config `_full`: + attach_moving, 0.1 kg object with his kT
scheduled, z_ki_in_orbit; extra bars: back within 0.03 m of 0.60 ≤ 5 s after the drop, tilt ≤ 3° for 10 s after
it, ring ≥ 0.08 m/s handoff→weld, no near_bound):

| run | detach | drop: back within 0.03 (min z) | ring speed approach / after weld | rejoin | dip m | T→landed s |
|---|---|---|---|---|---|---|
| R0695 (discovery) | 2.5 | 4.4 s (0.495) | 0.130 / 0.107 | 2.8 | 0.016 | 105.6 |
| R0696 | 2.8 | 4.6 s (0.494) | 0.133 / 0.107 | 2.5 | 0.011 | 104.3 |
| R0697 | 2.6 | 4.7 s (0.491) | 0.129 / 0.103 | 3.2 (carrier kick at weld+5.5 s) | 0.020 | 132.8 |
| R0698 (+ attach_approach_direct) | 2.8 | 4.6 s | — | 2.1 | 0.010 | 129.5 |
| R0699 (canonical defaults, 0.01 kg) | 2.9 | min 0.582 | 0.131 / 0.105 | 2.4 | 0.013 | 121.2 |

Reviewer SUPPORTED on R0696 + R0697. The 104–133 s spread is inside his mission (reattach retries); our segments
are equal.

Root causes found:

| symptom | cause | fix | evidence |
|---|---|---|---|
| M2 carrier flips at our takeover (75–84°) | sim rate loop P-only (kp 0.5); his tether pivots 0.04 m below the body and the 45° rod pull needs ~0.08 N·m, which P-only answers with ~257 °/s of rate error | rate-loop I-term with anti-windup (clamp 200, integrate above u 0.09) on every sim bridge | T0007–T0010, S0001–S0004, T0011/T0012, T0015 |
| limit cycle 19 ± 4.5° at 3.1 Hz (R0647); M2 ring rock 4.74° (T0017) | ki 10 puts the PI zero (ki/kp 20 rad/s = 3.2 Hz) on the drone-tilt dynamics (hypothesis) | ki 5 | R0647–R0650, T0016–T0019 |
| M2 join stall 0/4 ('requested hull extends beyond a trajectory without a stopped hold') | our simultaneous test mode let two vehicles transit at once; his safety checker hulls a moving peer's trajectory | `/m2d/transit_owner` token, one C++ transit at a time | T0012–T0014, T0014b, T0015 |
| partner weld 80–91 s late; M1 rejoin yank 15–36° | DART ignores ball-joint `<axis><damping>`: the arm swung, and the tip kept the spin left by the ball carry, which the gz weld copied onto the ring (ring \|ω\| steps 0.78–2.27 rad/s) | partner rod on damped universal joints at both ends (removes rod twist) | S0005–S0010, R0660/R0662, R0668–R0680, R0684 |
| weld mid-descent (R0668 23.5°) | our magnet manager differenced poses over wall time: speeds 4× low at RTF 0.25, the 0.05 m/s gate passed at 0.10–0.12 m/s | `weld_velocity_clock sim` | R0668, R0669 |
| ball stayed welded after DROP | his mission's one-shot gz CLI detach drops | object magnet backend both + ros_gz bridges `/pickup/attach\|detach` | R0678, R0680 |
| weld moment | newcomer's 1 N start tension × its lever about the incumbents' apex (0.25 N·m) | `attach_t_start_new` 0.1, `attach_blend_balanced` | R0662–R0666 |
| ring 0.13 m off its xy reference at the weld | approach hold froze the orbit reference while the ring coasted | `attach_datum_shift` | R0669, R0672 |
| ring 0.066 m high through all of M1 | height integral frozen whenever traj_t > 0 | `z_ki_in_orbit` | R0653, R0692/R0693 |
| G1b stall in his LIFT_OBJECT | his MPC has no z integral and no object mass | his kT scheduled on `/tejen/object_attached` (82.7·m_d/(m_d+m_obj): 71.6 attached, 82.7 released) | R0681, R0688 |
| handoff blip (reference +7.5 cm, vz +0.22) | our ApproachProfile always started with a climb | `attach_approach_direct` | R0687, R0698 |
| z_ki_in_orbit "falsified" | start-up race: dissipative node up late, drone 3 folded in after TAKEOFF; R0676 VOID, v1 untested | start-weld validity check | R0676v |
| benches R0654–R0659 never welded | the model's pickup DetachableJoint held the tip to the ball on the floor (only his mission releases it) | `_nopick` copies; runner GZ_PUB `/pickup/detach` | R0659, R0673 |

Adopted defaults (Wesley's word each; decisions.md 2026-09-26/27):

| default | value | commit | evidence |
|---|---|---|---|
| sim rate-loop I-term, all four sim bridges + his m2d launch env | rate_ki 5 (clamp 200, u > 0.09); `SIM_RATE_KI=0` = old P-only, `M2_RATE_KI` for his m2d bridges | a55c58d (10), a4c748e (5) | R0648–R0652, T0018/T0019 |
| partner rod (shipped `x3_drone3_magnet_rod_partner[_attached].sdf`) | damped universal joints at both ends (0.002 body, 1e-4 tip) | 2050359 (copy), ac6c4cf | R0684–R0687 |
| `attach_t_start_new` | 0.1 N (was 1.0) | ac6c4cf | R0663–R0666, R0685/R0686 |
| `attach_blend_balanced` | true | ac6c4cf | R0665/R0666, R0685/R0686 |
| `attach_datum_shift` (also the rig attach path) | true | 65979a6 (opt-in), ac6c4cf | R0672, R0685/R0686 |
| `weld_velocity_clock` (magnet manager) | sim | 65979a6 (opt-in), ac6c4cf | R0669, R0685/R0686 |
| `attach_moving` | true | a86f99d (opt-in), e6aa37b | R0689–R0691, R0696/R0697 |
| `z_ki_in_orbit` (OCP path; orbit clock advanced, past 4 s spin-up, v²/r ≤ 0.05) | true | a86f99d (opt-in), e6aa37b | R0692–R0694, R0696/R0697 |
| `attach_approach_direct` | true | 6be82d4 (opt-in), e6aa37b | R0698, R0699 |
| canonical M1 LAND | weld + 8 s (config, untracked) | — | R0699 |
| Gazebo reference-staleness gate | `safety_ref_timeout_s` 3.0 (rig 1.0) | d8aea49 | W-21:08 |
| partner at-rest weld gate | 0.05 m/s (was 0.35) | d8aea49 | R0646 |

Opt-in, not default: his object-mass kT schedule (`object_mass_kg`, c809015); partner object magnet over ROS
bridges (6e1606c, used by every M1 config); mocap stamp differencing tried and removed (2b8b3b9 -> 364889d).

2026-09-28/29 (G7 work toward the rig):

| finding | cause | fix / state | evidence |
|---|---|---|---|
| our abort never reached a drone on the partner's stream (his next command re-armed it) | his MPC ignores /fleet/abort; the mux forwarded his stream | mux abort latch (disarmed idle from every input, 20 Hz, until restart, /drone_i/mux_state); ARM gate while latched; ARM-failed = service disarm only; partner-scoped faults do not escalate; operator DISARM (the rig kill switch) publishes /fleet/abort (0aef5c7) | R0751 (DISARM, drone 3 on his MPC down in 0.5 s), R0752 (M1 unchanged), T0034 (M2 ESTOP before hand-over: 4 latched, his streams armed, all down) |
| M2 join stall, 4 of 11 runs (drone_0 in M2_CPP_TRANSIT_CAPTURE forever, constant (-0.008, 0.056, -0.027) tube error) | his `advance_reference_window` padded past a mid-transit cached window with a MOVING terminal state; a backend dropout > 2 s froze the position at 0.22 m/s and the recovery check rejected every fresh reference | stopped-hold padding in our fork (37b3b5d, authority.md) | T0023/T0025/T0032/T0033 vs T0034 (3.6 s dropout recovered, join 4/4) |
| R0749 "n4 creep never swept" | not the creep: every tracker pose-timed-out between ARM and TAKEOFF; the manager still took TAKEOFF | OPEN, GOALS question 11 (card 2026-09-29_pre_takeoff_disarm_gate.md v2, critic NOT READY -> applied); rig sheet line meanwhile (c201a85) | R0749c |
| M2 tilt at lift 10.25 deg | carrier kick on drone 2 (mocap-differenced pitch rate -1.6 rad/s), same mechanism as M-kicks | OPEN, GOALS question 10; gyro-fed x0 rate kept as an off-by-default sim oracle (ced228b) | T0035 (M2 on the gyro bridges: 2 of 3 under 5 deg) |
| rig defaults | Wesley Q7 | real:=true defaults to the creep floor start + reconfig_mode ocp, typed values kept (3b7a6a8) | test_real_mode_launches |

## 5. Locked decisions (do not "fix")

- No geofence: the pilot at the cage owns out-of-bounds. Kept: reference staleness, tilt
  and speed envelopes, fleet-wide disarm on any envelope fault.
- kT starts from the typed value (sim: derived secant; hardware 24) and is corrected only
  by the bounded per-drone trim of §4.10 (lock lifted 2026-09-23). The adaptive UKF and the
  airborne kT schedule stay deleted.
- The network is yaw-only and open-loop in load pose except the bounded common-mode trim.
  Building geometry on the measured load attitude or position closes a loop that
  self-amplifies tilt (§6).
- Attach at 0.2 m/s. 0.4 m/s flips the load at the weld (R0267).
- The attachment behaves as a ball joint; N3 is stated around what the rig does. (Sim partner rod:
  universal joints at both ends since 2026-09-27, a DART workaround; the real rod's freedom unchecked.)
- The reference wire format is frozen at 12 fields per node.
- New behaviour goes behind a parameter defaulting to the old behaviour.
- `results/` is single-copy; anything behind a figure is promoted to `results_archive/`
  the same day.
- 2026-09-09: payload is the 500 mm disc, 0.6 kg in sim, rim magnets. [Superseded 2026-09-23: M2A ring,
  0.86 kg, §1.]
- 2026-09-10: tethers at 4/12/8 in `three_attach.sdf` and the launch defaults; RViz role
  colours and error lines removed, banner kept. (The detach study and M1 fly plates 1/3/5/9; the
  3-drone attach demo layout is still OPEN in decisions.md.)

---

## 6. Negatives — measured, do not retry

| idea | result | evidence |
|---|---|---|
| Terminal velocity reference (T1) | slightly worse on all four tracking metrics | R0054–R0059 vs R0061–R0065 |
| `net_traj_lean` (tilt the cone onto g_eff) | no radius benefit, contraction worse | Aug 2026 A/B |
| Per-drone integrators (`vel_ki 1`) | tilt the load to 60°; `vel_ki 0` keeps the tracking win | R0077 series |
| Tilt-aware wrench (`diss_wrench_true_attitude`) | weld transient 68°, abort 2/2 | R0212, R0213 |
| Extra weld compliance (seg2) | no change in outcome | R0132, R0133 |
| IMU-residual tension admittance (`diss_k_adm`) | worse in Gazebo (biased estimate through a secant kT) | R0250, R0251 |
| Horizontal leash (`net_pull_max`) | closes the position loop; classic loop aborts, newcomer never welds | R0258, R0259 |
| Measured rod length as the network length | no benefit; the 0.53 m reading was drone-to-rim with the tip stand-off | R0261–R0263 |
| Classic loop + anti-swing | no benefit; the compliant tracker already has the damping | R0256 |
| Full-gain INDI + anti-swing | capsizes after the weld on 4/12/8 | R0269 |
| Measured-force (INDI-lite) throttle, any gain | exact at the drone (kT told 19 % low: 0.21 m -> 4 mm) but a stiff position servo on rods of approximate length lets them go slack; code removed 2026-09-23 | R0238/R0239, R0244, R0247, R0257 |
| OCP hand-back with an even n=3 ring after 4 → 3 | 62° tilt; survivors keep their true azimuths instead | R0093 |
| SIL bench as an attach discriminator | 45° and 65° both diverge on the current plant; use Gazebo | R0034–R0043 |
| Offline harness (`verify_dissipative`, PD proxies) for tilt questions | holds level where Gazebo tilts 34°; smoke test only | — |
| Attitude-reference hemisphere alignment (rig yaw spins) | mechanism exercised in sim, no spin; the squared attitude cost is sign-invariant, so it cannot matter; code removed | R0276–R0279 |
| Sim rate-loop I-term ki 10 | limit cycle at 3.1 Hz on our X3, M2 ring rock 4.74°; ki 2 and 5 clean | R0647–R0650, T0016/T0017 |
| Steeper nominal rods for the M2 takeover (`cable_elev_deg` 70) | same lever flip (75.2°): a rigid rod turns any position mismatch into torque at the 0.04 m lever | T0010 |
| His `endsInStoppedHold` tolerance 1e-9 → 1e-6 (join stall) | same stall; reverted (cause was the simultaneous transits) | T0013 |
| Fixed or revolute tip joint on the partner rod | rod becomes a cantilever from the ring: 6.5° hang before the detach, drone 3 faults | R0682, R0683 |
| Stopping his launch at the handoff (M1 yank) | yank persists (15.4°): the cause was the state his mission leaves, not his nodes | R0679 |
| His thrust-ratio UKF for the 0.1 kg object | not flown: critic, recovers ~20–30 s after the drop, too slow for the rejoin; scheduled kT instead | card 2026-09-27_g1b_mass |

---

## 7. Open problems, priority order

1. **Layout default** for the demo: 4/12/8 (level hover, rolled circle) or 3/12/9 (tilted
   hover, level circle). OPEN in `docs/decisions.md`.
2. **Moving-load stability with a stiff tracker.** Slack rods at INDI gain 1; the
   newcomer's 0.4 m reference lead after the resume. Candidates: a slack-aware tension
   solve with a t_min bound, or softer position gains with exact thrust.
3. **Make `attach_traj_hold_mode: settle` the default?** OPEN.
4. **Lift transient variance.** The n=4 circle lift is bimodal (R0096 4° vs R0098 51°);
   `diss_circle_n3` lost 4 of 10 runs in the OCP lift. Undiagnosed.
5. **LAND with a parked detached drone aborts** after the trajectory completes.
6. ~~A drone that disarms before takeoff does not trigger the fleet abort~~ CLOSED 2026-09-29 (189b6c6): the manager
   grounds the fleet and refuses TAKEOFF (Wesley Q11); drone 3 on the manager's single command path; D0001/D0003.
7. **~18° residual tilt after 4 → 3** on the uneven surviving ring. Possibly geometry
   (`metrics.cog_margin`), not a bug.
8. **Rig yaw spins, unexplained.** 2026-09-16: one drone per flight spun a full turn at
   spool-up with the yaw stick pinned (16:40 drone 2 at +169°, 16:42 drone 0 at −109°,
   `tools/yaw_excursion.py`), the other two held heading to 2°. Ruled out: yaw actuator
   sign (consistent on all airframes), quaternion hemisphere (R0276–R0279), solver
   failure, mocap rate frame. The tracker log now records body rates, the reference
   quaternion and solver status for the next occurrence.
9. [SETTLED 2026-09-25 by the height integral, z_ki 0.4 default: R0557 0.661 → R0558 0.600; extended
   through a level orbit 2026-09-27, `z_ki_in_orbit`, R0692/R0693. History kept below.]
   **OCP hover height offset grows with mass, in the planner's reference.** 2026-09-24,
   linear sim plant: SIL R0470 hovers 5.4 cm HIGH with the tracker exact (trim converged to
   0.2 % of the plant) and every drone on its reference; Gazebo on the quadratic plant sat
   5.5 cm LOW (R0466). So this is purely the planner: node-0 drone references re-pinned to
   the measured drone positions carry no load-height feedback. Candidate: pin node 0 for
   the load state only, or a bounded load-height term on the drone references. Card first. Gazebo,
   launch defaults from a flat floor: 0.6 kg hovers at 0.566 m, 1.0 kg at 0.537 m for a
   0.60 target (R0284/R0285), with every drone on its reference to 0.1 mm and kT at the
   derived value; the drone reference itself sits 2.5 cm lower at 1.0 kg. SIL at 0.6 kg
   sits 3.4 cm HIGH (0.634, R0308, hover-only window) and 9.4 cm low at 0.72 kg with the
   0.6 kg belief (R0324), so the plants disagree in sign. Raising kT does not move it.
   Earlier SIL heights (0.589 R0282, 0.598 R0232) averaged the landing into the hover;
   the window now ends at LAND.
10. **Hardware: first carry flights 2026-09-16 did not lift** (`start_taut` from flat
   rods; use the creep path, see §9). Still the critical path. The 0.46 m rods against a
   typed 0.50 alone cost 22 cm of hover in SIL (§4.8); fly with `measure_rod_len:=true`
   or type 0.46. 2026-09-23 13:13, n=2 hover, typed 0.50, kT 24 (logs
   `planner_drone*_20260923_131300`): the payload never left the floor (z 0.05 m, xy drift
   3 mm). Drone 1 started at 21.2 V (6S, ~3.5 V/cell) and sagged to 17 V, reached 0.18 m at
   throttle 0.54, 5–10 cm under its reference; drone 0 (22.3 V) reached 0.40 m at 0.46,
   15 cm under. With the load still on the floor the planner references (which assume the
   load at its own reference height) sit inward and low, so both drones flew in over the
   disc (drone 0 to 0.13 m horizontal from the centre). The rest reference is already
   18–22 cm inward of the resting drone (start_taut assumes lifted rods). Preflight battery
   bar (15.2 V) and the `kt_batt_v_*` defaults (16.8/14.0) are 4S numbers on 6S packs.
   The preflight's own rod estimates disagreed by 6 cm (payload origin 4.3 cm off), so
   `measure_rod_len` would have refused them; type `cable_len:=0.47` (the disc fit).

11. **Carrier tracker kicks** (G4, parked, within every bar): a carrier's thrust steps 0.167 → 0.232 and pitch
   rate to −2.3 rad/s in one 20 ms step with smooth references and solver status 0; it starts as a one-sample jump
   of the MEASURED body rate (wy −0.17 → −2.28 rad/s in 14 ms). 1.6–4.6° ring excursions (R0685, R0686, R0691,
   R0696 at 47.5 s, R0697 at weld+5.1 s). Not the rate I-term (also in ki 0 runs R0640, R0642). The mocap stamp
   test (R0700-R0703) was VOID: the bridged PoseArray has no stamp (R0536), so the fix never engaged (option removed,
   364889d). Both the mocap emulator and the sim Betaflight bridge difference unstamped poses (the bridge with a 0.25 s
   average period): bunched/dropped samples under load -> rate spikes. Proposed: rate loop on the IMU gyro (every X3 model
   already has a 1 kHz base_link IMU, bridged to /drone_N/imu; opt-in rate_source:=imu). Kicks scale with the clock rate (500 Hz: 2–26 per M1; unthrottled:
   ~180 + speed faults, R0702 — clock_hz stays 500). 
12. **His MPC in M1** (Tejen review item 3, sent to him): DESCEND_TO_ATTACHMENT vz overshoot and a sharp upward
   correction; his reattach retries ('attachment descent aborted after loss of relative alignment') set the
   104–133 s spread (R0686, R0697, R0698). Measure across the M1 logs before any tuning.
13. **Post-pickup lift height** (review item 1, sent to Tejen): our `partner_mission.launch.py` override
   `pickup_lift_height` 1.40 / `minimum_search_z_m` 1.8 (his C++ transit needs z ≥ 1.8, R0639). What the 1.8 m
   floor protects is his answer; lowering both saves ~10 s per M1.
14. **Partner clearance** (review item 4, `metrics.partner_clearance`: partner to carrier body ≥ 0.30 m, to rod
   ≥ 0.15 m): our step-out 0.51–0.53 m, our rejoin ≥ 0.48 (rod) / 0.52 (body); HIS mission segment comes closest,
   0.29–0.35 m to a carrier body (R0687 0.294). For Tejen (keep-out around the carriers).
15. **0.1 kg object as the default M1 world?** Wesley's word. Canonical `partner_attached_orbit` still flies the
   0.01 kg ball (R0699); the shipped ball inertia 0.002 at 0.01 kg is 78× a solid sphere (the m100 copy uses
   2.56e-4).
16. **Tejen merge.** Fork base a7524bae; merge his a7524bae..HEAD (path `drone_communication` → `tejen_mission`)
   in one go after his state-machine renumbering, re-apply `authority.md` §6, then his tests + one M2 + one M1.
   d9c6941 (IRL gate profiles) checked read-only: sim-neutral (parameter renames at equal values;
   `m2b_ring_pose_timeout_s` 0.25 → 0.50 already set by our M2D launch); overrides of the renamed parameters
   would stop reaching the gates (none today).
17. **Owed:** Wesley's GUI watch of M2 (`drive_m2_handover.py LOGDIR true 20`); the real rod's twist freedom vs
   the sim universal joints; lab workstation Wed 30 Sep 2026 (remote headless runs over Tailscale + SSH,
   NoMachine or Sunshine for the GUI; no Docker for now; Wesley brings lscpu and the Tailscale name).

---

### 7b. 2026-09-24 evening, plan "six days to the lab" (plan file: sparkling-sprouting-possum)
- **Sim throttle linearised** (decision above): `payload_betaflight_comm.py` (the per-drone node every OCP launch runs; the first edit missed it and Wesley's 20:12 pair sat on the floor at throttle 0.188), `betaflight_communication.py` /
  `angle_betaflight_communication.py` command → motor speed = 4631·sqrt(u); SIL plant
  linear; `thrust_ratio:=auto` = 88.6. The +10 % / +15 % typed-gain configs are 97.46 /
  101.89 now. All kt_trim numbers in §4.10 and the card were flown on the quadratic plant.
- **Creep floor start** (R0466 n3 lifted, R0468 n4 aborted): on the quadratic plant the
  drones never followed the arc (free hover at 0.33 throttle while the loaded secant said
  +2 m/s²), handover on the 3 s timeout at ~12°, OCP lifted from flat rods. Re-fly both on
  the linear plant before the rig. Rig command in tests.txt A-real.
- **Detach LAND fix v2** built, critic-revised, Gazebo proof pending
  (`docs/experiments/2026-09-24_detach_land_fix.md`): feedforward off once the load is
  down during LAND (the sideways push that tipped R0113's drone 0 and R0276–R0279),
  survivors-only TouchdownDetector, departed drones descend at 2× and are judged by
  stall, magnets re-arm at ARM, detach refused during LAND. `real_dissipative_launch.py`
  has `reconfig_mode` / `detach_magnet`; `real_io_launch.py detach:=true`.
- **Drone mass 0.64** (decision above): `DRONE_MASS` in params.py, `drone_mass` planner
  parameter and launch arg; 'auto' gain 83.1; SIL plant 0.64 / 83.1. The rig must pass the
  weighed airframe-with-pack mass; the kT 24 measurement is per-airframe and unaffected.
- **6S constants everywhere** (preflight 22.8 V bar, launches 25.2/21.0, warning 22.5,
  panel bands 23.4/21.6/19.8).
- **Launch defaults changed 2026-09-24 22:30 (Wesley's word):** kt_trim on, z_ki 0.4,
  creep floor start (start_taut false, handover 45°, settle 1.0 s, creep_vel 0.2) on the
  six floor-start launches; existing configs pinned to their old semantics. Rig command
  in tests.txt is now just mass / rod / kT / ramp / traj. The planner height integral
  (card 2026-09-24_planner_offset): SIL 0.600 in every static-offset arm both ways, inert
  in the circle, rails+WARN under a gain fault, no upward authority against a typed-high
  gain (the trim's job); Gazebo W-22:16 0.644 → 0.600 in 12 s, W-22:26 with 0.4 and the
  fast creep reaches target 2 s sooner. Headless pair still owed for the claim.
- **kt_trim v3** (filtered inputs, convergence freeze, applied once airborne, 10 cm
  latch, floor test only while the load rests): SIL R0465 0.607; two-drone Gazebo pair
  = Wesley's runs. Two-drone v2 crashed (16:28) on a 2 Hz estimator–tracker chatter.

- **Sim rate loop on the gyro by default (2026-09-28, Wesley):** every sim Betaflight bridge
  (`payload_betaflight_comm`, `tejen_betaflight_communication`) steps on the X3 IMU
  (`rate_source imu`) instead of differenced poses; every Gazebo baseline shifts once (re-fly,
  record the shift). `run_experiment.py` and the gz-starting launches refuse a world without the
  Imu system (`simulation_communication/imu_world.py`); `m2_bench.yaml` flies `m2_bench_world_imu.sdf`.

## 8. Plan to submission

Results freeze Fri 7 Nov 2026. Submit Thu 27 Nov 2026.

| window | milestone |
|---|---|
| Sep 15–26 | attach hand-out and layout decision; hardware H0 bring-up and H1 two-drone carry |
| Sep 29–Oct 17 | hardware carry and detach campaign; attach matrix in sim |
| Oct 20–Nov 7 | hardware attach window, gap-filling, demo capture; freeze |
| Nov 10–21 | full draft, supervisor review, revisions |
| Nov 24–27 | final revisions, submit |

Rig access about 2 days a week. The lit review is drafted in LaTeX outside the repo.
Chapters are written the week their method freezes.

Descope ladder, in order, if the clock wins: drop the failure-redistribution study; drop
scalability beyond n=2/4 spot checks; fig-8 only where already flown; one perturbation per
axis in robustness; hardware attach steps down moving weld → hover weld → approach without
weld → sim-only attach. The floor: hardware carry and detach, the sim attach boundary,
the architecture study, honest sim-to-real accounting.

---

## 9. Hardware

Launches: `real_io_launch.py` (mocap + ELRS + RViz, terminal 1, must be up first),
`real_control_launch.py` (OCP), `real_dissipative_launch.py`, `real_attach_launch.py`
(twin of the sim attach launch, same 69 arguments; `tools/param_diff.py --sim-vs-real`
shows only the intended deltas). Desk test without the rig: `tools/fake_mocap.py`.
Pre-arm checklist: `tools/preflight.py --real`. Rig checklist for attach:
`docs/experimentation/real_attach_gap.md`.

Rig facts (updated 2026-09-29): our drones 0-3 = Tejen's quad1-4 = `/dev/QUAD1..4` (udev: four unique serials) =
Motive bodies 11-14 (`mocap_drone_body_ids`, typed on the rig launch), payload body 8 (code default; tests.txt types 9:
confirm in Motive, Q3); the magnet tip
needs its own rigid body; `thrust_ratio:=24`; floor start on the CREEP path,
`start_taut:=false handover_elev_deg:=45 handover_settle_s:=2.0` (the launch default
`start_taut:=true` runs the OCP from tick 1, whose references sit 18–22 cm inward of a
resting drone with flat rods: the 2026-09-23 13:13 non-lift; the arc creep anchors each
drone where it rests and sweeps the rod to 45° first; plan 2026-09-24);
`load_mass:=<weighed> cable_len:=<measured>` every session (the real launches default load_mass to 0.86, the ring).
ELRS adapters: `real_io_launch.py` still defaults drone i to `/dev/QUAD<i>` while the map is QUAD(i+1), so pass
`drone0_serial:=/dev/QUAD1 ... drone3_serial:=/dev/QUAD4` (the M1/M2 `real:=true` modes map it themselves).
`tools/preflight.py --real --planner load_planner` for the OCP launch (the default
planner name is the dissipative node's); `--max-ground-z 0.5` when the drones sit on taut
rods before takeoff. Since 2026-09-16 `real_dissipative_launch.py` takes `control_mode`,
`vel_*`, `auto_network_handover` and `diss_ki_load` like the sim launch (defaults unchanged:
MPC tracker, network only on a detach), so the Part A configuration can be flown on the rig. Tether magnets: each drone's
`elrs_interface` latches a magnet value into aux channel `magnet_channel` (6 = AUX4 on every launch; `/drone_<i>/magnet` String
ON|OFF; `rio magnet_initial:=ON` holds them from boot), and the RViz panel shows one
MAGNET toggle per drone on the real launch. The tethered airframes' magnet mode is on `channel_6` (Betaflight AUX4), found with
`tools/aux_sweep.py` on 2026-09-16 (confirmed on drone 0 only: check the others at R0b), and is the launch default since
2026-09-16; one magnet path per drone through the radio latch since 6b8e01c.

Test order, earned in August: hold (`lift_ramp_vel:=0.0`) to isolate thrust, then hover
at `lift_ramp_vel:=0.15`, then shuffled and yawed starting placements (the load yaw datum
must print in the planner log), then line, circle, fig-8, spin at 0.2 m/s, then detach,
then attach last.

Real-world bugs already fixed (2026-08-03): slot assignment in the load frame (yawed
payload made the QP infeasible), latched payload yaw datum, latched per-drone heading (a
drone did a 360° on takeoff), trackers honour `use_sim_time`.

Raw commands without RViz:

```bash
ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: ARM}"      # ARM|TAKEOFF|LAND|DISARM|ESTOP
ros2 topic pub -t 3 /fleet/detach  std_msgs/msg/Int32  "{data: 1}"
ros2 topic pub -t 3 /magnet/command std_msgs/msg/String "{data: ON}"      # = the RViz ATTACH button
```

Shell helpers (in `~/.bashrc`): `mdc` sources both setups; `cb [pkg]` builds; `rio`,
`rctl`, `rdiss` are the real launches; `sgz <world>`, `sviz`, `smpc`, `sdo`, `sdet`,
`satt` are the sim ones; `simcheck <world> <launch>` runs the geometry check.

---

## 10. Traps and hard-won lessons

- **Read the parameters back.** Launch args have silently failed to plumb through three
  times (net_traj_lean, control_mode on the newcomer, a cut command line). Every run
  writes `params/` by read-back; check it before believing a comparison.
- **Same wall-clock duration or no comparison.** Trackers once ran on wall time while the
  planner ran on sim time; RTF then set the control rate.
- **`tools/prebuild_planner.py` after touching the planner model, masses or inertia.** The
  SIL bench starts its clock before the solver builds; an interrupted build deletes the
  old `.so` and never writes the signature, which presents as "the planner publishes no
  references".
- **The sim's CPU goes to the ROS side, not to physics.** Gazebo alone runs the ring
  world at RTF 1.0; with the controller stack up it sat at 0.37 idle / 0.29 in flight
  (12-thread laptop, 2026-09-23). Causes: Gazebo's /clock at 700–1000 Hz handled in a
  PYTHON callback by every node, the emulators republishing every Gazebo pose
  (300–500 Hz) into every tracker, and fleet_viz redrawing per message.
  `rviz_quad_load_launch.py` defaults clock_hz 100, mocap_hz 120, viz_hz 30 since 2026-09-23 (clock via
  `clock_throttle`, publisher RELIABLE or the runner sees no sim time) gives 0.60 idle /
  0.45 in flight with the same hover to 1 mm (R0329 vs R0330). Still unexplained: each
  tracker burns ~0.9 core while DISARMED. Simplifying drone, rod or ring models does
  nothing (measured).
- **Payload contact geometry is one cylinder.** The ring's 24 box collisions, each
  touching the floor, took Gazebo's RTF from ~30 % to 7 % (2026-09-23); the ring and
  plates are visuals only.
- **acados refuses a cached solver whose `parameter_values` differ** ("OCP formulation
  has changed"), even when the signature matches; a run that hits this compiles mid-run
  and is void (R0297–R0301). `planner_ocp.py` now seeds canonical parameter values so the
  cache is geometry-independent.
- **`metrics.steady_window` ends at LAND.** Before 2026-09-23 it ran to the end of the
  record and averaged the descent into every "settled" height (0.598 reported for a
  0.634 hover). Recompute with `tools/plot_run.py --metrics` before comparing to old rows.
- **Start Gazebo from inside `simulation_assets/`** (relative model paths) and **RViz
  launch first** (it owns the pose bridges and mocap emulators).
- **`tools/clean_slate.sh` between runs.** Leaked nodes and Fast-DDS shared memory fake
  divergence.
- **Wall-clock watchdogs in a 0.3× sim** (pose timeout, reference staleness) fire on
  healthy fleets under CPU load. Sim launches carry longer budgets and headless Gazebo
  runs niced. Do not run analysis while a batch flies.
- **`dissipative_launch.py` never engages the network** unless a detach or attach fires;
  `dissipative_only_launch.py` does.
- **The offline harness and the SIL bench are necessary, not sufficient.** PD proxies
  settle cases Gazebo capsizes; the bench has no rotor vibration and an exact plant.
- **The tracker has no integrator and trusts the cable feedforward.** Any transient where
  the real cable force exceeds the modelled one under-thrusts the drone. Fixes are
  reference-side (keep actual ≈ modelled) or the common-mode trim.
- **Physics that tuning cannot remove:** ~13 m/s² of thrust authority after gravity; three
  fixed tethers cannot form an even 4-gon under equal shares; a centre weld cannot be a
  ring member; a disc on two rim cables is a pendulum.
- **Circle is one eased sweep, not laps.** Trajectory metrics use `metrics.sweep_window`;
  a whole-run radius ratio is meaningless.
- **A pre-takeoff disarm grounds the fleet and refuses TAKEOFF** since 189b6c6 (was open problem 6; R0749c, D0001, D0003).
- **The sim rate loop is PI by default since 2026-09-26** (`rate_ki` 10, clamp 200,
  integrates only armed and u > 0.09; all four sim bridges via `rate_pid.RatePid`).
  `SIM_RATE_KI=0` gives the old P-only loop (the m2d launch honours it; `M2_RATE_KI`
  overrides it there). Gazebo runs before this date were P-only: compare only against
  baselines flown with the same `rate_ki`. The SIL plant (`tools/sil/plant.py`) has no rate PID, so SIL and Gazebo now
  differ in the inner loop. [Superseded 2026-09-27: default `rate_ki` 5 on every bridge, Tejen's included, env
  fallback '5.0' (a4c748e); 10 limit-cycles (R0647, T0017). Runs R0647–R0650 and T0016/T0017 are the ki sweep.]
- **Never throttle /clock in Tejen's world** (breaks his velocity estimates, W-20:56); never pause the world
  mid-flight. His per-drone nodes dominate M2 CPU (planners 163 %, MPCs 128 %).
- **Difference poses over sim stamps, not the wall or node clock.** The magnet manager's weld speed gate read
  4× low at RTF 0.25 and welded mid-descent (R0668).
- **Never run M1 with /clock unthrottled** (clock_hz 0): ~180 carrier rate kicks and speed faults (R0702/R0703).
- **gz CLI one-shots drop** (the ball stayed welded after DROP, R0678): use ros_gz bridges, or send 3× (the
  runner's GZ_PUB does).
- **DART ignores `<axis><damping>` on ball joints** (S0005–S0008): every ball-joint damping in our models is a
  no-op; a universal joint takes it (S0009/S0010 settle a swing in 0.6 s).
- **Partner runs are void without the start weld**: check 'drone 3 welded at start: plate at 90'
  (`m1.start_weld_ok`); R0676 lost a start-up race and first read as a falsification.
- **The partner model's pickup DetachableJoint** holds the tip to the ball until his mission sends
  `/pickup/detach`; a bench without his mission uses a `_nopick` copy or GZ_PUB it (R0654–R0659 void).
- **Runner events are time-sorted**: a WAIT placed before the event it waits for blocks forever (R0671).

---

## 11. How to run and verify

```bash
source ~/ros2_humble/install/setup.bash && source install/setup.bash   # or: mdc
colcon build --symlink-install                                          # or: cb

./tools/gate.sh --quick        # pytest, imports, geometry (seconds)
./tools/gate.sh                # + dissipative A–K + SIL smoke (~2.5 min)
python3 tools/prebuild_planner.py
python3 tools/sil_bench.py configs/sil/carry_hover_n3.yaml
python3 tools/run_experiment.py configs/experiments/attach_circle_n3.yaml --repeats 2
tools/plot_run.py R0234 ; tools/compare_runs.py R0234 R0243 --label timed settle
```

Verification ladder, cheapest first: pytest → gate quick → SIL bench → headless Gazebo
(~4 min a run) → rig. A claim is never promoted above the rung that produced it. Every
experiment starts from a card (`docs/experiments/<date>_<name>.md`), one variable, a
named baseline, a falsifier, two repeats, one registry row per run.

M1, M2 and the weld benches (§4.11):

```bash
python3 tools/run_experiment.py configs/experiments/partner_attached_orbit.yaml   # M1, 11-13 min wall
tools/clean_slate.sh; python3 tools/sim_test/drive_m2_handover.py results/<date>/T00NN false 20   # M2, ~21 min
python3 tools/run_experiment.py configs/experiments/m1_rejoin_orbit_fix_pick.yaml # rejoin bench, ~4-5 min
python3 tools/metrics.py run|m2 <run_dir>   # re-score: writes <run_dir>/metrics.json (M1 stages / summarise_m2)
```

Tools added 2026-09-26/27:
- `tools/metrics.py`: `event_window_metrics` (per event: peak ring and drone tilt, carriers' dip, time under 2°,
  ring z error; window to the next disturbing event or +15 s), `m1_stage_metrics` (every M1 stage, ball in the
  net in the ring frame, reattach retries, `start_weld_ok`), `summarise_m2` (join, fallbacks, faults, ring z and
  tilt per segment, drone tilt, landed; CLI writes `metrics.json`), `partner_clearance`; unit tests on R0646 and
  T0015 fixtures.
- Runner (`tools/experiment/`): logs PHASE/HANDOFF/PICKUP/DROP/FOLD_IN/PARTNER_* at sim time, the ball pose, a
  `wall` column, `wall_s` and `mean_rtf`; events GZ_PUB (gz topic, sent 3×), HANDOFF, WAIT_PARTNER_RELEASE,
  HANGER_RELEASE; `timing.stop_launch_on_handoff` (SIGINT his launch at the handoff).
- Weld benches (M1 without his mission): `m1_rejoin_hover*` (hover), `m1_rejoin_orbit_fix*` (orbit; `_pick` the
  full model with GZ_PUB `/pickup/detach`, `_late` detach 70 s later), `m1_rejoin_orbit_held` / `_moving`,
  `m1_zki_late_off` / `_on`.
- `tools/sim_test/stage1_torque_test.py` (one X3: body torque and tether swing, S-rows).

The demo by hand:

```bash
cd simulation_assets && gz sim three_attach_rod.sdf -v4 -r
ros2 launch controller_quad_load rviz_quad_load_launch.py num_drones:=3 attach:=true
ros2 launch controller_quad_load three_attach_launch.py load_traj:=circle traj_speed:=0.2
#   ARM -> TAKEOFF -> ATTACH
```

---

## 12. Where things live

| path | what |
|---|---|
| `src/controller_load_mpc` | OCP planner, geometry, trajectories, `params.py` |
| `src/controller_quad_load` | tracker (`controller_mpc.py`, `velocity_loop.py`, `thrust_model.py`), fleet manager, **all launch files** |
| `src/controller_dissipative` | network, node, offline harness |
| `src/drone_magnet`, `src/controller_mpc_payload` | attach chain and the approach MPC |
| `src/simulation_communication`, `src/drone_communication` | sim bridges; real mocap and ELRS |
| `simulation_assets/generate_rigid_world.py` | source of truth for every world; `tools/make_weld_variants.py` for the rod/seg2/rigid worlds |
| `configs/experiments/*.yaml`, `configs/sil/*.yaml` | every flown configuration |
| `tools/` | gate, runner, bench, metrics, plots, preflight, geometry and parameter checks |
| `results/registry.csv`, `docs/decisions.md` | what has been tried; what was decided |
| `docs/design/` | velocity loop, measured-force loop, SIL bench, fleet safety |
| `docs/experimentation/` | control-methods survey, hybrid dwell, robustness table, rig gap, agenda |
| `results_archive/` | tracked data behind thesis figures; `configs/thesis_figures.yaml` + `tools/thesis_figures.py` regenerate them |
| `src/tejen_mission`, `src/tejen_mpc`, `src/tejen_dynamic_planner`, `src/tejen_utility_objects` | Tejen's forked code (base a7524bae); our changes listed in `authority.md` §6 |
| `simulation_assets/tejen/`, `simulation_assets/three_attach_partner*.sdf`, `simulation_assets/models/x3_drone3_magnet_rod_partner*.sdf` | his worlds; M1 worlds (incl. `_m100` 0.1 kg object, `_nopick` bench copies); the partner rod models |
| `src/simulation_communication/.../rate_pid.py` | the sim rate PI shared by every sim bridge |
| `tools/sim_test/` | M2 driver `drive_m2_handover.py`, stage-1/2 single-drone benches, his runner scripts |
| `docs/GOALS.md`, `authority.md` | the integration loop's plan and status; Tejen integration notes |
