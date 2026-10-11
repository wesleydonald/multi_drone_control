# Thesis direction and document plan (11 Oct; workflow wf_d29adf9f-262: 4 readers, 3 directions, 3 judges, document plan, 3 critics)

# Final thesis plan, 11 Oct 2026

This plan merges the 11 Oct direction, the document plan and the three critiques. I checked the critics' factual claims against the repository before accepting them. Six of those checks changed the plan:

- **r0013 (7 Oct capsize).** The drop-trigger replay (`docs/experiments/2026-10-08_drop_trigger_replay.md:28`) shows a survivor (`/drone_1`, rig plate 11) came off its plate with no command 0.155 s after the release, at a ring tilt of only 2.6°. A second survivor came off at +0.386 s. So the capsize is not "a step tipped the ring". It is "a step, then a survivor's magnet let go".
- **r200006 (7 Oct measured-pull circle).** It lost two magnets with no command. The first came at 0.9° of tilt and the ring recovered from 30.6°. The second capsized the ring at 61°. Wesley's note in the registry: the magnets hold about 9–10 N, and the release came from a sharp jerk.
- **The W4ff config is already a one-variable pair of W4.** On week4, `common.yaml` sets `drop_on_loss`, `departed_land`, `departed_ground_idle` and `detach_detect_m 0.06`. So `detach_unannounced_ff.yaml` differs from W4 only in `cable_source: 'model'`.
- **Plate numbering.** The rig sheets number plates clockwise ("old 1/3/5/9 = 11/9/7/3", `rig_2026-10-08_commands.md:3`). The direction's mapping (thesis 1/3/5/9 = rig 3/11/7/9) is wrong.
- **The rig join card differs from the twin.** The rig proof is a lift of up to 3 cm after about 1 cm of slack, with up to 2 cm of plate rise allowed. The twin proof is a 1 cm lift on a rigid tether. The rig card's ring bar is ≤ 5° absolute; the direction said ≤ 10°.
- **Due date.** The title page says 20/11/26. Every plan says 27 Nov.

---

## 1. Verdict

**The thesis in one sentence:** a team carrying a ring can lose or take on a quadcopter in flight as an observed, owned switch between pre-built optimal-control problems. On hardware, the size of the ring's transient is set by where the carriers learn of the change (from the plan or from their own measured rod pull) and by the support margin the leaver leaves behind. The ring is lost only when a magnet lets go.

**Why it is strong:**
- It rests on hardware: six commanded releases and three uncommanded ones, all in the record.
- It has a mechanism that predicts signs before the runs: a one-degree-of-freedom model of feedforward against feedback.
- It has a real-life finding the twin structurally cannot produce: the magnet's hold is the binding constraint.
- It has a join that no prior work has: admitting a drone flown by another student's controller to a payload a team is already carrying.
- It is honest about its simulator through a prediction ledger.

That is the rubric's top band for execution, "something unexpected, thoughtful and original, such as a novel perspective" (50 % weight). The leave and ledger chapters together are RA-L-paper-shaped, which is the top band for conclusions (20 %).

---

## 2. Thesis statement, research questions, contributions

### 2.1 Title
**Mid-Flight Reconfiguration for Cooperative Quadcopter Payload Transport: Leaving and Joining a Carried Ring.** The long control-question subtitle promised "what sets the transient when a quadcopter joins", which the join chapter does not answer. The control question now lives in the thesis statement. This is a decision for Wesley (§10, D2).

### 2.2 Thesis statement
1. A centralised optimal controller can let a quadcopter leave or join a team carrying a 0.86 kg ring in flight. The event stays bounded when three conditions hold:
   - team size and attachment geometry are runtime parameters, so each change switches to a pre-built problem;
   - every change is triggered by an observation: a stretched link for a loss, and for a capture, contact plus a lift proven from motion capture;
   - every drone has exactly one owner and a tested way down.
   On the rig, "tested" holds only for the exits flown there; §3.5 says which.
2. On hardware the ring was lost only when a magnet let go. Every rig capsize began with a magnet release that nobody commanded, and all of them came with the rods planned at 45°.
3. The transient the survivors must ride out is set by two things:
   - **Where they learn of the change.** The plan is fast but only as right as the model, and its step can tear a magnet off. The measured rod pull needs no model, comes late, and works when nobody announced the release.
   - **The support margin** left by the leaver.
4. On the measured pull, resizing does not lower the peak. In the twin, the earlier the resize comes, the higher the peak. What knowing about the release buys is the state after it: shared load, a level hover, a clean landing, a freed drone that clears the team.
5. The rig-calibrated twin predicts the reactive release well. It misses what it does not model: the magnet's hold, rotor wake and loop latency. The prediction ledger says by how much.

### 2.3 Research questions

| RQ | Question | Rig or twin | Replaces |
|---|---|---|---|
| **RQ1 Carry** | What does feeding the trackers the rod pull measured from motion capture buy on unmodified Betaflight firmware, and which error sources does the rig expose that the twin does not? | rig first | Q1 + Q2 |
| **RQ2 Leave** | When a carrier leaves in flight, what sets the ring's transient, and what keeps the ring from being lost? | rig first, twin for the levers | RQ1a |
| **RQ3 Join** | What must be observed before a quadcopter flown by another controller is admitted to a carried ring, and what does admission cost the ring? | twin claim, rig R11b | RQ1b |
| (§4.4, not an RQ unless Wesley wants one) | What the twin predicted: the prediction ledger, from by-product twin previews only | both | new |

The old RQ2 (planner against network) becomes a scope paragraph plus Appendix F, labelled as the earlier plant.

### 2.4 Contributions
There are three claims and two method items. The switch statistics and the fault matrix are validation, not contributions.

**C1. Carrying with a measured rod pull on unmodified Betaflight firmware (RQ1).**
- *Claim:*
  - One affine thrust map fits all four airframes: u = a_i + 0.507 M − 0.022 (V − 23.5), with a_i = 0.181–0.187 and rms 0.0032 over 13 hovers.
  - On that map alone, with no integrator, the drones hold free hover to +3.7 / −0.5 / −0.4 / +0.9 cm.
  - Feeding the trackers the measured rod pull, **together with planning the rods at 60°**, took the four-drone circle at 0.125 m/s from 8.25 to 2.91 cm mean path error. Lag fell from 0.45 to 0.20 s and ring height sd from 4.8 to 0.5 cm.
  - The figure-eight **at 0.25 m/s** went from 8.2 to 3.0 cm.
- *Still needed:*
  - Thu 15: the 60° planned-pull circle.
  - One repeat of the measured-pull circle.
  - A hover of at least 30 s. This is folded into the 3-min ring hover (D13 of the M1 plan).
- *Falsifier, frozen before Thu 15:* if the 60° planned-pull circle is within max(1 cm, the lap-to-lap sd) of 2.91 cm, the gain came from the rod angle, not the measurement.
- *Also reported:*
  - About 0.55 N per drone acts inward and not along the rod. Rotor wake is a hypothesis; the alternatives are rod mass (0.075 kg), marker offset and rate rounding. It gets tested on twin logs.
  - Hover drift toward −x.
  - Lag of 0.20 s on the rig against 0.04–0.11 s in the twin.
  - What the N−1 layout costs the carry: the 1/3/5/9 hover r011 against the even ring.

**C2. The leave transient on hardware: the full record and the levers that set it (RQ2).**
- *A. Hardware record (exists; table T8).*
  - Six commanded releases. The ring was carried on through five, with peak tilt 4.6–17.8°. Unannounced releases were detected in 0.33–0.42 s.
  - Three uncommanded magnet releases in two flights:
    - r0013: a survivor at +0.155 s, at 2.6° of tilt;
    - r200006: two releases.
  - Every ring loss began with a magnet letting go. Pending: the slip detector replayed on the 8 Oct bags.
  - Prior hardware work does not report the peak transient of a true release from a team-carried payload. Hedged: Bacheti 2026 and Delbene were read from abstracts only; read the full texts first.
- *B. Levers, each a one-variable pair with a frozen sign and the resolution rule of §4/3.6:*
  - **Where the survivors learn of the change (the crossover).** Frozen prediction: the planned pull gives a lower Δθ when the change is announced and the model is right. The measured pull gives a lower Δθ when the change is unannounced or the model is wrong.
    - Twin today, announced, measured against planned: 8.4 / 3.3° at 120° and 16.4 / 4.1° at 150° (n = 1 each, 7–8 Oct trees).
    - Rig r0012 (planned, 45° plan, reference integral: 4.6°) and r302_3 (measured, 60° plan: 15.4°) differ in five things. They are two configurations, not a pair.
  - **Ramp (soft v2).** Planned pull: 3.3 → 2.5 / 2.6° at 120° and 4.1 → 3.3 / 3.5° at 150°. v1 is the negative that defined v2 (4.7°, 5.6°). On tilt the effect is about 0.8°, probably below the rig's minimum detectable effect. Its rig value is tested on the measured pull and on the magnet margin.
  - **When the resize comes**, measured pull, 120°, twin: never 6.7° (n = 1) < at detection 7.5 / 7.9° < at the release 8.4° (n = 1). The rig matches the order: unannounced 6.5° against announced 15.4°, n = 1 each. The hypothesis is a double count of the load change at the resize tick, tested offline from the logs.
  - **Support margin** m = r·cos(g_max/2) on the measured pull: 112 mm (120°) gives 7.5–7.9° in the twin and 6.5° on the rig; 58 mm (150°) gives 13.9–19.2° in the twin and 17.8° on the rig.
  - **Magnet margin (new safety metric).** The survivors' peak measured rod pull, and how fast it rises, against the hold at that rod angle.
  - **Integral mode**, twin only: reference 7.5–7.9° against model 3.6–5.0°. This crosses trees and is probably the pre-event state, so the pre-event offset, tilt and leaver pull are reported with every arm.
- *C. What knowing about the release buys.*
  - Detection off (R1144): peak 6.7°, landing lean 5.4°, the freed drone ends 15.6 cm outside the rim.
  - Detection on: 7.5–7.9°, 2.0°, and it clears.
  - With three survivors, statics fixes the vertical shares (three equations, §3.4). The OCP is credited only with re-placing the survivors (rod directions, horizontal balance), measured from the logs.

**C3. An admission contract for a drone flown by another controller (RQ3).**
- *The contract.* The other controller must:
  - descend to contact;
  - lift by a set amount from the latched contact height;
  - hold;
  - give up authority on proof, or give up.
- *What we do:*
  - observe contact (the tip on the plate, moving with the ring) and the lift (drift from contact, taut length, plate rise) from motion capture only;
  - switch the mux on the same tick;
  - fold the newcomer in from 0.1 N;
  - take it back if nothing is proven.
- *Counterfactuals (what each of our parts is worth):*
  - Fold-in seed: started at the OCP share, 7.7–8.6° (4 of 4 over 5°); started at 0.1 N, 1.8–2.6°. On one tree that is n = 2 against 2.
  - Proof: QUAD4's mocap force changes by 0.00 to −0.06 N during the lift, against rig noise of 0.11–0.35 N sd. A force proof is infeasible, so the proof is geometric.
  - Hand-over step ≤ 0.5 % of hover.
  - Take-back: hover passes; the circle failed three ways.
- *Twin claim:* the M1 joint claim batch on the rig candidate tree (Fri 16), then the reviewer. Today's numbers on c6f2032:
  - 8 of 8 catches proven, 0.58–0.97 s after contact;
  - ring tilt 2.07–2.81°;
  - 0 false admissions.
- *Rig:*
  - the proof logic replayed over rig bags;
  - R11a as the null;
  - R11b ×2, then to n ≥ 3, scored on Δθ.

**Method M1. Reconfiguration and authority.**
- Notice, re-plan, redistribute, fall back.
- Geometry as a runtime parameter, with one pre-built solver per team size, primed twice before it flies.
- The fall-backs, framed as runtime assurance (Simplex):
  - take-back;
  - tilt cut;
  - LAND for every drone;
  - landing the fleet on a newcomer fault;
  - dropping the ring on a second loss or a tilt over 30°;
  - the freed drone disarmed alone.
- Validation: the switch table and the fault coverage, per tree.

**Method M2. The claim protocol and the prediction ledger** (§3.6 and §4.4).

---

## 3. The headline result and the headline figure

**Headline result (rig, 8 Oct, r305).**
- During a 0.125 m/s circle, a carrier's magnet was switched off without telling the planner.
- The rod stretch revealed the release 0.34 s later. The twin took 0.21–0.27 s; the rig is later by about one 10 Hz planner tick plus the magnet's release.
- The planner switched to the three-drone problem, and the survivors carried the ring on round the circle.
- Peak tilt was 8.1°, *above* the twin's 6.0–7.3°.
- The freed drone cleared to 1.50 m from the path centre.
- **Wording that must not change:** it did not land by itself. Its touchdown detector stalled at 0.19–0.21 m and it fell about 0.3 m at the fleet disarm, on the old code.
- **Check before quoting a dip:** the registry's "3.9" for r305 is the along-track RMSE, not a dip. Reconcile the 8 Oct dip range (3.7–6.9 cm) from the logs.

**Upgrade path:**
1. W5 on Wed 14: the same flight with the model integral and the freed drone disarmed alone.
2. R11b on the rig inside its pre-frozen band: the first capture of a drone onto a payload a team is already carrying.
3. The full M1 on the rig on 4–5 Nov, if R11b passed.

**Headline figure, F10: "Releases on the rig and what set them".** Full width, 6.27 × about 4.0 in, Computer Modern 9/8 pt, built through the thesis-figure loop (render-audit-look ≤ 3 rounds, `figure_audit.py`, blind critic).
- **Panel (a), the hardware record (rig primary).**
  - One row per rig release event: 6 commanded, plus r200006's 2 uncommanded.
  - x = Δθ (°) on a broken axis (0–20 | 30–90), so both capsizes show with value labels.
  - Rig events as filled markers: colour = pull source; shape = announced, unannounced or uncommanded.
  - A magnet slip gets a distinct marker.
  - The twin's prediction is an open range bar on the same row, or "no twin pair".
  - Rows are labelled with conditions (pull source, rod plan, gap, announced or not), never run ids.
- **Panel (b), the crossover.**
  - x = {announced (ramped), unannounced}; y = Δθ; one line per pull source.
  - Twin runs as open dots with range bars and n; rig runs as filled markers.
  - The model-error arm as a third series if flown.
- No ✓/✗ glyphs; filled against open markers carry that meaning.
- Script in `tools/`, output in `docs/figures/F_releases/`, run ids only in `configs/thesis_figures.yaml` and `PROVENANCE.md`.

---

## 4. Chapter-by-chapter plan (main text 49 pp, 1 pp slack; Wesley cuts lines later)

| Chapter | pp | Sections (pp) |
|---|---|---|
| 1 Introduction | 3.0 | 1.1 Why a team changes in flight (0.75) · F1 storyboard (0.6) · 1.2 Roles, T1 (0.4) · 1.3 RQs, T2 (0.6) · 1.4 Contributions (0.5) · 1.5 Outline (0.15) |
| 2 Literature | 8.5 | 2.1 Transport and link models (1.5) · 2.2 Membership change, T3 (2.5) · 2.3 Capture proof, docking, magnetic grasping (1.0) · 2.4 Switching, bumpless transfer, runtime assurance (1.25) · 2.5 Execution layer and force estimation (0.75) · 2.6 Sim-to-real (0.5) · 2.7 Gap (0.5) |
| 3 System and method | 12.0 | 3.1 Rig, twin, roles (2.0) · 3.2 Model and planner (2.5) · 3.3 Tracker, thrust map, measured pull, model integral, takeoff/landing (2.0) · 3.4 Reconfiguration (3.0) · 3.5 Authority and safety (1.5) · 3.6 Method and metrics (1.0) |
| 4.1 Carry (RQ1) | 3.5 | |
| 4.2 Leave (RQ2) | 8.0 | |
| 4.3 Join (RQ3) | 6.5 | |
| 4.4 What the twin predicted | 2.5 | |
| 4.5 Discussion | 3.5 | |
| 5 Conclusion | 1.5 | |

The report guide's per-section lengths are indicative; only the 50 pages are a limit. Tell the supervisor that 8.5 pp of literature against the indicative ~15 pp is a deliberate split.

### 1 Introduction
- **1.1 Why a team changes in flight.**
  - Motivation: battery rotation, re-tasking, a member that must leave.
  - The releases here are commanded or magnet-off releases. A dead carrier left hanging on the ring is out of scope and left to the 30° drop rule.
  - The gap, stated narrowly:
    - no work attaches a drone to a payload *already carried by a team*;
    - no hardware work we read in full reports the peak transient of a true release.
  - Lab speeds (0.125–0.25 m/s).
- **F1, the full-M1 storyboard.** Six 3-D frames rebuilt from the logged poses of a frozen-tree full M1: welded carry, release, his pickup and drop, contact and lift, fold-in, LAND. An owner strip under each frame, and rig photo insets wherever a phase has flown.
- **1.2 T1 roles.** One row per phase, rig and twin:
  - Tejen's controller: pickup, transit, drop, descent to contact, the proof lift.
  - Ours: carry, detection, confirmation, mux, fold-in, take-back, fault policy.
  - Our code inside his stack (LIFT_PROOF, RING_ATTACHED, TAKEN_BACK, lift-from-contact, the pickup retry, magnet OFF in TAKEN_BACK, the drop check) is listed as ours, "with his written agreement".
- **1.3 RQs** with T2: question → experiments → rig or twin → section.
- **1.4 Contributions:** C1–C3 plus M1–M2, one testable sentence each.

### 2 Literature
- **2.1** Sun 2025 as the benchmark; their planner time grows with team size, and they do not ablate the INDI force estimate. Also Wahba; Lee, Sreenath & Kumar 2013; Sreenath & Kumar 2013; Six 2018 rigid links (λ ≥ 0 is a modelling choice); Liu…Quan 2026 TNSE in place of the arXiv preprint, plus one paragraph on why the network is not an RQ.
- **2.2 T3, numeric membership-change table.** Columns:
  - event;
  - mechanism (slack, motors off, severance, release, capture);
  - announced or not;
  - **leaver removed or left hanging**;
  - sim or hardware;
  - team sizes;
  - peak tilt or sag;
  - **detection time, kept apart from recovery time** (Hajieghrary's 0.32 s is a recovery time).

  Rows: Bacheti, Zeng, Quan, Oishi, Gao, Liang, Delbene, Hajieghrary, Shankar; ModQuad, TRADY and LEGION marked "no payload"; this work. Also plug-and-play MPC and Farina 2020, and statics (Michael 2011, Fink 2011, Tognon 2018).
- **2.3** Proof from a sensor in the gripper (Loianno 2018, Shankar 2020) against proof from pose or wrench (ModQuad, LEGION). TRADY's thrust blend. Magnetic grasping, where hold falls with peel angle (Gawel 2017).
- **2.4** Graebe & Ahlén; Peng et al.; Simplex (Sha 2001); RTI (Diehl 2005; Gros 2020); open multi-agent systems.
- **2.5** Tagliabue 2019 (UKF force estimate), Sun 2025 (INDI), offset-free MPC (Pannocchia & Rawlings 2003; Maeder 2009), commercial flight-controller precedent.
- **2.6** RotorTM's per-axis format, RotorS, Cruz & Fierro.
- **2.7 Gap:** three sentences (the join; the release transient on hardware; the ledger).

### 3 System and method
- **3.1 Rig, twin, roles.**
  - Airframes; Betaflight as a rate autopilot; ELRS; motion capture; the 6×6×4 m cage.
  - Rods 0.55 m with ball joints, pivot 4 cm below centre. QUAD4's tether goes slack on the rig and is rigid in the twin (locked).
  - Magnets catch on contact only and hold about 9–10 N; add the angle dependence if measured Wed.
  - **T4, plate numbering by angle:**
  - | Thesis (CCW from +x) | Angle | Rig (CW from +x) | Tejen (CCW) |
    |---|---|---|---|
    | 1 | 30° | 11 | 1 |
    | 3 | 90° | 9 | 3 |
    | 5 | 150° | 7 | 5 |
    | 6 | 180° | 6 | 6 |
    | 9 | 270° | 3 | 9 |

    Text and tables use angles. The 7 Oct registry rows use the CCW numbering, the 8 Oct rows the CW one.
  - **Twin:** built from the rig-identified thrust map, geometry and masses. **Not modelled:**
    - loop latency: the twin cannot fly with 50 + 30 ms of delay, a drone tipped 83° at takeoff;
    - solver timing (the twin runs at about 0.3 real time);
    - mocap noise;
    - rotor wake;
    - the slack tether;
    - the magnet's holding limit.
  - Rig body rates were rounded to 0.1 rad/s until the evening of 8 Oct (fix 96acbcf).
  - F2: rig photo plus a to-scale top and side view.
- **3.2 Model and planner.**
  - 13 + 14N states, the d term, F3 (payload free-body diagram).
  - acados RTI, N_h = 20 over 2 s.
  - Runtime attachment geometry; one solver per team size, primed twice.
  - The 60 ms budget, with the rig's four-drone median of 66 ms explained from the logs before it is written.
  - "Dissipative mode" relabelled: the rig flew the OCP inside the reconfiguration node.
- **3.3 Tracker.**
  - 17 states; the affine map; the measured pull (80 ms slope, 12 m/s² cap, rods planned at 60°).
  - The model integral in two equations (d → F, convergence 1/k_i, overshoot on release).
  - Takeoff and landing in 0.5 pp.
- **3.4 Reconfiguration** (F6, state machine with leave and join lanes and the fall-backs):
  - Notice: link stretch of 0.06 m; contact plus a lift.
  - Re-plan.
  - Redistribute: soft v2; v1 named as the negative that defined it; the newcomer seeded at 0.1 N.
  - Fall back.
  - **Three-equation statics** for three survivors.
  - **Support margin** m = r·cos(g_max/2), credited to statics (Michael 2011, with Tognon's qualifier: exact when the centre of mass lies in the attachment plane). Values: 112 / 86 / 58 / 29 / 0 mm for gaps of 120 / 135 / 150 / 165 / 180°.
  - **T5b, every constant with a one-line reason** (a noise floor times a factor, or a physical bound): 0.06 m stretch, 30° drop, proof 1 cm / 3 mm / 8 mm / 2.5 mm, seed 0.1 N, contact 0.01 m / 0.05 m/s / 0.15 s. Back each with the threshold sweeps of §6.
  - The even carry ring has zero margin after any loss; r200006 is the hardware origin of the N−1 rule.
- **3.5 Authority and safety as runtime assurance.**
  - F5: one block diagram of the cascade and its owners (planner, trackers, mux, Tejen's stack, Betaflight). It replaces old Figs 4 and 8.
  - Exactly one owner per drone.
  - No geofence, and the fleet disarm argued: low altitude and the cage; a falling ring is safer than an uncontrolled armed drone.
  - **What is tested where.** "A tested way down in every phase" holds in the twin. On the rig it holds for 21–22 Oct only with `warm_fault_not_live` after its reviewer. `tilt_cut` and `fc_disarm_land` stay off until 28–29 Oct (D6). Before then a stuck take-back ends in DISARM: QUAD4 falls 2.3–2.9 m.
  - The fault-coverage matrix: one paragraph here, full detail in App K.
  - D6 stated plainly: redistribution after a release flew on the rig; the policy for a true fault stays the fleet stop and the drop rule.
- **3.6 Method.**
  - Cards; bars frozen before the runs; critic and reviewer described accurately, as automated checks against pre-frozen bars, not independent review; the AI disclosure.
  - One tree per claim table.
  - **Rig rows carry their own tree.** Each rig row's twin pair is flown on that same tree before the flight; that pair is the ledger row. The frozen-tree twin is a separate column.
  - **Resolution rule.** A lever pair is "resolved" only if all three hold:
    1. the sign matches the frozen sign;
    2. the arms' ranges do not overlap;
    3. the difference is at least max(1°, 2 × the larger within-arm spread).

    Otherwise it is "unresolved".
  - Every row is labelled **replication** (seen 7–8 Oct) or **new prediction**.
  - **T6, metrics defined once:**
    - **Δθ (primary):** the peak of |tilt vector − its mean over the 5 s before the event|, over the event to LAND.
    - The scalar rise and the absolute peak beside it; the absolute value is used for the 30° drop rule and as the safety limit.
    - Event time: the command (announced), the ramp start (soft), the magnet-OFF time in the bag (unannounced).
    - **Time back in band:** within max(2σ_pre, 0.5°) of the *post-event* steady tilt, held for 1 s.
    - Dip; detection time; steady offset; landing lean; freed-drone clearance; survivors' share.
    - **Peak survivor pull and its rate, against the hold.**
    - Contact → proof; throttle step at the switch.
    - With every arm: the pre-event offset, tilt and leaver pull.
  - Ledger band rule, cross-referenced to §4.4.

### 4.1 Carry (RQ1), 3.5 pp
| Claim | Figure/table | Runs (planning only) | Still needed |
|---|---|---|---|
| One thrust map fits four airframes; free hover within 4 cm without an integrator | F7 (exists), T6 row | RIG-0930 ladder1/ladder4/model-hover | – |
| The measured pull plus the 60° plan: circle 8.25 → 2.91 cm, figure-eight (0.25 m/s) 8.2 → 3.0 cm | T7 carry table (pull source, rod plan, error mean/max, lag, z sd, breakaway, n); F8 circle plan view, along-track error and height | r006/r206e60, r0007/r2008e60, r002/r20004 | Thu 15: the 60° planned-pull circle (falsifier); a measured-pull repeat |
| Rig-only error sources | T7b source → signature → remedy; cable-replay paragraph | cable replay r0001/r002/r006; twin replay of R1105/R1106 | The 0.55 N test on twin logs (no run) |

Answer in four sentences. Hover rock is reported as before and after the rate fix.

### 4.2 Leave (RQ2), 8 pp, rig figures first
1. **The event on the rig (1.0 pp).** F9, the r305 storyboard; caption about 60 words, detail in the text.
2. **The full hardware record (1.5 pp).** **T8**, one row per rig release event: r0012, r0013, r302_3, r303_4, r304_3, r305, r200006 ×2, then W4, W5 and later flights.
   - Condition columns: pull source, rod plan, integral mode, announced / unannounced / uncommanded, layout, largest gap, margin, freed-drone policy, software version (a thesis label; SHAs only in PROVENANCE), before or after the rate fix.
   - Metrics: detection, Δθ, absolute peak, dip, time back in band, steady offset, landing lean, peak survivor pull, and the twin range with n.
   - F10 panel (a).
   - Rule: 6.5° is never quoted without 15.4°. The range is 4.6–17.8° for the five that carried on.
3. **How the ring was lost (1.0 pp).** **F11, magnet anatomy.**
   - r0013 timeline: the planned tension step (+64 % / +68 % / −34 %, survivor 1 from 2.80 to 4.59 N), the uncommanded release at +0.155 s at 2.6°, the second at +0.386 s, tilt to 68.6°. The twin at the same settings gives 5.5°: it has no holding limit.
   - r200006: two uncommanded releases with the measured pull at 45° (3.0 N per rod against about 2.5 N at 60°). Recovered from 30.6°, then capsized at 61° in the descent.
   - The 30° drop rule replayed on r200006 fires at +0.51 s on a ring that recovered: the false-drop trade-off.
4. **Where the survivors learn of the change (1.25 pp).**
   - F10 panel (b): the twin crossover (Tier A), with rig dots as they come (W4; the soft-v2 pair; W4ff if flown).
   - Text: the frozen sign, then the outcome under the resolution rule.
5. **Other levers (1.5 pp).**
   - **F12**, two panels:
     - (a) a forest of one-variable pairs: ramp v2 on each pull source, v1 negative, resize timing (release / detection / never), integral mode (twin only), and the model-error arm;
     - (b) Δθ against support margin, split by pull source, from Tier B plus the rig points; r0013 off-scale and marked "magnet slip".
   - Claim only the measured-pull curve and the N−1 rule.
   - T9 lever table in full in App J.
6. **What knowing about the release buys (0.75 pp).**
   - Statics gives the shares; the OCP's re-placement (rod directions, horizontal balance) is measured.
   - Detection on against off, on both pull sources (Tier A fills the planned × detection-off cell).
   - Scored on time back in band, landing lean and clearance.
7. **The switch itself (0.5 pp).** **T10 validation, leave and join switches:**
   - solver status and KKT residual at the first live solve;
   - the gap to an offline SQP solve of the same logged problem run to convergence;
   - solve time against 60 ms (rig only);
   - the throttle step.

   The switch is numerically clean. The tension step is a planned physical step, and on the planned pull it is a lever.
8. **Answer (0.5 pp).**

### 4.3 Join (RQ3), 6.5 pp
1. **The admission contract and the proof timeline (1.25 pp).** F14 on twin data, with the rig's noise band overlaid from the replay: contact, the lift, plate rise against noise, proof, mux switch on the same tick, QUAD4 throttle continuous, ring tilt.
2. **The twin claim (1.0 pp).** T11 from the M1 joint claim on the rig candidate tree, with contact_still on, and its reviewer. Quote false admissions per tree (n = 4 per tree); the pooled 12 over 5 trees is exploratory.
3. **Counterfactuals (1.0 pp):** seed, force proof, hand-over step against TRADY's blend, and the take-back (hover passes; the circle failed three ways: rim snag, a 0.24 m pass over a carrier, a sweep into a carrier). The co-moving take-back appears here only if it passes by the Thu 15 gate; otherwise the claim is hover-only (D1).
4. **The proof on rig noise (0.75 pp).** The full proof logic replayed over rig bags (plate-9 point at full rate, Wednesday's body-24 noise and hand catches): false-accept and false-reject rates, and the rig thresholds set from them. The twin's proof (1 cm, rigid) and the rig's (≤ 3 cm after slack, plate rise ≤ 2 cm) are stated as different protocols.
5. **On the rig (1.0 pp).** **F15**: R11a (the null) and R11b ×2, then n ≥ 3. Scored on Δθ against R11a's same window: ≤ 5° on Δθ for the thesis, ≤ 10° absolute as the safety limit. Fallback wording is written now; Wesley picks by 29 Oct.
6. **Fault coverage and authority (0.75 pp).**
   - One paragraph, per tree.
   - Tilt cut: injection → disarm about 0.26 s, with the switch-off baseline staying armed.
   - No false cut over 45 rig bags.
   - drop_on_loss armed on 4 rig detaches, no false trigger.
   - Disarm-alone 17 of 17 (twin).
   - The matrix in App K.
7. **Full M1 (0.5 pp).** T12 scored from `metrics.json` m1.windows under its own card, 2 hover + 2 circle. Reconcile R1523 (1.22° against 4.78°).
8. **Answer (0.25 pp).**

### 4.4 What the twin predicted, 2.5 pp
- **T13, parity in RotorTM format on Δθ,** with n. Factors recomputed on Δθ, rows labelled before or after the rate fix.
- **F16, the ledger.** Per rig flight: the frozen band as a bar, the rig value as a dot.
  - Scored three ways: hit or miss; band width; log(rig / twin midpoint).
  - Also scored against two naive predictors frozen with it: "rig = twin" and "rig = the previous rig flight of the same cell".
  - Only "before" rows are scored; the 8 Oct pairs are "agreement, not scored".
- **Band rule:** [twin min × f_lo − 2σ_rig, twin max × f_hi + 2σ_rig].
  - Class factors are used only where ≥ 2 rig pairs exist, else f = 1.
  - σ_rig is the sd of the rig's pre-event Δθ null.
- **The clear-a-fix rule:** a twin arm clears a rig fix only if the twin reproduces the failure.
- **Pattern:** the twin misses mechanisms it lacks (the magnet hold, wake, latency) and hits the reactive release.
- **The measured-pull reversal** appears only if Tier A's 60° circle pair is re-flown on the frozen tree; otherwise it is cut.

### 4.5 Discussion, 3.5 pp
- **a. Feedforward against feedback (1.0 pp).**
  - A one-degree-of-freedom tilt model: survivor thrust = planned + k·(measured − planned).
  - Its predicted signs are frozen before Tier A flies (§6), then scored. It must not claim r0013.
  - Design rule: the measured pull in steady state; the plan, ramped, at announced events. Switching the pull source at the event is future work.
- **b. The magnet margin as the binding hardware constraint (0.5 pp).** The 60° plan and the ramp are levers on the margin as much as on tilt.
- **c. Numbers against T3 (0.75 pp).**
- **d. What the resize does not guarantee (0.4 pp):** measured over N switches, set beside plug-and-play MPC.
- **e. Design rules (0.35 pp):**
  - N−1 layout with margin > 0;
  - plan the rods steep;
  - ramp every announced change and every join;
  - seed at 0.1 N;
  - a geometric proof.
- **f. Limitations (0.5 pp):**
  - n per rig cell; lab speed; motion capture;
  - a single 4↔3 change; rigid twin links; no holding limit in the twin;
  - mixed integral modes across rig days;
  - dependence on Tejen's controller;
  - releases, not dead carriers.

### 5 Conclusion, 1.5 pp
- One answer per RQ with its number.
- Value: the hardware release record, the admission contract, the ledger.
- Future work: onboard sensing, outdoor flight, more than one change at once, a pull-source switch at the event, a slack-tether twin with a holding limit.

### Figures (14 main) and tables (13)
**Figures:**
- F1 storyboard
- F2 rig and geometry
- F3 free-body diagram
- F5 cascade and owners
- F6 state machine
- F7 thrust map
- F8 carry circle
- F9 r305 storyboard
- F10 headline
- F11 magnet anatomy
- F12 levers and margin
- F14 proof timeline
- F15 rig attach
- F16 ledger

**Moved or cut:**
- Cut: rod against spring-damper; the reversal (unless re-flown); old-tree figures.
- Into F10a: the departed landing.
- To App E/F/K/D/B: tip feedback, the 45 against 65 attach elevation, landing phases, steady-error figures, solve time.

**Tables:** T1–T13 as named above. T3, T8 and T11 carry the weight.

---

## 5. What to cut or rewrite in the current draft (thesis_v2, thesis-rig 1659d7d)

| File / element | Action |
|---|---|
| `title_page.tex` | Date after the due date is confirmed (now 20/11/26); 100-word abstract (§8); acknowledgements; originality statement naming Tejen's parts; real nomenclature; AI statement if required |
| `intro.tex` para 5 and Fig 1 | Narrow the gap sentence; F1 replaces the TikZ mission profile; T1 roles |
| `contributions.tex` | Replace Q1/Q2/RQ1a/RQ1b/RQ2, the hypothesis, `tab:rq_map` and contributions 1–6 (3 redo boxes) with RQ1–RQ3, C1–C3, M1–M2 |
| `control.tex` :86–142 | Condense to 1.5 pp in 2.1; the distributed detail and Fig 3 → App F |
| `control.tex` :144–193, `Literature_review.tex` :39/:47/:65/:67, Table 2 | Rebuild as 2.2–2.4 with T3; fix the Bacheti (3→2→3 on hardware), Tasooji and "only work with a mid-task dropout" cells; state Quan's unplug-and-play claim, then the difference |
| `modelling.tex` λ ≥ 0 | A modelling choice (Six 2018) |
| `system_overview.tex` | Split into 3.1 and 3.5; T4 by angle; links and magnets; what the twin does not model; owner diagram |
| `controller.tex` :149–241 | Keep the two-equation derivation; the alternatives → App D |
| `takeoff_landing.tex` | 0.5 pp; "slack tethers" consistent with 3.1 |
| `reconfiguration.tex` detach | Notice / re-plan / redistribute / fall back; statics; margin with qualifier; T5b reasons |
| `reconfiguration.tex` attach :76–132 | Rewrite as the admission contract; the 8 cm weld approach → App E; the network attach → App F; TODO :81 cite Tejen "in preparation" |
| `dissipative.tex`, `sim_architecture.tex` (6 redo) | → App F, labelled as the earlier plant |
| `simulation.tex`, `trajectories.tex`, `metrics.tex` | → 3.6 with T6, the resolution rule and the band rule; current run and test counts |
| `res_carry.tex` | Keep Table 3 (→ T7) and Fig 10 (→ F7); fix the figure-eight speed to 0.25 m/s; cut Fig 9, Table 17 and Table 4; landing → App K; computation → T10 with "dissipative mode" relabelled; Fig 13 → F16, placeholder point removed |
| `res_steady.tex` | Keep Table 6 (→ T7b) and Fig 16 (or fold into F8); the rest → App D; "56 % and 51 %" → about 28 % / 25 % |
| `sim_detach.tex` | Delete the layout redo, "The transient", Fig 17, Table 20 and the 2 network redo boxes; Table 7 → T8 with the uncommanded releases and conditions; the r0013 text rewritten (step, then a magnet slip); Figs 18–20 updated on Δθ; Fig 21 merged; **"QUAD4" in the caption at :197 and "rig W4/W5" at :201 removed** |
| `sim_attach.tex` | Delete the network attach, attach by OCP resize, errors during attach (5 redo boxes); tip loop and Fig 23 → App E; Fig 24 → F15 |
| `discussion.tex` | "Geometry decides" rewritten on F12b; :115–118 hedged; Table 8 → T3 row; dip range reconciled; add §4.5a/b |
| `Conclusion.tex` | Rewrite per RQ; remove 3 redo boxes and the "geometric condition" claims at :39 and :63 |
| Abstract | "Within 1 s" goes; replace with §8 |
| `Appendices.tex` | Weld radius = contact on the rig; Tables 13/24 updated; App C arming retries; App G negatives; label Tables 16 and 19 (n = 1, ε = 0.025); delete the weld-ladder and ground-join redo boxes |
| Every file | A vocabulary pass (QUAD4 → "the newcomer", soft v2 → "ramped release", take-back, rungs, Tiers); thesis flight labels L1–L9 (leave), J1… (join), K1… (carry), mapped in PROVENANCE; a grep lint before each hand-off for `R\d{4}`, `RIG-`, `r30\d`, `QUAD\d`, `W\d`, `R8`/`R11` and branch names outside comments |
| Captions on Figs 13, 19, 22 | About 60 words each |

---

## 6. Evidence still needed by 7 Nov (priority order; lab days fixed)

| P | Item | Serves | Where / when | Gate | If lost |
|---|---|---|---|---|---|
| 1 | Freeze: T6 metrics (Δθ vector, 5 s pre-window, event times, post-event band), the resolution rule, the replication/prediction labels, the ledger band rule, the 1-DOF model's signs for every Tier A cell | all of RQ2, §4.4 | laptop docs, Mon 12–Tue 13; Wesley signs | none | W4/W5 become "after" rows |
| 2 | W4/W5 bands from the same-tree twin runs (R1457, R1459, R1460) plus 1 run each on week4 HEAD; factors recomputed on Δθ from the 8 Oct pairs | ledger | lab, one slot inside the M1 plan's Mon 12 schedule | P1 | as above |
| 3 | Registry: append corrections for the 7 misaligned rows (R1517, R1536, R1541, R1557, R1564, R1573, R1579) and reconcile R1523; back up `results/` | integrity | Mon 12, before the rig stack goes up | – | – |
| 4 | **W4, W5** (model integral plus disarm-alone; a new cell). Ask for 15–20 s on three drones before LAND | headline cell; the freed drone lands itself | rig Wed 14 | P1–P2 | headline stays on old code |
| 5 | Props-off additions (Wesley's call): spring-scale magnet hold at 0/30/45/60° from the axis; hand catches and body-24 noise for the proof; phone and tripod video of every flight from now on | magnet margin; RQ3 rig thresholds; F1/F2/F15 | rig Wed 14 | – | margin stays qualitative |
| 6 | Offline, rig stack down:<br>• Δθ (vector) and time in band for all rig releases<br>• slip detector over the 8 Oct bags<br>• r302_3/r303_4 planned step and measured term at the resize tick (the double-count test)<br>• rig latency (commanded against measured body rates)<br>• rig Δθ null and the minimum detectable effect at n = 3<br>• stretch-detector sweep (false alarms over 45 bags; delay against threshold from 5 releases)<br>• cable replay on twin R1105/R1106<br>• switch validation (T10)<br>• margins per layout<br>• r305 dip reconciliation | RQ1–RQ3, T5b | laptop, Mon 12–Fri 16 between rig days | – | – |
| 7 | **60° planned-pull circle**; the 3-min ring hover (covers ≥ 30 s); R10a | C1 falsifier; precondition for any planned-pull detach | rig Thu 15 | – | C1 stays "two changes at once" |
| 8 | M1 joint claim batch on the rig candidate SHA, then the reviewers → the join table and the R11b prediction. Optional: one twin arm with the rig's proof lift (3 cm, plate rise about 2 cm) to get degrees of ring tilt per mm | RQ3 | lab Fri 16, reviewers Sat–Sun | M1 plan steps 23–30 | R11b becomes an "after" row |
| 9 | Proof logic replayed over rig bags → rig thresholds; R11b bars on Δθ; F1–F9 closed | RQ3 rig | before 21 Oct | P5, P8 | NO-GO |
| 10 | **R11a, R11b ×2**, hover only unless the co-move passed the Thu 15 gate | C3 rig | rig 21–22 Oct | P9, Tejen's OK | fallback wording |
| 11 | **Tier A, exploratory** on the week5 candidate (predictions frozen in P1 and scored on these first flights): <br>• crossover 2×2 at 120°: {measured, planned} × {announced soft v2, unannounced}, ×3<br>• plain announced on both sources, ×3<br>• detection off on both sources, ×3<br>• model-error arm (planner mass or rod angle off by the rig's error), ×3, Wesley's word<br>• reference integral, ×3<br>• 60° circle measured against planned, ×2<br>Each repeat shifts the release instant against the ring's motion. About 35 runs | RQ2 levers; W4ff preview (survivors' peak planned pull and step against the measured hold) | lab from Sat 17, when free | P1 | levers stay twin-unresolved |
| 12 | Measured-pull soft v2 (n = 3) passes its reviewer | gate for P14 | Sun 18–Fri 23 | P11 | no rig ramp pair |
| 13 | **Twin freeze about Sun 25 (week5).** Tier A and Tier B claim re-fly. Tier B: measured pull, unannounced, gaps 135/150/165/180 ×3 (drop_on_loss off for 165/180 or recorded as drops); planned 150° ×3. Tier C only if the join path changed. The full-M1 card 2 + 2. Labelled F6b/H1/F6a arms per D7 | claim tables | lab, by about 2 Nov | – | tables stay exploratory |
| 14 | **Rig 28–29 Oct:**<br>• announced soft v2, measured pull, 120° (the ramp row; against r302_3)<br>• announced soft v2, planned pull at 60°, 120° (the pull-source pair)<br>• repeats to n ≥ 3 of the 120° unannounced hover and circle<br>• R11b to n ≥ 3<br>• tilt cut on if passed and Tejen agrees<br>• **W4ff only if** the 60° planned circle flew clean, the preview's peak survivor pull is ≤ hold/2 at that angle, and a 2-run twin preview on the rig SHA is in | RQ2 rig pairs; RQ3 | rig | P7, P12, P5 | drop W4ff first |
| 15 | Full M1 on the rig if R11b passed; else a reserve session for missed cells | F1 rig insets | rig 4–5 Nov | R11b 2 of 2 | stays twin |
| 16 | Reviewers on every claim table (leave levers, join, full M1, fault coverage) | – | before Sat 7 Nov | P13 | – |

**Stop rule for lost rig sessions.** Drop in this order:
1. W4ff;
2. the planned soft-v2 rig flight;
3. the full M1 on the rig;
4. repeats beyond n = 2.

**Minimum per cell:**
- the 120° unannounced hover and circle with the model integral, n ≥ 2;
- R11b, n ≥ 2;
- the 60° planned circle, n = 1.

**Optional, only if the lab is free after Tier B:**
- the LAND-in-every-combined-phase re-fly (about 40 runs). Otherwise the matrix is shown per tree as exploratory.
- the masking or "detected, not resized" arm, only if the tension bounds are already runtime parameters. It is new flight-loop code otherwise; see D8.

---

## 7. Literature additions
One approval batch for Wesley, about 35 entries. Each is checked before citing; preprints are marked; the peer-reviewed version is used where one exists.

| Section | Add |
|---|---|
| 2.1 | Sreenath & Kumar 2013 (RSS); Lee, Sreenath & Kumar 2013 (CDC); Mellinger 2010; Six 2018 (RA-L); Masone 2016; Jackson 2020 (RA-L); Geng & Langelaan 2020; Liu…Quan 2026 (TNSE; replaces arXiv 2509.03563 where they overlap); Wang 2026 (T-RO) |
| 2.2 | Zeng 2025 (CoRL); Oishi 2023; Hajieghrary & Schmitt 2026 (preprint; recovery time); Delbene & Baglietto 2024/2025 (read in full); Liang 2021; Gao 2024; Saiella 2021; Bacheti 2026 (ISA Trans; read in full, check whether the agent count changes); Fink 2011 (IJRR); Tognon 2018 (RA-L); Michael 2011; Xu 2025; Farina, Zhang & Scattolini 2020 (TCST); Riverso, Zeilinger, Aboudonia, Saccani |
| 2.3 | Loianno 2018 (RA-L); Miyazaki 2018 (IROS); Larsson 2020; LEGION 2026 (preprint); Gawel et al. 2017 (ICRA, magnetic picking with MAVs) |
| 2.4 | Graebe & Ahlén 1996; Peng, Vrančić & Hanus 1996; Sha 2001 (Simplex); Diehl, Bock & Schlöder 2005 (RTI); Gros 2020; Hendrickx & Martin 2017; Şekercioğlu, De Carli & Dimarogonas 2026 (preprint) |
| 2.5 | Tagliabue 2019 (IJRR); Pannocchia & Rawlings 2003; Maeder, Borrelli & Morari 2009; de Marina & Smeur 2019 (ICRA); Bulka 2022 (ICUAS); Klausen 2020; Verschueren 2022 (acados) |
| 2.6 | RotorTM (Li, Liu & Loianno, T-RO); RotorS (Furrer); Cruz & Fierro 2017 |
| 3.1 | Tejen's thesis "in preparation", agreed with the supervisor, plus a stand-alone description |

---

## 8. Abstract draft (template limit 100 words)

> Quadcopter teams carrying a suspended payload may lose or gain a member in flight. This thesis treats each change as an observed switch between pre-built optimal-control problems and asks what sets the payload's transient. On unmodified Betaflight firmware, four quadcopters carried a 0.86 kg ring at 0.125–0.25 m/s; three carried it on through five of six releases, detecting unannounced releases in 0.33–0.42 s. The ring was lost only when a magnet let go. [In simulation,] where the carriers learn of the change, from the plan or their measured rod pull, and the support margin set the peak tilt. A quadcopter flown by another student's controller was admitted [in simulation | on the rig].

About 98 words. Brackets fill from the rig pull-source pair and R11b. If the rig pair flies and resolves, "[In simulation,]" goes.

---

## 9. Schedule to 27 Nov (order of work; only lab days, the freeze and submission are fixed)

| Window | Fixed | Evidence | Writing |
|---|---|---|---|
| Sun 11 – Tue 13 | Tue 13: supervisor meeting | P1–P3; Wesley's word on §10 | No prose (pause kept); freeze texts for §3.6 |
| Wed 14 – Thu 15 | Rig: W4, W5, R8a0, props-off; Thu 15:00–18:00: R10a, 60° planned circle, 3-min hover | P4, P5, P7; P6 when the stack is down. Nothing on the laptop while the stack is up | Figure scripts on final rig data (F7, F9, F11, rig half of F10) |
| Fri 16 – Sun 18 | – | M1 joint batch, reviewers (P8); Tier A starts Sat 17 (P11) | From Fri 16: Ch2, §3.2, §3.3, §3.6; then §3.1 and §4.1 |
| Mon 19 – Sun 25 | Tue 20 desk rehearsal; Wed 21 / Thu 22 R11a, R11b; R11b band frozen before 21 Oct | P9, P10, P12; **twin freeze about Sun 25** | Rig half of §4.2; T3 built; Ch2 + Ch3 (minus 3.4/3.5) to the supervisor |
| 26 Oct – 1 Nov | Wed 28 / Thu 29 rig (P14); fallback wording chosen by 29 Oct | P13 claim re-flies | §3.4, §3.5 on the frozen design; §4.1–4.2 to the supervisor |
| 2 – 7 Nov | Wed 4 / Thu 5 rig (P15, the last rig day); **Sat 7 results freeze** | Reviewers (P16) | §4.3, §4.4 drafts |
| 8 – 14 Nov | – | No new runs | §4.5, Ch5, abstract; every figure through the blind critic; full draft to the supervisor by about 13 Nov |
| 15 – 22 Nov | – | – | Revisions; Wesley's cuts to 50 pp; front matter; roles wording agreed with Tejen |
| 23 – 27 Nov | Submit Fri 27 Nov | – | Proof read, references, font audit; Wesley merges into Overleaf (never `/home/wesley/thesis`) |

**If the due date is 20 Nov:** everything after 7 Nov moves one week earlier, and §4.3/§4.4 are drafted during 2–7 Nov in parallel with the reviewers.

---

## 10. Decisions for Wesley (recommended option first)

1. **D1, research questions.** RQ1 (what the measured pull buys), RQ2 (leave), RQ3 (join), with the ledger as §4.4, not an RQ. The old RQ2 goes to App F. *Recommended.* Alternative: keep RQ4. The cost is about 0.5 pp and a weaker fit to "real life first".
2. **D2, subtitle.** "Leaving and Joining a Carried Ring" (*recommended*), with the control question in the thesis statement. Alternative: the long subtitle.
3. **D3, thesis statement and spine.** The conditional wording, the magnet margin, and feedforward against feedback (§2.2). *Recommended.*
4. **D4, freeze before Wed 14.** T6 metrics, the resolution rule, the ledger band rule, the W4/W5 bands from the same-tree runs plus one lab run each on Mon 12. *Recommended*; without it W4/W5 cannot be scored ledger rows.
5. **D5, W4/W5 duration.** 15–20 s on three drones before LAND, against the no-dead-time preference. *Recommended for W4/W5 only*: time back in band needs it.
6. **D6, Wednesday props-off additions.** The spring-scale magnet hold at 0/30/45/60°, and phone video of every flight from now on. *Recommended.*
7. **D7, W4ff.** Not on Wed 14. On 28–29 Oct only under the P14 preconditions. The rig pull-source pair comes from the two announced soft-v2 detaches. *Recommended.*
8. **D8, "what the resize buys".** Rename it "knowing about the release" and credit the OCP only for what statics does not give. Build the masking arm only if it is a parameter change. *Recommended*: skip the arm if it is new code.
9. **D9, model-error arm** in Tier A (the planner's mass or rod angle off by the rig's error). This touches launch values, so it needs your word. *Recommended*: twin only, a config arm.
10. **D10, R11b card.** 5° on Δθ against R11a's window, 10° absolute as safety, `proof_rise_max` 0.02 after the hand lifts, and the optional twin arm at the rig's lift. *Recommended.* The draft card scores 5° absolute, which the rig's pre-event rock (up to 6.1°) could fail on its own.
11. **D11, credit.** "Tejen's controller, with the changes in App X made by the author with his written agreement", the admission-contract framing, and his thesis cited "in preparation". *Recommended*; it needs his OK (D12 of the M1 plan).
12. **D12, writing.** No prose until Fri 16, then the order in §9, handing chapters over as they finish. *Recommended.*
13. **D13, checks at Tue 13:**
    - the due date (title page 20/11 against plans 27/11) and other Thesis C deliverables;
    - the supervisor's view on demoting the network RQ;
    - UNSW policy on AI assistance, and the disclosure wording before §3.6.
14. **D14, literature.** One approval batch for §7. *Recommended.*
15. **D15, registry.** Append correction rows for the 7 misaligned rows (*recommended*) rather than a byte-preserving edit.
16. **D16, LAND matrix.** Per tree as exploratory (*recommended*), with the combined-phase re-fly only if the lab is free after Tier B.

---

## 11. Critique findings: applied or rejected

**Examiner**

| # | Finding | Status | Where / reason |
|---|---|---|---|
| E1 | Pull-source lever is twin-only where the twin is weakest; W4ff off Wed 14 | Applied | P11, P14, D7; "[In simulation]" in the abstract until the rig pair flies |
| E2 | On the measured pull an earlier resize raises the peak | Applied | Statement §2.2(4); resize-timing row from the existing arms; offline double-count test (P6). A "1 s late" arm only if it is a parameter |
| E3 | Twin loop ≠ rig loop; rate rounding | Applied | §3.1 "not modelled"; before/after-fix labels; rig latency from logs |
| E4 | The proof's signal sits in rig plate noise | Applied | §4.3.4 replay as an R11b gate (P9); bars on Δθ |
| E5 | Unannounced release, not failure; D6 not contradicted | Applied | Wording throughout; T3 "removed or left hanging" column; §3.5. The document plan's D6 point is rejected |
| E6 | Tilt-vector Δθ; post-event band; time on three drones | Applied | T6; D5 |
| E7 | Resize scored against a straw man | Partly applied | Renamed, statics added, OCP credited narrowly; masking arm only if it is a parameter (D8): new flight-loop code is not worth the lab time before the freeze |
| E8 | Twin repeats near-deterministic; no minimum effect | Applied | Resolution rule; n = 3; repeats shift the release instant; replication labels |
| E9 | Rig rows cannot obey one tree | Applied | §3.6 per-tree rule; version column in T8 |
| E10 | Ledger bands too easy | Applied | Three-way scoring, naive predictors, class factors only with ≥ 2 pairs |
| E11 | Automated review presented as rigour | Applied | §3.6 wording; P3 registry fix; D13 AI policy |
| E12 | Subtitle overpromises the join | Applied | Short subtitle (D2); RQ3 reworded |
| E13 | "Independent controller" is our modified port | Applied | T1, App list, D11 |
| E14 | Thresholds look ad hoc | Applied | T5b reasons; threshold sweeps (P6) |
| E15 | Carry layout breaks N−1; wide gaps hit the 30° rule | Applied | §3.4, §4.1; Tier B drop handling |
| E16 | The 0.55 N attributed to wake untested | Applied | Hypothesis wording; twin-log replay (P6) |
| E17 | Detection agreement and r305 overstated | Applied | §3 headline wording; ranges and ratios |
| E18 | Levers lack mechanism; 1-DOF fitted after the fact | Applied | Pre-event state with every arm; predictions frozen in P1 |
| E19 | Rig list too big; no stop rule | Applied | §6 priorities and stop rule |
| E20 | RTI "converges" undefined | Applied | T10 metrics |
| E21 | Five contributions too many | Applied | C1–C3 plus M1–M2 |
| E22 | Facts: figure-eight 0.25 m/s; circle credits two changes; range 4.6–17.8° | Applied | C1, C2-A (verified in the registry) |

**Reviewer**

| # | Finding | Status | Where / reason |
|---|---|---|---|
| R1 | Magnet release is the real failure mode | Applied | Verified in the replay card; §2.2(2), C2-A, F11, the magnet-margin metric, D6 |
| R2 | The rig proof is a different protocol | Applied | §4.3.4; optional twin arm (P8); D10 |
| R3 | F5 gate against the 25 Oct freeze | Applied | The join table is the M1 joint batch on the rig SHA (P8); frozen-tree table later |
| R4 | Twin pull-source biased; model-error arm; reversal confounded; crossover spine | Applied | Tier A model-error arm (D9); F10b; reversal only if re-flown |
| R5 | R1144 not one variable; statics | Applied | §4.2.6, §3.4 |
| R6 | Levers below rig resolution | Applied | Rig null and minimum detectable effect (P6); integral mode twin-only; ramp on the rig on the measured pull |
| R7 | W4ff can recreate the r0013 step | Applied | P14 preconditions |
| R8 | Replication against prediction; r0012/r302_3 not a pair | Applied | §3.6 labels; C2-B wording |
| R9 | Ledger too easy; demote RQ4 | Applied | §4.4 scoring; RQ4 as a section (D1) |
| R10 | Comparisons not like-for-like | Applied | T3 detection against recovery; detection wording; tilt cut injection → disarm 0.26 s; hedge on "only hardware" |
| R11 | Hardware record cherry-picked | Applied | T8 includes the uncommanded releases; the false-drop trade-off |
| R12 | Credit: admission contract | Applied | C3, T1, D11 |
| R13 | Padding | Applied | RQ1 reframed; fault matrix as a paragraph plus appendix; switch table as validation; rod-against-spring and reversal figures cut; departed landing merged; 14 figures |
| R14 | Carryovers (r0012 rise 2.2°; W4 against r303_4 not an integral effect) | Applied | T8 on Δθ; §3.6 one-variable rule |

**Completeness**

| # | Finding | Status | Where / reason |
|---|---|---|---|
| C1 | Writing back-loaded; due date unconfirmed | Applied | §9, D12, D13 (title page verified at 20/11/26) |
| C2 | Ledger predictions from the wrong tree and metric | Applied | P2, D4 |
| C3 | Statement overclaims (a), (b), (c) | Applied | §2.2, §3.5, C2-B |
| C4 | Levers unresolvable at n = 2; signs known | Applied | Resolution rule, n = 3, labels, 1-DOF frozen first |
| C5 | W4ff config is fine; the order is unsafe | Applied | Verified in `common.yaml`; read-back diff replaces the "new config"; P14 |
| C6 | Duplicates the M1 plan | Applied | The join table is the M1 joint claim; Tier A from Sat 17 |
| C7 | Coverage counts pool trees | Applied | Per tree; re-fly optional (D16) |
| C8 | Credit incomplete; Tejen's thesis may not be locatable | Applied | D11; "in preparation" plus a stand-alone description |
| C9 | RQ4 against Wesley's preference; drop the land_unwind-off run | Applied | D1; that run dropped; ledger from by-product previews only |
| C10 | Plate mapping wrong | Applied | Verified; T4 by angle |
| C11 | Soft v2 not ready for the rig | Applied | P12 gate before P14 |
| C12 | Rig sessions oversubscribed | Applied | §6 session list; the ≥ 30 s hover folded into the 3-min hover |
| C13 | Circle credits one of two changes | Applied | C1, abstract wording |
| C14 | Rig and twin trees drift | Applied | §3.6 rule; version column |
| C15 | Metric definitions; missing no-run work | Applied | T6; P6 |
| C16 | No-run-ids mechanism; leaked labels | Applied | §5 lint and vocabulary (leaks verified at sim_detach.tex:197/:201) |
| C17 | Hero figure is twin data; glyph fonts; mixed metrics | Applied | F10 rig-first; markers instead of glyphs; seed row moved to the join chapter |
| C18 | Literature: preprints; approval; the 8 pp split | Applied | §7, D14, note to the supervisor |
| C19 | No rig footage planned | Applied | P5, D6 |
| C20 | Only the 100-word abstract | Applied | §8; the 200-word version is rejected as wasted effort |
| C21 | Back up `results/` Mon 12 | Applied | P3 |

**Rejected from the direction and the document plan** (superseded by the checks above):
- "The switch itself does not set the transient", unconditional.
- r0013 explained as "an un-ramped step alone".
- The "new config" must-fix for W4ff.
- The T4 mapping 3/11/7/9.
- r0012 against r302_3 as a pull-source pair.
- Ledger factors on absolute peaks.
- The 200-word abstract.
- "Feedforward is catastrophic when the model is wrong", which was built on r0013.
- RQ3 bars of ≤ 10° on the rig as the go/no-go: replaced by 5° on Δθ with 10° absolute as safety (D10).

I worked read-only. I made one deviation: to read the report guide I extracted the .docx into a scratch folder in /tmp, then deleted it. Nothing else was created or changed.
