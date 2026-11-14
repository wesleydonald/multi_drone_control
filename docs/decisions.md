# Decisions (dated; who; why). OPEN lines are waiting on Wesley.

2026-09-09  Wesley  Payload is a 500 mm ring, 0.6 kg, rim magnets anywhere. Weld method free as long as it behaves like a ball joint.
2026-09-10  Wesley  Sim kT derived from the operating point (thrust_ratio:=auto); hardware keeps 24.
2026-09-10  Wesley  RViz role colours and error lines removed; banner kept.
2026-09-10  Wesley  Tethers at 4/12/8, newcomer at 6 (three-drone hover level; four-point rim uneven).
OPEN  Which layout is the default for the demo: 4/12/8 (level hover, rolled circle) or 3/12/9 (tilted hover, level circle)?  [2026-09-25: for the 4->3 DETACH the answer is plates 1/3/5/9 (R0510, R0554, R0556); this line is about the 3-drone ATTACH demo layout]  [2026-10-05 Wesley: deferred to the attach re-fly]
2026-10-05  Wesley  CLOSED: the 2026-09-10 evening work on measured-force-velocity-loop is committed there (768ca09, 11 Sep, pushed); kept on that branch, not merged.
OPEN  Make attach_traj_hold_mode=settle the default?  [2026-10-05 Wesley: deferred to the attach re-fly on the 0.86 kg ring (thesis R6); timed stays]
2026-09-16  Wesley  RViz status banner over the payload removed (fleet_viz text marker + display); /fleet/status topic kept for tools.
2026-09-16  Wesley  Tether magnet aux channel defaults to channel_6 (Betaflight AUX4) on real_io_launch and elrs_interface; found by tools/aux_sweep.py.
2026-09-23  Wesley  measure_rod_len=true is the launch default (sim and real), cable_len 0.5 as the initial estimate; SIL matrix R0307-R0327 absorbs rod +-4 cm / rim +-2 cm, no-op exact (Gazebo half not flown). Real thrust_ratio stays 24, sim stays derived. Same day: the 'kT is fixed' lock is LIFTED for a slow bounded per-drone thrust-gain trim (card 2026-09-23_kt_trim.md).
2026-09-23  Wesley  Payload official mass 0.86 kg. Sim payload is Te-Jen's M2A ring (simulation_assets/m2a_ring_fixture.sdf: 24 segments, 12 magnet plates every 30 deg, plate 0 on +x) in every world, generate_rigid_world.py default mass 0.86; bench legs omitted (flying payload rests on the ring). Same day, Wesley: params.py, launch defaults, Gazebo/SIL configs and the SIL plant moved to 0.86 kg; inertia derived from load_mass (ring per-kg values); +-25 % / +20 % belief arms rescaled (1.075 / 0.645 / 1.032) and flagged mis_seed.
2026-09-23  Wesley  Sim-interface rate ceilings are the launch defaults on rviz_quad_load_launch.py: clock_hz 100 (clock_throttle), mocap_hz 120, viz_hz 30. In-flight RTF 0.29 -> 0.45, hover unchanged to 1 mm (R0329 vs R0330). 0 restores the old behaviour.
2026-09-23  Wesley  Measured-force (INDI-lite) throttle removed from the velocity loop, tracker params, launches, tests and configs; the payload-force problem is handled on the planner side instead. Design note kept as history (docs/design/measured_force_loop.md).
2026-09-23  Wesley  Load-level force estimator (Kalman filter, OCP p_dist) deleted after the SIL matrix R0347-R0366 falsified it for a wrong kT; card kept as history. Next: per-drone vertical integrator, card 2026-09-23_drone_z_integrator.md, critic says not ready (sign, moving setpoint, freeze never fires in sim, kT not fixable by a vertical add-on).
2026-09-24  Wesley  Per-drone thrust-gain trim (kt_trim) built and passed its SIL matrix (R0405-R0442): typed kT 10-15 % off and mass +20 % all hover at the correct-gain height (0.614-0.620 vs 0.46-0.51 off), settle in 5-10 s, kT_hat within 0.7 % of the true gain. Default tau 1.5 s. Launch default kt_trim: see the report.
2026-09-24  Wesley  kt_trim Gazebo pair R0460-R0463: typed gain +10 % parks the load at 0.175 m off, 0.609 m on (reviewer: supported). Floor-contact detector added after the interactive run showed the estimate railing on the floor. Launch default for kt_trim: Wesley's word.
CLOSED  kt_trim: (a) floor-contact test rejects a hover under a typed-LOW gain beyond ~14 % (Gazebo 15:06, typed 30.4: stuck at 30.4, load 1.19 m); (b) TWO-drone hover with the trim ON chatters at 2 Hz and crashes at 27 s while OFF is steady (Gazebo 16:26/16:28): the estimate reacts to the tracker's own throttle corrections before the lagged acceleration arrives. Fix candidates in the card. Trim supported for 3 drones in hover only; not for rig use with 2 drones.  -> CLOSED 2026-09-25: (a) fixed by v2 (contact test only while the load is at rest), (b) fixed by v3 (aligned inputs, convergence freeze, airborne-only apply): Wesley's two-drone pair W-20:17/W-20:18 steady, no chatter; trim on by default since 09-24 and flown in R0558 (0.600).
DECIDED 2026-09-24  Sim throttle LINEARISED (Wesley's word): the sim Betaflight nodes (payload_betaflight_comm, the one every OCP launch runs per drone, plus betaflight_communication and angle_betaflight_communication) map motor speed = 4631*sqrt(u), so Gazebo thrust is a = 88.6 u at every throttle, like the rig's linear kT; the SIL plant is linear too; thrust_ratio:=auto is 88.6. Reason: with a = 88.6 u^2 a fixed linear gain matched one operating point only (hover height moved with mass; free drones on the floor could not follow the creep arc: R0466/R0468; attach weld capsize at the tangent gain). Every sim number before this line was flown on the quadratic plant (kT 32.9 / 36.75 / 40.22 secants).
DECIDED 2026-09-24  Rig floor start = the creep path (start_taut:=false handover_elev_deg:=45 handover_settle_s:=2.0), typed on the command line; the launch default start_taut:=true stays (R0466 lifted n3 in Gazebo even though the arc itself was not followed on the quadratic plant; re-fly on the linear plant).
DECIDED 2026-09-24  Drone mass 0.64 kg (Wesley's word): the x3 model is 0.64 over all its links, so DRONE_MASS lives in params.py at 0.64 (was 0.6 in planner_node), the planner declares `drone_mass` (launch arg on all seven OCP launches, default 0.64), thrust_c / 'auto' = 83.1, the SIL plant is 0.64 / 83.1, kt110/kt115 configs 91.41 / 95.57. The OCP cache signature covers the drone mass, so the solvers were prebuilt. RIG: `drone_mass:=<airframe weighed WITH its pack>` on the command line; 0.64 is the sim model, not a measurement. The measured rig kT (24) is unaffected (it is acceleration per throttle for the whole airframe).
CLOSED 2026-09-25 Planner height offset (card 2026-09-24_planner_offset): gate met (W-21:52 0.658 with the trim on), critic pass 2 'build with four changes' applied, ZBias built (z_ki 0 default; 0.2 = tau 5 s; z_i_max 0.15; z_taut_gate 0.99 sim / 0.9 real). SIL: 0.600 in every static-offset arm both directions, inert in the circle, rails+WARN under a gain fault, no upward authority against a typed-high gain (the trim's job). Gazebo pair pending; launch default stays 0 until Wesley's word. Two cheaper suspects the critic named: the 45-deg s_ref/t_nom references replayed from a node 0 re-pinned at ~42.5 deg, and the tracker's node-0 bound relaxation controller_mpc.py:1160 (lbx = x*(1-0.025), ubx = x*(1+0.025): INVERTED for every negative state component, +-2.5 cm of free height at 1 m). The bound fix is a one-liner (absolute +-eps); it is in the flight loop, so card + critic before a run.  -> SETTLED by the Gazebo pair R0557 (z_ki 0, trim off: 0.661) / R0558 (defaults z_ki 0.4 + trim: 0.600, tilt 0.1, zI -0.053); defaults already moved on Wesley's word 09-24. The node-0 bound inversion stays a diagnostic flag (x0_relax_symmetric tipped a drone on LAND in R0527: keep off).
DECIDED 2026-09-24  Launch defaults (Wesley's word, after W-22:26): kt_trim true and z_ki 0.4 on all seven OCP launches (+ real_hover kt_trim); start_taut false, handover_elev_deg 45, handover_settle_s 1.0, creep_vel 0.2 on the six floor-start launches (three_attach_launch keeps start_taut true: its attach worlds air-start on stands). Every existing config in configs/ was pinned to its previous semantics (kt_trim false, z_ki 0, start_taut true, handover_settle_s 0.75/0.5, creep_vel 0.1 where the launch default flipped) so registry baselines keep their meaning. The "kT is fixed" lock now reads: fixed typed gain as the initial value, kt_trim on by default.
CLOSED 2026-10-03  4 -> 3 detach on an even 4-ring hangs the 0.86 kg ring at 31-41 deg (R0493/R0494, linear plant; transient max 57 deg against the 60 deg fault). Any three of four 90-deg plates leave a 180-deg gap (CoG on the hull). For the rig: either accept a tilted 3-drone hang and LAND soon after the detach, raise the tilt fault margin knowingly, or use an uneven 4-ring (e.g. 0/90/180/270 -> detach the 270 one leaves 0/90/180: same; a 5th plate would be needed for a level 3-ring). Wesley's call before the rig detach.  [2026-09-25: runs settle it for Wesley's decision: plates 1/3/5/9 with the leaving drone at plate 3 hover level (R0510) and detach mid-circle at 6.9-7.7 deg peak, 0.3 deg settled (R0554, R0556); rig layout needs his word; Gazebo claim reviewed SUPPORTED 09-25 evening: 1/3/5/9 peak 3.0-3.3 / settled 0.1-0.2 deg vs even 25.6-28.5 / 6.7-6.8, R0559-R0572]  [closed by the 2026-09-28 rig layout decision: plates 1/3/5/9 for four drones, 1/5/9 for three (R0559-R0572)]

DECIDED 2026-09-25  Attach mode = OCP resize (n -> n+1), the mirror of the detach: SIL R0518/R0520 hold the 0.86 kg ring at a 10 deg weld peak in hover and mid-circle while the network attach capsizes it 8 s after every weld (R0519/R0521); built as `_attach_ocp` under reconfig_mode ocp (card 2026-09-25_attach_ocp). The 4-drone SIL post-attach ring shows a slowly growing 1.5 Hz rock (4 -> 10 deg in 30 s) not seen in the Gazebo 4-ring hover R0510: Gazebo decides.
CLOSED  The collaborator's approach MPC (controller_mpc_payload) is unstable on the linearised sim plant (R0522: throttle to the 0.05 floor, 2 Hz roll oscillation, flip 7.7 s after ATTACH; kt_solo 83.1 vs ~30 before). Either the collaborator retunes it for a = kT u, or the sim demo keeps the tracker-flown approach built 2026-09-25 (enable_approach_mpc:=false). Wesley's call, and the rig's approach controller is the other student's anyway.  -> CLOSED 2026-09-25 by Wesley: retune when the collaborator's updated code is integrated; the tracker-flown approach is the sim path meanwhile.
CLOSED 2026-10-08 (Wesley: carriers 270/30/150 on plates 3/11/7, weld on plate 9 at +y 0.225, rig-prep 3a37400; the arm length is measured Wednesday)  Rig attach geometry for the rejoin: the demo's 330/90/210 + weld at 270 is the 30/150/270 + rejoin at 90 layout rotated 180 deg, i.e. the same plates (1/3/5/9) as the detach study; confirm the magnet arm length (0.5 typed) on the fourth airframe.
OPEN  SIL plant vs Gazebo on UNEVEN 4-rings: SIL rocks 330/90/210/270 at 1.4 Hz growing (R0524, no attach involved; R0518/R0520 after an attach) while the even ring is steady (R0525) and Gazebo flies the same shape level (R0510/R0512). Rod/ball-joint damping is the suspect. Until settled, uneven-ring claims come from Gazebo; SIL keeps the transient A/Bs.
DECIDED 2026-09-25  Sim Betaflight rate estimate timed from the pose stamp, not the throttled ROS clock (bug since the 09-23 clock_hz 100 default): a free x3 was un-flyable under every controller (R0522, R0526-R0534), fixed in R0535 with clock_hz 0 and then in the three sim Betaflight nodes. Tethered baselines since 09-23 flew on the weak inner loop; re-fly the creep n3 hover once as the regression.
CLOSED  rviz_quad_load_launch clock_hz default (100 since 09-23 for CPU relief): a FREE drone cannot fly under it (R0538: 0.5 m wobble, 5 deg) and is perfect at 500 or 0 (R0539/R0535); tethered drones are cable-damped and never showed it. The attach configs carry clock_hz 500. Wesley's word: make 500 the default (every sim run), or keep 100 for carry-only runs.  -> CLOSED 2026-09-25: default 500 on Wesley's word (sim-only; no rig launch throttles /clock).
CLOSED  Post-weld flip of the OCP-attached newcomer in Gazebo (R0541/R0547/R0548/R0549: flips 2-5 s after the weld; SIL R0546 holds the same weld at 2.1 deg with the reference slew). Cleared: the immediate yank (slew), the tip-vs-rim offset, the slack start, the arm's joint damping. Left: the arm pivot 0.05 m below the body centre vs the OCP's centre-attached cable, a rod that can push, and the OCP's inconsistent horizon during the swing (planned z falling with positive planned vz; tautness gate 0.46). Wesley's call: the newcomer's arm model (SDF) or a different post-weld handling (e.g. hold the newcomer's rod near vertical with a small tension and let the incumbents carry it until the circle resumes).  -> CLOSED 2026-09-25: the 0.05 m arm-pivot lever (R0552 with the pivot centred: clean). Follow-up is the OPEN line on modelling the lever.
DECIDED 2026-09-25  rviz_quad_load_launch clock_hz default 100 -> 500 (Wesley's word: fine as long as the real world is unaffected; it is sim-only: the rig launches carry no /clock throttle). A free drone needs it (R0538 vs R0539).
DECIDED 2026-09-25  Post-weld flip of the OCP-attached newcomer = the magnet arm's pivot 0.05 m below the body centre (R0552 with the pivot at the centre: full rejoin chain clean, peak 11.6 deg, settled 0.8 deg; R0548 identical otherwise: flip at +3 s). Wesley: NO hardware change, every drone hangs its cable from a magnet at the tip. So the control side must carry the lever: OPEN below.
CLOSED 2026-09-25 Model the tether pivot offset (rod force acting below the body centre) in the newcomer's tracker/OCP, or make the post-weld transient gentle enough for the unmodelled lever (longer slew, rod held near vertical until the circle resumes). Note the rig: where each airframe's tether hooks relative to its CoM sets the lever for ALL four drones (measure it).  -> Wesley: the newcomer's arm pivots at the body centre like the tethered drones (canonical x3_drone3_magnet_rod.sdf changed), rejoin straight to 45 deg; Gazebo R0575/R0577 clean (peak 10.3-10.5, 0.8-0.9 settled with the circle resumed on four). Rig: measure where each airframe's cable hooks vs its CoM.
DECIDED 2026-09-25  Collaborator's approach MPC: retune later, when their updated code is integrated (Wesley). The tracker-flown approach (approach.py, enable_approach_mpc:=false) is the sim demo path meanwhile.
DECIDED 2026-09-25  Wesley reviews and commits the 2026-09-25 work himself tonight; nothing staged.

DECIDED 2026-09-25  Newcomer arm pivot at the body centre in the sim (Wesley), consistent with the three tethered drones; attach = OCP resize straight to 45 deg with a 10 s reference slew (R0575, R0577).DECIDED 2026-09-26  Tracker reference-staleness gate in Gazebo: 3.0 s via the sim-only launch arg safety_ref_timeout_s (dissipative_launch; default 1.0 = the rig, unchanged). Wesley's word after the GUI M2 run W-21:08: planner slow ticks of up to 0.8 s at ~0.1 RTF tripped the 1.0 s wall-clock gate mid-lift.
DECIDED 2026-09-26  Sim rate-loop I-term (rate_ki 10, anti-windup clamp 200 / u > 0.09) becomes the default in every sim Betaflight bridge; re-fly the baselines once each and record the shift (Wesley). Evidence: S0001-S0004, T0011/T0012, T0015.
DECIDED 2026-09-26  Damped universal tether/rod joints: test on the weld bench first (test copies), adopt only if it clearly helps (Wesley). DART ignores ball-joint <axis> damping (S0005-S0008).
DECIDED 2026-09-26  Tejen-integration branch: source and worlds only (src/, simulation_assets/ minus simulation_assets/tejen/logs); tools/docs/configs stay untracked; Wesley commits (Wesley).
DECIDED 2026-09-26  Loop run budget: no cap on headless runs for the GOALS.md loop; report at milestones (Wesley).
DECIDED 2026-09-27  Partner drone's magnet rod on universal joints at both ends (shipped x3_drone3_magnet_rod_partner[_attached].sdf; damping 0.002 body end, 1e-4 tip): DART ignores ball damping, the arm swung ~80 s before the weld gate passed and the gz weld copied the tip's leftover spin onto the ring. Rod twist blocked (drone 3 yaw coupled while welded); the real rod's freedom unchecked (Wesley). Evidence: R0660/R0662, R0668-R0680, M-yank, R0684-R0687.
DECIDED 2026-09-27  Rejoin defaults flipped (launch + node): attach_t_start_new 0.1, attach_blend_balanced true, attach_datum_shift true (dissipative node, also on the rig attach path), weld_velocity_clock sim (magnet manager) (Wesley). Evidence: benches R0663-R0666, R0672-R0677; M1 claim R0685/R0686 (card 2026-09-27_m1_rejoin.md, reviewer SUPPORTED).
DECIDED 2026-09-27  Sim rate-loop I-term ki 5 on every bridge (10 sat on the 3.2 Hz PI zero: R0647 limit cycle on our X3, T0017 ring rock on his). M2 claim T0018/T0019 reviewer SUPPORTED.
DECIDED 2026-09-27  G1b (0.1 kg object): schedule Tejen's MPC kT on the object-attached flag (kT*m_d/(m_d+m_obj) while attached) instead of his UKF (too slow to recover after the drop, critic on card 2026-09-27_g1b_mass.md) (Wesley).
DECIDED 2026-09-27  Defaults flipped (launch + node): z_ki_in_orbit true (planner height integral also in a slow level orbit, OCP path), attach_moving true (trajectory keeps running through the rejoin; approach tracks the plate with velocity feedforward), attach_approach_direct true (no climb when handed over above the plate) (Wesley). Evidence: R0692/R0693 (bench A/B), R0694, combined M1 claim R0696/R0697 (card 2026-09-27_m1_full.md, reviewer SUPPORTED), R0698. Canonical partner_attached_orbit.yaml LAND at weld+8 s. R0676 was VOID (start-up race), not a falsification.
DECIDED 2026-09-27  The partner mission's object is 0.1 kg by default (payload_model.sdf mass 0.1, solid-sphere inertia 2.56e-4; partner_mission.launch.py object_mass_kg 0.1 -> his MPC kT 71.6 while carrying) (Wesley). Evidence: combined M1 claim R0696/R0697 (card 2026-09-27_m1_full.md). Sim geometry kept at 0.08 m (the real ball is ~0.029 m).
DECIDED 2026-09-27  Keep the sim Betaflight gyro rate-loop option (rate_source imu, payload_betaflight_comm; commit 1093b89) as an opt-in, OFF by default (rate_source pose stays the default) (Wesley). Evidence: R0705, no clear effect on the carrier kicks (7 vs 7, ring p99 2.04 vs 2.16); kept for plant realism, inert when off.
DECIDED 2026-09-28  Sim Betaflight bridges run their rate loop on the simulated gyro by default (rate_source imu), for Tejen's bridges on the M2 path AND ours (Wesley: "yes make the gyro the default"; ours "as long as it works in the real world" - the bridges are sim-only, the rig's rate loop is Betaflight's own on its gyro). Evidence: loaded M2 bench pose 3/6 rock vs gyro 0/6 (R0709-R0720); full M2 on the gyro T0028/T0029 no sustained rock. Changes every sim baseline: re-fly each baseline once and record the shift.
DECIDED 2026-09-28  M2 ring-tilt bar is 5 deg (was 3 deg), per sample, through sweep, lift, hover and LAND (Wesley). Set after T0028/T0029 (3.44/3.33 on short transients); the claim is re-flown on fresh runs, not re-scored from T0028/T0029.
DECIDED 2026-09-28  Rig layout: plates 1/3/5/9 for four drones, 1/5/9 for three (Wesley). Closes the 4->3 detach layout OPEN line (line 25) for the rig.
DECIDED 2026-09-28  Real M1/M2 launches are a `real:=true` mode on the sim launches M1/M2 already fly (three_attach_launch, dissipative_launch), not re-synced separate real_* files (Wesley).
DECIDED 2026-09-28  First real rejoin weld (G7 rung R8b) go/no-go bar <= 10 deg; the thesis claim runs keep the sim's 5 deg bar (Wesley).
DECIDED 2026-09-28  Abort behind the mux (Q6a): an operator ESTOP or a fault on a flying drone grounds EVERY drone, the partner's included (mux latch; card 2026-09-28_mux_abort_latch.md rewritten per the critic, critic again before building) (Wesley).
DECIDED 2026-09-28  An ARM failure of our stack refuses TAKEOFF and disarms our drones but no longer raises /fleet/abort (Q6b) (Wesley).
DECIDED 2026-09-28  Rig mode (real:=true) defaults to the creep floor start (start_taut false, handover_elev_deg 45, handover_settle_s 2.0, creep_vel 0.2) and reconfig_mode ocp, honouring typed values; sim defaults unchanged (Q7) (Wesley).
DECIDED 2026-09-28  Commit the rig magnet path (P10: one radio-latch path per drone, attach_magnet_initial auto, detach releases); magnet ownership in a combined M2 rig run still to agree with Tejen (Q8) (Wesley).
DECIDED 2026-09-28  Floor-start M1: drone 3 creeps as the 4th carrier (resize to n=4 before TAKEOFF, creep sized n=4), option (b) (Q9) (Wesley).
DECIDED 2026-09-28  Pursue the carrier kicks now with the sim oracle test (tracker x0 body rate from the simulated gyro on the carriers), card + critic first, 1-2 clock-0 runs approved (Q10) (Wesley).
DECIDED 2026-09-28  Hardware: our drones 0-3 are Tejen's quad1-4 (drone i = quad i+1), all four ready; kill switch = the disarm button; cage about 4 x 4 x 3 m (orbit r 0.5 and the 1.8 m transit fit) (Wesley).
DECIDED 2026-09-28  Abort scope: faults of a tracker whose drone is NOT under our command (its mux still forwarding the partner) do not fire the fleet abort; operator ESTOP and faults of drones we command still ground everyone (Wesley, critic pass 2 F5 / R0618).
DECIDED 2026-09-28  The operator's disarm button (panel/spacebar DISARM, the rig kill switch) grounds EVERY drone: it latches every mux like ESTOP (drones still on the partner's software included) until the stack is relaunched (Wesley).
DECIDED 2026-09-29 (Wesley)  A tracker that disarms between ARM and TAKEOFF grounds our fleet like a failed ARM: service disarm, no /fleet/abort, TAKEOFF refused (GOALS Q11, card 2026-09-29_pre_takeoff_disarm_gate.md v2; R0749c).
DECIDED 2026-09-29 (Wesley)  Carrier kicks: fixed from the R0b rest bag (P9 rate estimate, rig first, then the same fix in sim); until then the M2 claim is worded "met when no kick lands in the lift" (T0035; GOALS Q10).
DECIDED 2026-09-29 (Wesley)  Drone 3 gets the same single command path as the carriers: its tracker no longer takes /fleet/command (the manager arms it and sends TAKEOFF), the manager manages n + 1 only with the approach chain, and the superseded real_attach_launch.py refuses to start (its newcomer took TAKEOFF from the broadcast). SIL R0753: drone 3 armed via service, TAKEOFF from the manager, weld and fold-in as R0578.

2026-09-30  Sim plant matches the rig: affine thrust + voltage sag, drone 0.55 kg, rod 0.55 m pivoting 4 cm below the drone centre, magnets at r 0.225; key baselines re-flown once (Wesley, after the rig day; docs/archive/rig_2026-09-30_review.md).
2026-09-30  No pull freeze at breakaway: the tethered model is to be corrected so breakaway is near 100 % of the planned pull (Wesley).
2026-09-30  Rig height integral = small safety net (z_ki 0.4, bound 0.15 m, gate 0.25 m); the model is judged with it off (Wesley; supervisor: the integral must not stand in for the model).
2026-09-30  Rig thrust map: throttle = offset + 0.507*mass - 0.022*(V - 23.5), offsets 0.181/0.185/0.187/0.187 (RIG-0930-ladder4); thrust_ratio 35.2 above the offset, kt_trim off on the rig.
2026-09-30  Rod joints: both ends close to free ball joints (pivot 4 cm under the drone centre, magnet on the plate); hardware unchanged for the next visit (Wesley).
2026-09-30  Plan docs/archive/plan_2026-10_tethered_model.md (Wesley, evening): rig keeps the breakaway freeze as its default until the airborne force deficit (~80 % of the ring weight) is explained; sim/flight logic can drop it behind a launch arg. Masses are trusted as measured (ring 0.86, drone 0.55, hung 98/196 g): no re-weighing.  [superseded 2026-10-03 by B2, below]
2026-09-30  Lab PC compute: Docker (Humble/Harmonic parity image) on the lab workstation, no headless run cap there (at most 4 in parallel, <= 16 of 24 cores for other users); the laptop keeps 10 per session (Wesley).
2026-09-30  Attach/M1/M2/weld worlds and Tejen's sim drones stay on the legacy geometry and linear thrust map for now, pinned explicitly; only the carry/floor-start worlds become rig twins (Wesley).  [2026-10-05: still pinned, now as the legacy profile (legacy:=true, worlds in simulation_assets/old_worlds/); porting them heads docs/archive/redo_on_twin.md]
2026-09-30  z_taut_gate 0.6 on the rig is allowed as a fallback if the planner pivot fix is not in, so the height safety net can open (Wesley).
2026-09-30  Lab PC parallelism: no fixed cap on parallel runs while nobody else uses the machine (about 6 slots on 24 cores); back off when other users are logged in or loading it (Wesley).
2026-10-01  After the rig day (Wesley, plan docs/archive/plan_2026-10-02_rig_fixes.md): drones are SHOWN as 1-n everywhere a person reads (panel, manager/tracker/planner log lines, preflight, fleet monitor); topics and CSV columns stay 0-based. Preflight resting tilt limit 20 deg (was 15). Figure-8 on the rig at a = 1.0 m (command line; the 0.5 launch default is shared with the orbit). Rig launches wait for each flight controller's own armed report (require_fc_armed, default true on real_* launches; elrs_interface retries a refused arm with a fresh switch edge, up to 3 times). real_io_launch serial default is now /dev/QUAD<i+1>, as real_mode.py.
2026-10-03  Wesley (plan noble-noodling-lollipop §3, approved 3 Oct)  Rename: controller_load_mpc -> mpc_planner (executable/node mpc_planner, was planner/load_planner); controller_quad_load split into tracker (executable tracker, node tracker_<i>, was controller/controller_<i>), fleet_manager (was main/central_controller) and bringup (launch files, rviz_config, real_mode); controller_dissipative -> dissipative_planner (was dissipative/dissipative_controller). Logs: logs/tracker/drone<i>_<stamp>, logs/mpc_planner/<stamp>, logs/dissipative_planner/<stamp> (tools/run_logs.py reads both layouts). Deleted: controller_ukf, real_attach_launch.py, unused sim bridges/launches. Shell shortcuts in shortcuts.md (gitignored), sourced from ~/.bashrc. Topics, parameters, message types unchanged; history keeps the old names. Verified: SIL R0840 bit-identical to R0796; old runs reload with identical metrics. Full map: CURRENT_STATE.md §12.
2026-10-03  Wesley  real_dissipative_launch.py takes real_control_launch.py's rig defaults: cable_len 0.55, drone_mass 0.55, thrust_ratio 35.2 with the affine map (thrust_offset 0.185, thrust_offset_v_slope 0.022, thrust_v_ref 23.5), kt_trim false, throttle_max 0.8, lift_ramp_vel 0.1, num_drones 4, and the args it lacked (attach_radius 0.225, attach_z 0, pivot_offset_z -0.04, solve_budget_s 0.06, pretension_s 3.0, ff_cap_release_s 2.0, rod_tol_frac 0.25, rod_spread_m 0.08, z_i_gate 0.25); detach defaults reconfig_mode ocp, detach_magnet true. Before this a bare launch flew kT 24 linear, no pivot model, rod 0.5, ring r 0.25 (node defaults). Pinned by test_rig_detach_launch_flies_the_rig_control_values; desk dry-launch read-back matches. The pivot model through an OCP-resize detach is unflown in sim: the W4 twin detach rehearsal must fly pivot_offset_z -0.04.
2026-10-03  Wesley (plan noble-noodling-lollipop §3b)  Launch consolidation: five launch files in bringup/launch (sim_io, sim_control mode:=mpc|free_hover|dissipative|network|attach, real_io, real_control mode:=mpc|free_hover|dissipative|attach|m2, sim_m2_bench). Defaults moved unchanged into bringup/config/{common,sim,real}.yaml (per-mode entries); launch arguments = the knobs set in >= 3 configs or commands plus the structural ones (bringup/profiles.py KEPT_*), every other knob through params_file:= (the harnesses write <run>/params_file.yaml); a typed non-argument is refused. Equivalence gate (tools/launch_gate.py, results/launch_gate_2026-10-03): 360/360 frozen old-launch commands give identical nodes and typed parameters, 310/310 config cases through the harness; SIL R0843 bit-identical to R0796. The sim detach mode also takes the rig-twin knobs when typed ('' = not passed), for the W4 detach rehearsal. Module renames: tracker/tracker_node.py, fleet_manager/fleet_manager_node.py.
2026-10-03  Wesley  Every rig mode flies the rig defaults (real.yaml base = the rig carry's): M1/M2 on the rig (real_control mode:=attach, mode:=m2) and rig free_hover dropped their sim/old overrides (cable_len 0.5, drone_mass 0.64, kT auto or 24 on the linear map, kt_trim on, lift_ramp_vel 0.22, traj_speed 0.6, m2 attach_z 0.025); the attach/m2 graphs now pass the rig thrust map (thrust_offset 0.185, slope 0.022, v_ref 23.5, throttle_max 0.8) to every tracker incl. the newcomer, and attach_radius 0.225, pivot -0.04, pretension 3.0, cap release 2.0, rod band 0.25/0.08, z_i_gate 0.25, solve_budget 0.06 to the planner (real_mode.map_and_geometry); rig free_hover gets throttle_max 0.8; the rig hand-over hold is 1.0 s everywhere (real_mode RIG_DEFAULTS, was 2.0). Kept: the attach geometry (330,90,210 carriers, weld offsets: OPEN line on the rig attach geometry) and the attach architecture (velocity_after_handover, vel_ki 0, diss_ki_load 1.0). Not flown (Wesley: update, no test); offline: launch gate 350/360 unchanged, the 10 changed cases are exactly the rig attach/m2/free_hover ones.
2026-10-03  Wesley (plan noble-noodling-lollipop §7)  B2 is the rig default: the breakaway cap is released to full pull over ff_cap_release_s 2.0 s once the lift ramp starts (9fc724b), replacing the 30 Sep "rig keeps the breakaway freeze". Twin: hold 1.033 m (R0834/R0836) vs 0.768 with the cap held (R0838/R0839), ring vz max 0.21-0.25 m/s; reviewer SUPPORTED on height. Not yet flown on the rig (Wed F1/F2).
2026-10-03  Wesley  W2 LAND unwind kept behind land_unwind (default off): it removed the rod shove after load down (outside the reference 0.5-0.7 cm vs 15.8-20.0, tilt after the blend 2.9-3.1 vs 4.5-5.6 deg; R0848/R0849/R0851 vs R0841/R0844/R0813) but missed the card bar (off-reference 10.1-12.9 > 8 cm) through lag during the 1 s LAND feedforward blend; next: the unwind starts after the blend, one Gazebo run.
2026-10-03  Wesley  W3 values: B3 latches the rod elevation measured at the hand-over as the nominal angle for s_ref and the nominal tension (new knob, default off); stiffness arms raise the planner ring-position xy weight 60 -> 180, then 600 if a 1 N push still moves the ring > 3 cm (new knob, default 60).
2026-10-03  Wesley  Wednesday 7 Oct starts on the even ring, plates 0/3/6/9 (the rig default, nothing typed), and moves to 1/3/5/9 only if that looks good; the xy error fix (W3) comes before the layout change. Supersedes "1/3/5/9 all day" in plan noble-noodling-lollipop.
2026-10-03  W2 follow-up "unwind starts after the blend" reverted the same session: R0852 disarmed the fleet 0.58 m up (the held reference stopped the descent and the touchdown stall detector read it as landed). W2 stays as flown in R0848-R0851 (arc from load down), default off.
2026-10-03  Wesley  W3 (card 2026-10-03_xy_stiffness_b3): ring-position weight x3/x10 cut the 1 N push offset only 23/38 % (18.5 -> 14.3 / 11.4 cm, x10 rocks the ring 6.0 deg; R0856-R0858) and B3 made it worse (20.7 cm, R0860): both knobs deleted the same session; the WRENCH push tool and push worlds stay. Wednesday drops F3/F4.
2026-10-03  Wesley  Next xy (and height) fix: a principled disturbance estimate (B4) rather than an integral term, even if it takes longer (better thesis argument). The existing B4 card (2026-10-01_load_ext_force.md) is NOT READY per its critic; its replay points to a world-fixed -0.4 to -0.5 N on the rig (mocap level or air, acting on every body), so the mocap level check on Wednesday comes first.
2026-10-03  Wesley  Payload position control: try several principled options, A first (offset-free MPC: ring-borne and drone-borne constant forces estimated from measured motion and delivered thrust, fed to the planner model; card 2026-10-03_offset_free_mpc.md, critic before any code). Kept aside for later: per-drone force observer in the tracker (TU Delft-like), adaptive control on the ring (L1/MRAC), a ring xy integral, cost retuning. The mocap frame is not tilted (Wesley).
2026-10-03  Wesley  Option A after the critic (NOT READY, 11 must-fixes in the card): observer only for now (no new integral); other methods for the internal model error to be tried after it is implemented. Must-fix 1 (offline replay of the d_L/d_D/tension split on existing runs and rig logs, no flying) approved and started.
2026-10-03  Wesley  After the critic on option C (NOT READY: C reduces to an integral-like linear controller; the twin +4.8 cm hold most likely comes from the rod weight booked on the drones): both, F first (model fixes: rod mass in the planner model, per-drone thrust offsets), then C reworked per the critic's must-fixes. C stays sim only; card 2026-10-03_l1_ring.md.
2026-10-04  Wesley  Overnight loop (3 Oct night): find a literature-backed control-law change for the ring's internal and external model errors, try as many methods as needed and compare them against z_ki 0, z_ki 0.4 and option A, sim only (laptop), offline -> SIL -> Gazebo, a card and the critic per method; losing methods stay behind off-by-default knobs until he decides. The offset-free observer (card 2026-10-04_offset_free_innovation.md) is integral action through the planner model: rig use needs his decision against the 2026-09-30 safety-net line and the 2026-10-03 observer-only line.
2026-10-04  Wesley  Tracker node-0 box at zero is the default (x0_relax_frac 0.0, was 0.025; common.yaml and the tracker node, sim and rig): each tracker's MPC starts from the measured state, the textbook initial condition. The 2.5 % box was the ring's internal height offset: SIL +6.76 -> -0.12 cm (R0956/R0957, z_ki 0), Gazebo floor-start hold +0.9 cm with clean takeoff, lift and LAND (R0960, one run). Time shift R0958 and w_thr_ref 10 R0959 did not move it. Neighbour arms (push/wind with A, map+-, orbit, n=2/4, attach/detach) not yet flown with it; the rig not yet.
2026-10-04  Wesley  land_unwind is the default (common.yaml, sim and rig). LAND matrix R0987-R1008 (card 2026-10-04_land_matrix): the rod shove is gone (0.5-0.6 cm vs 17.5-19.8 in 16 clean landings) but 2 of 18 aborted: R1003 (survivor: the cable feedforward steps off at load-down and the attitude oscillates; R0981 without the unwind is the same) and R0997 (departed drone tracking 10 cm below the floor). Feedforward ramp + ground idle proposed; decision pending.
2026-10-05  Wesley  land_ff_ramp and ground_idle are the defaults (common.yaml, sim and rig): 18/18 clean landings in the fix matrix (card 2026-10-04_land_fix, R1009-R1034) and every landing since (T9 R1038-R1067, line/spin R1068-R1071).
2026-10-05  Wesley  The model integral is the default everywhere, the rig included: int_mode auto (common.yaml) flies model with the rig twin's stiffness (int_k_xy 5.26, int_k_z 7.13 N/m; card 2026-10-04_z_int_model) and falls back to reference where model is refused (kt_trim on, a fleet that can attach or detach, z_ki 0, offset_free on, dist_est ring/full); a typed int_mode model is still refused there. The rig carry flies model, the rig detach/attach/m2 modes reference.
2026-10-05  Wesley  z_i_gate stays 0.25 m for now: a +20 % mass error misses by 25.8 cm and the integral never starts (R1039/R1044); stated as a limit in the thesis (sim_robustness.tex); revisit after the rig.
2026-10-05  Wesley  Rig day 7 Oct, in order: tethered hover and LAND (epsilon 0, landing fix, model integral), circle, figure-8, then the 4->3 detach on plates 1/3/5/9.
2026-10-05  Wesley  The rig twin is the simulation: bringup/config/sim.yaml carries the twin's values (rig geometry, masses, affine thrust map, ff_cap_force 0.75, 3 drones) and sim_io_launch defaults to the rig plant. The legacy worlds moved to simulation_assets/old_worlds/ and fly on sim_legacy.yaml with legacy:=true (the runner infers it from an old_worlds/ or tejen/ world path). The SIL bench and mode:=attach / mode:=network stay on the legacy profile until ported. Thesis: legacy-flown results are \redo boxes, legacy descriptions deleted; kt_trim and the first height-integral test are cut, not redone. Everything to fly again: docs/archive/redo_on_twin.md.
2026-10-07  Wesley  Rig LAND straight down after touchdown: land_unwind false in real.yaml (carry and detach). The rig tethers are cables that go slack; the unwind fixed the twin's rigid links (R0793/R0816), not a rig problem (30 Sep / 1 Oct rig landings flew straight down without a tip). The twin keeps land_unwind true until its tethers are modelled as cables.
2026-10-07  Wesley  Rig carry default (real.yaml modes.mpc): cable_source mocap (cap 12) and a 60 deg plan (cable_elev_deg, handover_elev_deg 60). Against round 1's feedforward on the same paths: circle path error 8.25 -> 2.91 cm, lag 0.45 -> 0.20 s, height sd 4.8 -> 0.5 cm (r006 vs r206e60); figure-8 8.2 -> 3.0 cm, 0.41 -> 0.21 s, 4.7 -> 0.6 cm (r0007 vs r2008e60). At 45 deg the measured pull flew the cables at the plan and a magnet let go (r200006; Wesley, 7 Oct evening: the magnets hold about 9-10 N, the release came from a sharp jerk, not the steady ~3 N). Carry only: detach, attach, m2 and the twin keep model / 45 until flown with it. Feedforward for comparisons: params_file configs/rig/feedforward_45.yaml. Twin run waived by Wesley.
2026-10-07  Wesley  Measured pull at 60 deg is the default everywhere (common.yaml: cable_source mocap, cable_accel_cap 12, cable_elev_deg 60, handover_elev_deg 60): rig carry, detach, attach, m2, free hover and the twin. Supersedes the carry-only line above. Only the legacy profile (sim_legacy.yaml) keeps model / 45 so old runs reproduce. Detach, attach, m2 and the twin have not flown it yet; the rig's freed drone flew on it for ~5 s after the r200006 release. Twin baselines from before 7 Oct were flown with model / 45.
2026-10-07  Wesley  fig_8 is timed by distance: traj_speed is the constant path speed between 4 s smootherstep ramps (as orbit), replacing the single eased sweep over the lap (0.03 m/s near the ends, traj_speed only at the centre crossing; mean 0.09 at 0.25). Rig figure-8 at 0.125 m/s for now (lap 52.8 s at a 1.0); higher speeds later. Earlier figure-8 runs (RIG-1007 r0007, r2008e60; twin R1075, R1107) used the old timing and are not comparable to new ones.
2026-10-07  Wesley  A detached drone steps 0.5 m radially out from the ring centre, then holds: detach_step_out_m 0.5 is the default (common.yaml; legacy profile keeps it off), for safety, sim and rig.
DECIDED 2026-10-08  Drop and land (Wesley 8 Oct): in detach mode, a loss that leaves fewer than 3 drones, or ring tilt over 30 deg, drops the ring and lands every drone instead of the fleet-wide disarm; amends the lock "fleet-wide disarm on any envelope fault" for detach mode only (carry keeps the disarm; the 60 deg envelope stays). Card docs/experiments/2026-10-08_drop_and_land.md; behind drop_on_loss on branch safety-defaults until the twin verdict and Wesley's go; the the working rules lock line changes at the merge.
DECIDED 2026-10-08  Drop and land, morning (Wesley): the trackers' plain 30 deg rule (2 samples), not the replay's rate rule; live on week4 for today's rig detaches through drop_on_loss in the rig detach configs (detach_drop.yaml, drop_and_land.yaml, detach_unannounced_orbit_drop.yaml), ported without the other safety-defaults changes. Week4 port twin check R1204-R1213, all as expected (card section "Week4 port"). the working rules lock line amended the same day.
DECIDED 2026-10-08  Model integral on every run (Wesley, evening): a fleet that can attach or detach flies int_mode model too; the learned ring force is kept through a resize and frozen in the reconfiguration hold (the integral half of f2f7d74 from safety-defaults; its default mode dissipative and detection-on defaults NOT taken). Rig evidence: every dissipative detach run on the reference integral sat 5.2-6.5 cm off its hover point in xy (r0012, r302_3, r303_4, r304_3, r305) against 1.9 cm for the model-integral circle r206e60; r305's circle with that mean offset and the lag removed tracked to 2.4 cm 3-D RMSE (6.8 cm raw). Twin port check on week4 R1217- (running). Kept on reference: legacy worlds (kt_trim on) and z_ki 0.
DECIDED 2026-10-08  Defaults, evening (Wesley): dissipative is the default mode and unannounced-detach detection (0.06 m), drop and land and a freed drone landing by itself are on by default, everywhere; legacy:=true keeps mpc with all of them off; the attach graph keeps drop and departed_land off until the drop covers a welded newcomer. Standing rule: what is shown to work better becomes the default. Old twin configs pinned to the old values (rig-prep 57912f4).
DECIDED 2026-10-08  Lag (Wesley, after the supervisor's questions): report it, do not fix it. The thesis and rig reports give path error, lag in seconds and height error separately; the half-speed circle (card 2026-10-09_half_speed_circle) is the evidence. ref_time_shift and the time-cascade branch are parked.
DECIDED 2026-10-08  M1 authority and safety (Wesley): LAND lands every drone whatever controller has it (while Tejen's MPC flies QUAD4, LAND goes to his land_now, falling back to our departed path); DISARM disarms everything; a fault causes a landing, not a disarm; an unwelded newcomer on LAND steps 1.2 m clear and lands, and a fault on it alone lands it while the carriers fly on. Combined stack: drone 3's mux boots on ours (his MPC only after /partner/release), his supervisor automatic, one writer for QUAD4's magnet (magnet gate), our mocap node the only Motive owner, his aux channels 5 and 7-10 zeroed, floor start with all four for R10b/R11, everything on the rig laptop, each side keeps its own lengths and calibration, kT on QUAD4 = the others', weld radius from a props-off measurement, pose gaps in his phase logged with no code. His code may be edited as needed with every change logged in src/tejen_mission/CHANGES_FROM_TEJEN.md. M2 decisions later.
DECIDED 2026-10-08  Free M1 drone = QUAD4 = /drone_3 ("drone 4"), body 14, magnet tip 24 (Wesley). The rig attach rungs are gated on twin runs in a rig-twin attach world (to be built).
DECIDED 2026-10-08  M1 rig attach geometry (Wesley, evening): carriers drones 1/2/3 on plates 3/11/7 (270/30/150 deg), QUAD4 welds onto plate 9 (+90 deg, ring +y) with attach offsets 0 / +0.225 m in the ring frame (the magnets' radius), handover elevation 60 deg in attach mode as everywhere (the rig attach forced 45). real.yaml modes.attach and RIG_DEFAULTS (rig-prep 3a37400).
DECIDED 2026-10-08  Mocap body rates at full precision (Wesley, evening): our mocap node rounded the filtered body rates to 0.1 rad/s, the trackers' x0 on every rig flight to 8 Oct; removed for all rig runs (rig-prep 96acbcf), as Tejen did in his copy (4b34d768).
DECIDED 2026-10-08  Twin attach worlds (Wesley, evening): build a rig-twin newcomer (QUAD4: rig-twin drone, 0.55 m rod with the pivot 4 cm below, magnet joint at the tip, tip ~0.58 m from the body) with a free-start world (carriers on plates 3/11/7, QUAD4 parked on the floor) for R8a0-R9 and a welded floor-start world for R10b/R11.
DECIDED 2026-10-08  [SUPERSEDED later the same night, see 'Newcomer fault: land everything'] Newcomer fault scope (Wesley, night): a fault on an unwelded QUAD4 (its tracker cuts its own motors: tilt > 60 deg, pose lost > 0.25 s, reference stale > 1 s) disarms QUAD4 alone and the carriers fly on, instead of the fleet-wide stop; its own card. A tracker "fault lands QUAD4" mode follows as a separate piece of work. Amends the fleet-wide-disarm lock for an unwelded newcomer only.
DECIDED 2026-10-08  Newcomer abort path (Wesley, night): on OFF, or LAND before the weld, QUAD4 moves radially outward from the ring to 1.2 m at its current height (no rise: a climb can itself collide), then lands by itself; the magnet goes OFF first and the step waits for the release.
DECIDED 2026-10-08  Body-to-tip on the rig (Wesley, night): the approach and weld capture use rod + pivot drop everywhere once the twin dry approach shows the tip within 1 cm of the target (twin-attach 08d9712).
DECIDED 2026-10-08  Newcomer fault tests (Wesley, night): a push variant of the free-start twin attach world (ApplyLinkWrench added by tools/make_attach_worlds.py; existing worlds unchanged) and a runner event that stops QUAD4's simulated mocap (pose-loss fault).
DECIDED 2026-10-08  Creep fix (rig-prep 6bfb881) supports in the twin direct test: R1242 on 6bfb881^ reproduces r303 (sweep timed out before TAKEOFF, ring never lifts), R1241/R1243 on the fix lift 9.91/9.83 s after WAIT_LIFT with the anchors latched at TAKEOFF. Wesley keeps relaunching T2 between rig flights anyway (night): the sheet rule stays.
DECIDED 2026-10-08  Newcomer fault: land everything (Wesley, late night, replacing 'the carriers fly on'): "if quad 4 fails we just land everything because we need to do the next run anyway". A fault on an unwelded QUAD4 (its tracker has already cut its motors) makes the fleet LAND instead of the fleet-wide disarm, anywhere in the approach; a fault after it is folded in stays the fleet-wide stop. Built as newcomer_fault_land (card 2026-10-09_newcomer_fault_scope rev 2).
DECIDED 2026-10-08  Monday freeze (Wesley, late night): at 18:00 on Mon 12 Oct merge rig-prep into week4 if the twin gates pass and the reviewer signs off the tables; anything that failed stays off week4; Wesley pushes.
DECIDED 2026-10-09  LAND lands every drone safely, whatever it is doing (Wesley, morning): the LAND bug (an unwelded QUAD4 disarmed in mid-air, R1245-R1249) must be fixed before any rig attach rung; the fix (newcomer_lifecycle: magnet OFF, radially out to 1.2 m at its height, lands by itself) becomes the default once a LAND-in-every-phase twin matrix and the reviewer pass. Supersedes the sheet-rule alternative ("no LAND while QUAD4 is unwelded and airborne").
DECIDED 2026-10-09  No day deadlines (Wesley, morning): the plan is an order of work, not a timetable; tasks take the time they need; what has passed its gates by a lab day flies, the rest waits for the next one.
DECIDED 2026-10-09  Approach on the measured magnet (Wesley, morning): the newcomer should use the mocap magnet tip (body 24) against its target to correct its approach (Tejen saw the magnet right over the plate but not low enough); new code in the flight loop: card + critic, twin with a wrong rod length (+/-4 cm) and a swinging rod.
DECIDED 2026-10-09  Freed drone landing by itself (Wesley, morning; W4/W5 and any drone off the ring): pushed down to its own resting height (departed descent with the 6 s press), then the fleet manager disarms THAT drone alone, as an expected disarm, not a fault; no armed time at zero throttle (Betaflight airmode stays active after a flight). New code: card + critic, twin. Replaces armed idle for a departed drone.
DECIDED 2026-10-09  QUAD4 failing over its plate (C1b, ring tilt ~57 deg in the twin, R1262/R1286): accepted for R8b as is (Wesley, morning).
DECIDED 2026-10-09  No rig code freeze (Wesley, morning; replaces the Monday 18:00 freeze): each item merges into week4 when it passes its gates, up to Wednesday morning, with Wesley's OK per merge; the desk rehearsal runs on the current tree and any item merged after it gets its read-back line re-run before it flies.
DECIDED 2026-10-09  Magnet feedback (Wesley, afternoon, after the critic's REWRITE and my recommendation of a one-shot trim): calibrate the setpoint (Wed 09:00 props-off, magnet face on plate 9, tip marker in the ring frame: one target for the approach and the weld check) AND a continuous loop on the measured tip (z only, integrating only in a still hold, tip-validity checks, contact anti-windup; card 2026-10-09_approach_tip_feedback rev 2 with the critic's must-fix items).
DECIDED 2026-10-09  Newcomer defaults (Wesley, afternoon): newcomer_lifecycle, newcomer_fault_land and departed_ground_idle (push to rest, then disarmed alone) become the default everywhere once the LAND-every-phase table (card 2026-10-09_land_every_phase) passes its reviewer.
DECIDED 2026-10-09  Twin links (Wesley, afternoon): the twin keeps QUAD4's tether as a rigid link, like the other links (no slack-tether model); a tip-over on a LAND within ~1 s of a weld (R1315, 1 of 2) is a known twin artefact, not a rig hazard (the rig tether goes slack and the rig lands straight down).
DECIDED 2026-10-09  Magnet feedback on the rig (Wesley, afternoon): approach_tip_feedback ON from the first R8b, on the twin setpoint until Wednesday's props-off calibration updates attach_tip_dz (protections: the contact freeze, the 6 cm down bound).
2026-10-09  note: configs/rig/attach_m1.yaml also turns the tip loop on for R8a (a no-weld preview of R8b's loop, said to Wesley in chat 9 Oct pm; reviewer flagged the difference from the decision line above); Wesley can drop it for R8a.
DONE    2026-10-09  Newcomer defaults applied (the decision above; LAND re-fly R1413-R1446 SUPPORTED with corrections by the reviewer, card 2026-10-09_land_every_phase; disarm-alone re-fly R1411/R1412/R1447-R1451): common.yaml newcomer_lifecycle, newcomer_fault_land, departed_ground_idle true; sim_legacy keeps them off. Resolved launch parameters change only there (sim/real attach: the three; sim/real dissipative: departed_ground_idle; legacy: none). An air start turns departed_ground_idle off with an info line (was an error). Gate: stages 1-5 pass; the SIL smoke fails its two peak bars identically with and without the change (R1452 = R1453; failing since R0840, 3 Oct: the sil_smoke re-baseline is still waiting for Wesley's word).
DECIDED 2026-10-10  Wednesday's tree (Wesley): merge rig-prep into week4 after one twin check run each on rig-prep's tip (acceptance, w4 hover detach and w5 circle detach with the freed drone disarmed alone, R8a0, the sheet's two r8a with the tip loop on, R8b weld with the loop on); the Tuesday desk rehearsal then runs on week4. Wesley pushes.
DECIDED 2026-10-10  Tejen port (Wesley): start now on branch tejen-sync, sim only (marker fix, 3-way merge of tejen_mission to 11a0683f, his C++ planner fixes; every change logged in src/tejen_mission/CHANGES_FROM_TEJEN.md); nothing reaches the rig tree before Wednesday. His repo has nothing new since 11a0683f (fetched 10 Oct).
DECIDED 2026-10-10  SIL smoke gate (Wesley): find what moved the two peak bars at R0840; if it was an intended default change, re-baseline the two bars from the current tree plus the profile's margin.
DECIDED 2026-10-10  Thesis (Wesley): use the proposed research questions (Q1 carry, Q2 steady accuracy, RQ1a leave, RQ1b join, RQ2 planner vs network); references for the three open topics found with Undermind, approved by him; blank space between paragraphs; the 50-page limit is not enforced while drafting (he cuts later).
DONE    2026-10-10  SIL smoke re-baselined (the decision above): both peaks come after LAND on the bench's stands and from decided defaults, the 0.86 kg ring (23 Sep) for the cable accel and land_unwind + x0_relax_frac 0 (4 Oct) for the tracking error (R1461-R1467; R1467 with the four 4-5 Oct LAND defaults reverted is bit-identical to R0843). configs/gate_thresholds.yaml: peak_track_err_m 0.40 -> 0.60, peak_cable_accel 8.0 -> 12.0 (current 0.4705 / 9.664, bit-identical over six runs; in flight 0.120 / 7.12). Correction to the 9 Oct lines: the pair has failed together since R1081 (5 Oct), not R0840. Alternative not taken: score the peaks only up to LAND.
DECIDED 2026-10-10  Merges (Wesley, evening): everything that passed goes into week4. Merged: partner-handoff-fix (R1473; unit test; gate 492 passed). Not merged: tejen-sync (R1474 reweld 5.6 deg against the 5 deg bar, one run), combined-m1 (O1/O6 never critic'd or twin-flown), time-cascade (parked 8 Oct: report the lag, do not fix), safety-defaults (its content reached week4 by other commits). The 12 Undermind papers approved for the thesis. week4 to be pushed to origin main (Wesley pushes).
DECIDED 2026-10-10  Combined M1 hand-back (Wesley, evening): Tejen's controller flies QUAD4 until it has attached to the ring and the attach is confirmed; only then does our controller take over (mux to ours, fold-in). Replaces the ATTACH_READY hand-back followed by our last descent (the hand-back fix of R1473 stays for the legacy flow until the combined stack replaces it).
DECIDED 2026-10-10  Rig magnet (Wesley, evening): in real life the magnet basically has to touch the plate to attach (pull-in gap about zero). A weld check that fires with the face a centimetre or more above the plate counts an attach the rig cannot make.
DECIDED 2026-10-10  Attach roles (Wesley, evening): getting the magnet onto the plate is Tejen's job (his controller flies QUAD4 to contact). The attach is confirmed by OUR check only (our motion capture: the tip on the plate and moving with the ring), and then we take QUAD4 over and fold it in. Moving attach: start as soon as everything before it is sorted. Scope (Wesley, same evening): Tejen's controller does EVERY QUAD4 attach on the rig; our own approach rungs R8a/R8b/R9 leave the rig plan (their twin results stay as thesis evidence); our rig job is the carry, the attach check from motion capture, the take-over and the fold-in, also on a moving ring.
DECIDED 2026-10-10  Landing drop (Wesley, late evening): carriers still 9-16 cm above their start height when LANDED fires and the fleet disarms (7-8 Oct rig landings) are acceptable; falling from that height is fine. No change.
DECIDED 2026-10-10  Combined M1 contact attach (Wesley, late evening): on the rig Tejen's descent aims QUAD4's magnet face 1 cm below the plate (the slack tether takes the overtravel; his ground-ring trial does the same); the twin, with no magnet-plate contact and a rigid rod, aims exactly at the plate and holds there, and our check (contact radius, 0.05 m/s, 0.15 s) welds; tested first as exploratory runs on the old partner world where his mission runs (R1473/R1474), the rig claim from the rig twin with his mission on it (to be built).
DECIDED 2026-10-10  Combined M1 attach goals (Wesley, late evening): (1) give Tejen's controller the best chance to attach: QUAD4's magnet ON from the start of his rejoin (after the ball drop, the only time it must be off on the rig) until the attach; (2) a reliable attach confirmation: contact (tip on plate 9's point, moving with the ring) then a lift proof flown by HIS controller (a small rise until the tether is taut, "don't need to lift too high"; the tip stays on the plate = attached; lifts off = descend and retry); the force QUAD4 feels is logged alongside and joins the decision once its rig noise is known; no wrong-plate detection ("very very unlikely"); (3) then a bumpless hand-over to our controller (our tracker starts from QUAD4's measured position, his last throttle and the measured pull; the planner ramps QUAD4's share). His 0.30 s reference lead in ATTACH_READY (lag compensation, no comment in his code): flown with 0.30 and 0 on the moving ring; ask Tejen what he measured.
DECIDED 2026-10-10  Combined M1 test world (Wesley, late evening, replacing "old partner world first" above): the hand-over is tested on the rig twin only (three_rigid_ground_rig_attach: carriers 3/11/7, QUAD4 free on its mat, rejoin on plate 9), with Tejen's mission in its rejoin-only mode (mission_mode join); the old partner world would only clear the wiring and everything would be re-flown on the twin. His ball pickup and drop come later in the full M1 run on a welded-start world.
DECIDED 2026-10-10  Combined M1 subtest on the rig twin (Wesley, late evening, replacing "rejoin-only mode"): our three carriers take off attached to the ring; Tejen's controller takes QUAD4 off its mat, picks up the object, flies it (his obstacle avoidance + controller) and drops it in, then attempts the attach (contact + lift proof, our check confirms, bumpless hand-over). If no attach happens (his give-up, or our LAND before the attach), QUAD4 goes back to OUR tracker, which runs the tested newcomer abort (magnet OFF, radially out to 1.2 m at its height, land, disarmed alone); newcomer_lifecycle stays on in these runs.
DECIDED 2026-10-10  Combined M1 proof and landing (Wesley, evening, after R1479-R1490): the twin's lift proof stays at the 1 cm lift with the plate-rise threshold 2.5 mm (drift 3 mm, fail 8 mm), frozen before the claim, BUT first check whether QUAD4's cable force estimated from mocap (double differentiation and the applied thrust, as the carriers' measured pull) can confirm the attach; QUAD4 tipping over in the landing unwind (2 of 7, rigid twin rod) is a twin artefact: score landing separately, do not hold the claim on it; after a failed proof his controller re-presses from where the lift ended (as built).
DONE    2026-10-10  QUAD4 force as an attach signal (the decision above): QUAD4's vertical pull from its mocap acceleration minus the applied thrust (the twin's rig law and pack model, 0.2 s average) changes by only 0.00 to -0.06 N during the 1 cm proof lift on a real catch (+0.00 to +0.01 N on the no-catch contacts, R1481-R1490), against the rig's measured-pull noise of 0.11-0.35 N sd (0.2 s average, the 8 Oct carriers in r303_4 / r305; p99 0.3-1.2 N): it cannot confirm the attach within a small lift. It does track QUAD4 taking its share after the switch (0 -> 1.2 N over the fold-in, R1487): usable as a post-switch check, not as the proof. Thresholds frozen as decided: lift 1 cm, d_ok 3 mm, d_fail 8 mm, h_plate 2.5 mm.
DECIDED 2026-10-10  Overnight work (Wesley, evening): finish the claim runs (look at runs as they finish, not all at once) and the reviewer; twin videos of the best runs matching Wednesday's tests (rendered from the logs first, then Gazebo's own rendering on the lab GPU, else on the laptop for the main runs only when nothing else needs it) plus ideas for Tuesday's meeting; a full rundown of the integration and of what changed in Tejen's controller/method; the real-life concerns vs the twin; the rig port of the contact attach with a desk test (one launch file if possible, else one or two new ones, sensible for the Wednesday tests); the full M1 in the twin (welded start, detach, his mission, contact attach). Not now: thesis writing (after next week's rig results), twin fidelity checks. Think toward what will happen in real life rather than in the twin.
DECIDED 2026-10-10  Combined M1 proof lift and pickup (Wesley, 22:20, after R1497/R1498): (1) his LIFT_PROOF rises from the latched contact height, not from his contact target (R1498: a target 1 cm low lifted the tip only back to the plate, 3 caught contacts never proved); (2) his pickup gets a bounded retry (no ball after ~10 s in MAGNET_ATTACH_WAIT: rise ~10 cm, re-centre, descend again; after 2 failed retries he gives up and we take QUAD4 back; R1493/R1497 stalled to LAND). Both in CHANGES_FROM_TEJEN.md, then the whole claim table (13 arms) is re-flown on that one final tree and the reviewer reads that table; tonight's e097a94 rows stay as the record.
DECIDED 2026-10-10  Videos (Wesley, 22:20): the log-rendered videos are not good enough: deleted with their tool (twin_video). The only video wanted is a full Gazebo recording of the full M1 (welded start, detach, his pickup and drop, contact attach, fold-in, LAND), once it flies cleanly; no subtest fallback.
DECIDED 2026-10-11  Combined M1 (Wesley, 11:55): (1) the QUAD4 take-back during a trajectory (the circle) MOVES WITH THE RING (its hold and rise follow the ring, then it steps out), the eventual goal; after R1543/R1549 (rim snag), R1554 (0.24 m over a carrier) and R1565 (world-fixed hold swept by the circling carriers -> fleet stop). (2) contact_still ON by default for the combined M1 in the rig and the twin (c0 from the last 0.05 s of the dwell, vertical speed <= 0.02 m/s); the twin claim is re-flown with it. (3) Wednesday's sheet gets the measurement items (concise; Wesley reviews). (4) The QUAD4 tilt cut's twin arms fly on the lab; defaults decided after. (5) The working rule "a fix not shown in two runs: stop and report" is removed.
DECIDED 2026-10-11  Plan for the remaining combined-M1 work (docs/archive/plan_m1_remaining_2026-10-11.md; Wesley 12:50): (D1) the co-moving circle take-back ONLY, no pause-the-circle fallback: if it has not passed its twin arms (TB6) by the Thu 15 gate, the rig flies HOVER ONLY on 21-22 Oct. (D6) switches split: warm_fault_not_live is a 21-22 Oct candidate after its reviewer (arm C3); tilt_cut and fc_disarm_land stay OFF on 21-22 Oct (props-off checks that morning are evidence only) and go on from 28-29 Oct if they pass and Tejen agrees; the DISARM-after-LANDED sheet line stands until then. (D7) F6b is 1 of 2 by the frozen rev 3 text (R1582: landing tip-over): fly H1, F6a, F6a2 now; keep/delete after them; a landing-artefact rule goes into tilt rev 4 for later runs only. (D19) the take-back hold ramps from the current velocity, no switch; land_transit_2 joins the claim. (D20) partner_contact refuses trajectories other than hover/orbit in the twin and anything but hover with real:=true until the co-move take-back is reviewed. (D4) all bars (card rev 10, tilt rev 4) frozen before any of these runs. (D5, changed) week4 stays as it is; everything that works and is tested next week goes on a new branch week5 (the candidate lineage ends there), commits as Wesley on the work branches and rig-prep docs.
DECIDED 2026-10-11  Thesis (docs/thesis_plan_2026-10-11.md; Wesley 13:40): the spine is adopted: RQ1 carry (what the measured rod pull buys on Betaflight), RQ2 leave (what sets the transient; the ring is lost only when a magnet lets go), RQ3 join (admitting another controller's drone); the twin prediction ledger is section 4.4, not an RQ; the planner-vs-network question moves to an appendix; subtitle "Leaving and Joining a Carried Ring"; prose starts Fri 16 in the plan's order. THE DUE DATE IS 20 NOV 2026 (the title page), not 27 Nov: everything after the 7 Nov results freeze moves one week earlier; sections 4.3/4.4 are drafted during 2-7 Nov. Wednesday 14 Oct additions: the spring-scale magnet hold at 0/30/45/60 deg and phone/tripod video of every flight from now on (D6); W4/W5 hold 15-20 s on three drones before LAND (D5, these two only); the release metrics, the ledger band rule and the W4/W5 bands frozen before Wednesday (D4, W4/W5 bands from the same-tree twin runs + 1 lab run each on Mon 12); no W4ff on Wednesday (D7, 28-29 Oct under its preconditions). Registry: misaligned rows get appended correction rows ('<id>-corr'), existing lines untouched (D15).
DECIDED 2026-10-11  Co-moving take-back, critic round 4 (Wesley 14:30): drop the escape height (c6f2032's rise on the circle too; TB5 bar: after the decouple the ring centre stays >= 0.9 m from QUAD4 at every phase of a 10 deg sweep, else the step continues further out, not higher); adopt the zone rule for orbit take-backs (far/high: today's path + ramp; below the ring: land where it is, no sideways move or rise; else co-move, rising only when the tip is at the ring); on the rig circle his TAKEOFF comes before the orbit starts (sheet order + a rig traj_hold_s long enough; one twin neighbour with his takeoff 12 s into the orbit); add the rig release guard before the rise (hold in the ring frame, re-send OFF, LAND the fleet if plate 9 follows or the tip has not left it within 2 s). Ramp constant 0.25 m/s^2 (Claude's call: 1.46 deg quasi-static under the 2 deg bar).
DECIDED 2026-10-11  Co-moving take-back, the helper's open points (Wesley ~15:15): (1) TB5's R1565 reproduction counts (rods meet at 0.07 m; the body gap 0.34 m against 0.28 m logged is carrier 2 reacting after the hit, which the kinematic replay cannot); (2) rig release guard failure: QUAD4 FOLLOWS THE RING DOWN (tether kept slack, no pull on plate 9 while the fleet lands), then steps out by the slack and lands beside the ring, disarmed alone, instead of holding at height; (3) zone order b (below the ring: land where it stops) before a (far/high: world-fixed path); (4) the constants as built (tip at the ring from 0.15 m below its centre, carrot brake 0.2 m/s^2, gain 1.5/s, ball 0.16 m, guard +0.07 m).
DECIDED 2026-10-11  Rig release guard failure, LAND timing (Wesley ~15:55): the fleet LAND goes once QUAD4's reference is down at its tether's slack (fresh ring pose), at most 2 s after the failure, instead of at the failure (sent at the failure, the ring would start down 0.07-0.12 m under a taut tether for ~2 s). Built on m1-guard (b1cb2de + this change); rig only (real:=true).
DECIDED 2026-10-11  From the thesis health check (another session's plan file wild-scribbling-alpaca.md, advisory; Wesley ~16:05): (1) NO R8b fallback: Tejen's controller stays the only rig attach path (our approach was twin-tested with the 8 cm proximity weld and never presses to contact; the rig magnet catches only on contact); if R10a on Thu 15 is not reliable the first rig attach moves to 28-29 Oct. This replaces the health check's 13:00 'fallback: yes'; CLAUDE.md's lock line stands. (2) The old RQ2 (planner against network) goes to a scope paragraph + Appendix F now, as the adopted spine (13:40), replacing the 13:00 'decide after the first rig attach'. (3) No per-lab-day control detach for now: repeats come once everything works. (4) The due date stays 20 Nov (the health check's 27 Nov is stale).
DECIDED 2026-10-11  TB6 readings (Wesley ~16:20): R1624's falsifier 'a sideways move with the tip within 5 cm of the floor' reads the COMMANDED path, so R1624 supports (the 0.33 m push was the rigid twin rod after his object magnet caught the ball, TB14 class; no code change; his object magnet left ON in a take-back before his pickup goes in the Tejen note). m1-guard (release-guard failure follows the ring down; LAND at the slack, max 2 s) merges into m1-comove now. TB6 COMPLETE (R1615-R1625): no arm meets its falsifier and every circle take-back confirms both clearance bars (rod 0.205-0.253 m, body 0.415-0.493 m); between the bars: R1618 decouple step 2.15 deg, R1621 transit take-back step 3.83 deg (for the Thu 15 gate); rig finding R1625: the ring passes over QUAD4 if his takeoff follows the orbit start (0.23 m xy, 0.12 m vertical).
NOTE    2026-10-11  Clock times on the 11 Oct lines are approximate: some were estimated rather than read from the clock and ran up to ~1.5 h ahead (the four afternoon lines above are corrected from their commit times; the earlier ones may be ~15-20 min ahead). The commit times are exact.
DECIDED 2026-10-11  TB14 measurement (Wesley 16:52, before any rev 10 claim run; card §6.4): tip on the floor = ring z < 0.07 m and tip z < 0.10 m; within 1 s = QUAD4's tilt from under 10 deg to past 60 deg in <= 1 s; body > 2 cm above its first logged z. The round-4 F10 wording (tip z <= 0.03 m, within 1 s of the tip reaching the floor) cannot be met (welded tip on plate 9 reads 0.052-0.060 m; 60 deg comes 1.5-2.6 s after touchdown) and found 0 of 12. The measurable reading finds exactly the hand count 12 of 48. Scorers committed (m1-score 986ac0a: tools/m1_score.py, m1_metrics.py; reproduce R1573, R1579, R1581, R1615, R1620, R1623, R1624 field by field), fast-forwarded into the candidate base m1-comove.
DECIDED 2026-10-11  The pre-Wednesday freeze (Wesley 17:24; docs/rig/freeze_2026-10-14.md): P1 (T6 release metrics, resolution rule, replication/prediction labels, ledger band rule, the 1-DOF model's signs) frozen with every question Q1-Q14 at its recommended option (§3); P2: Q15 the rig releases at the twin's timing (W4 6-7 s after the ring reaches 1.0 m, W5 ~10 s into the circle), so the existing twin configs and R1457-R1460 stay in the band; the two week4 band runs are on the lab now; band numbers are computed with tools/release_bands.py only after them. The 14 Oct sheet's 23 proposed rows are on the sheet in run order (tilt check (c) only with Tejen's OK).
DONE    2026-10-11  W4/W5 bands frozen (17:35, after the sign-off; docs/rig/freeze_2026-10-14.md §2.4): W4 [1.75, 14.51] deg (twin R1459, R1460, R1626; midpoint 6.21), W5 [1.00, 13.20] deg (twin R1457, R1627; midpoint 5.39); f 0.851-1.704 from the three unannounced 8 Oct pairs, sigma_rig 1.568. Wide by construction (rig/twin ratios span 2x): the ledger's three-way scoring (hit, width, log ratio) carries the claim.
DECIDED 2026-10-11  Thesis lane (Wesley ~17:30, via the thesis lane's hand-back docs/thesis_work_2026-10-11.md; work merged 18:18: thesis-rig fast-forwarded to 67cb4df, desk-ext merged into rig-prep): (1) the 7 Oct rig releases r0012 and r0013 are not tabulated in the thesis; r0013 is cited in the text as the magnet release; the 7 Oct tests may be repeated once everything else works. (2) The leaver rule (a four-carrier layout admits at most two leavers, 1/3/5/9 one; any-single-loss tolerance needs five carriers) goes into Section 3.4 with what control still sets after a four-to-three loss; the resize-equals-statics result and the gap scaling into Section 4.2; the r0013 over-step (2.75 N against 1.61 static) into the magnet paragraph as a precedent. The newcomer-beside-a-carrier remark and the mission-value sentence are not used. (3) Zeng 2025, Lv 2026 (T-ITS), Liu 2026 (TNSE), Antal 2026 (RA-L) and Xu 2026 (ACC) approved for the bibliography; Fu 2026 not taken. (4) The planner-against-network question is a scope paragraph plus Appendix F (earlier plant), done in the draft. (5) Q1 and Q2 folded into RQ1 in the draft's headings; the estimator comparisons move to Appendix D with the results rewrite. (6) The leave claim narrowed to "no work reports a number for the payload's attitude through a departure, or flies a join", after Quan et al.'s full text (4 -> 3 under 4 kg, tensions balanced after 13 s, no attitude) and Zeng et al. 2025 (plots, no number).
DECIDED 2026-10-11  Rig-port critic round 2 on the candidate (GO WITH CHANGES for hover R11a/R11b; rig card ## Critic) (Wesley 18:41): (A1) release guard for 21-22 Oct: rise 0.05 above slack (a new height term; the 2 cm plate-rise failure check unchanged) and GUARD_WAIT_S 3 s, with a rig-geometry unit test; the card scores a guard failure with plate 9 rising as a falsifier and a timeout with the plate still as reported, not a stop; (A8) R11a uses contact_dz +0.05 instead of +0.03 if R10a shows his height error over 2 cm. (A2) a sheet line only: to stop his attach publish his give-up (the ring stays at hover), LAND only as an emergency; no LAND-hold code before 21-22 Oct. (A3, A4, A5, A7, A9) hardening before Friday's claim, through tests and a short critic pass: refuse weld_point_z == 0 and an untyped QUAD4 rod length on the combined rig, weld_point_z as a launch argument, contact_rig_orbit honoured only when typed; the guard reads plate 9 from the weld point, passes only on 0.1 s of fresh samples, averages the plate height; his drop check waits for a fresh puck pose (logged in CHANGES_FROM_TEJEN.md). (A6/F6) Wesley accepts in writing the sheet line 'after LANDED, if drone 4's mux still reads partner, clear the cage and press DISARM'; warm_fault_not_live stays off until after 21-22 Oct.
DECIDED 2026-10-11  His drop check's puck wait is a switch (Wesley 19:12): drop_puck_wait_s, rig default 2.0 s (an unseen puck after the release dwell is a give-up), 0 = trust the release (his earlier behaviour). Wednesday 09:00 item 11 (puck in the net, ring on a stand at ~1 m: does body 6 stream?) decides the 21-22 Oct value; with 0, Wesley checks by eye that the puck dropped. Hardening committed on m1-harden (b8af32e, ac85597); it merges into m1-comove after a short critic pass.
DECIDED 2026-10-11  Release-guard follow-ups (Wesley 19:41, after the critic's follow-up on the hardening): GUARD_SLACK_M 0.12 m (at 0.05 the reference ran 2.7 cm over taut through the LAND step; the twin's QUAD4 trails a moving reference by 1.5-3 cm p95, TB6 R1615/R1616/R1620/R1623); the reference speed capped at 0.06 m/s during the guard stage only (a held plate's peak lift 5.9-6.0 -> 3.3-3.7 cm; a real release passes 0.3-0.4 s later, >= 1.3 s inside the 3 s wait). Also committed on m1-harden (fff090a): the combined rig refuses a typed rod length or magnet_drop_below_quad more than 1 cm off the measured taut length; a guard timeout names its causes; tip0 is taken when the guard starts.
DECIDED 2026-10-11  Combined M1 attach flights move to WEDNESDAY 14 Oct afternoon (Wesley 20:56, against the recommended staged plan): R11a (the whole combined mission on hover, no-catch offset) then R11b (contact attach on hover), after the 09:00 measurements (QUAD4 record: weld_point_z, taut length, contact radius, puck in the net) and the props-off F1 load test pass the same day; no R10a before them. The REVIEWER IS WAIVED for these flights (noted in their registry rows). QUAD4 is built the same as QUAD2 and the other quads (Wesley), so his file's values apply: mpc_thrust_ratio 22, magnet_drop_below_quad 0.575, marker-to-face -0.0225 (body/marker ids are QUAD4's, 14/24); the 09:00 taut length must agree with 0.575 within 1 cm (the launch cross-check) or the measured value is typed everywhere. R11a's no-catch offset is contact_dz +0.05 (no height-error data without R10a). Wednesday's combined flights and Part B run on the CANDIDATE install (m1-comove, pinned), rehearsed at the desk Tuesday; the 21-22 Oct plan becomes repeats and the circle only under its own gates.
NOTE    2026-10-11  Wesley (20:57): Tejen has done multiple real tests of his controller: one drone takes off, picks up the object, puts it in the basket and attaches to the payload. Asked for: his bags/logs of a few runs (height error at contact, kT in flight), the code version and drone, and whether the payload was on the ground or carried. It lowers the risk on his contact (the old R10a question); our fork's contact hold + lift proof + our confirmation + mux hand-over has not flown on hardware.
DECIDED 2026-10-11  Wednesday's order (Wesley 21:10): Tuesday's desk rehearsal and desk load test are skipped (Wesley runs the stack on this laptop); the props-off load test on the rig laptop (both stacks, real radios, 10 min, the F1 bars) stays as the go/no-go before R11a and also checks the Part C launch lines; the QUAD4 record (taut length, contact height x3, marker to face, d_ok) is measured, not taken from 0.575 (the rig proof's taut check works at the mm level); r8a0 is dropped (R11a covers it); the magnet hold and the tilt checks run only if time remains. Order: 09:00 items, hand ramps, W4, W5, load test, R11a, R11b.
DECIDED 2026-10-11  Wednesday install fixes (Wesley 22:05, launch-only, no flight-loop code): the fleet manager announces on the latched /fleet/manager_status at startup (rig_flight killed every combined launch at 120 s, and fleet_reach and preflight refused on a fresh launch); the workspace root lookup accepts a git worktree (every tracker died at startup from mdc_wt/rig-wed). The pinned Wednesday install is rig-wed ba40bd0. DONE: the desk load test (tools/load_check.py) passes every F1 bar at normal priority over 10 min (worst command gap 0.095 s against 0.15; 0.077 s on /join_planner/reference against 0.30; zero fault lines) and fails at nice 19 (0.36 s gaps): nothing else may run on the laptop during combined flights; one unexplained all-process stall (~0.35 s, 40 s after launch, with a disk-write burst) disarmed QUAD4 on its mat through his 0.25 s pose watchdog, so his ARM waits 2 min after the launch.
CLOSED  2026-10-12  The combined claim on rig-wed ba40bd0 (R1638-R1654) is NOT SUPPORTED overall (reviewer, card §6.3a). Hover lines supported; circle take-back and push abort (R1654 rod 0.094 m < 0.10) not. Wesley to choose: claim the hover lines now and re-fly the circle and abort lines as a new table on week5 (recommended), or re-fly the whole table. Wave 1 (R1638-R1641) is a confound: 2-slot re-flies R1679/R1680 pass his gate with ref-age p99 0.10-0.11 s (wave 1: 0.12-0.13); cause not established.
CLOSED  2026-10-12  Take-back race after a tilt cut (tilt reviewer, R1662/R1673/R1678 on ba40bd0): the fleet's FAULT_LAND reaches the planner before /drone_3/tilt_fault, so a TAKE_BACK is published for a cut QUAD4 (inert: the mux stays cut). Proposed for week5, with a critic: the planner handles a tilt fault before a LAND take-back and skips a drone whose mux reads cut. The tilt cut stays off until this is fixed. Tilt table: T1 and F1 1 of 2, N3 between the bars; R1673 a real tumble after his takeoff (1 of 43; R1681 no repeat).
DECIDED 2026-10-12  The combined claim's way forward (Wesley ~10:10, closes the first OPEN line of 12 Oct): the hover lines are claimed now (hover attach, hover take-back, no-catch hover, LAND during the proof; rig-wed ba40bd0, card §6.3a). The failed lines (circle take-back, circle no-catch, push abort) go into a new table on week5 with the same bars, and the batch starts with hover arms. Tonight, exploratory on rig-wed, in this order: (1) re-fly the failed lines x2 each (m1c_nocatch_orbit, m1c_lead030, the circle LAND take-back of R1639, m1c_abort); (2) the full M1 on the circle x4 (is R1656's miss a one-off? 6 of 7 overall, 0 of 1 on rig-wed); (3) a push-abort clearance probe with its neighbour (the other push direction); (4) his circle-gate probe (his MATCH_VELOCITY errors against 2.5 cm and 0.08 m/s, from logging that already exists).
DECIDED 2026-10-12  The take-back race after a tilt cut (Wesley ~10:10, closes the second OPEN line of 12 Oct): fix it in the planner. It never takes back a drone whose mux reads cut, and it handles a tilt fault before a LAND take-back. Critic WAIVED by Wesley ("a small change ... just make the fix"); the tilt arms that show the race (T1, N3) are re-flown on the fix's tree. The cut stays off on the rig until 28-29 Oct as before.
DECIDED 2026-10-12  Thesis lane 3 (Wesley ~10:10): the 17 bracketed Chapter 2 citations are approved in one batch (Six 2018; Fink 2011 and Tognon 2018; Farina 2020; Loianno 2018; Gawel 2017; Graebe and Ahlen 1996; Peng 1996; Sha 2001; Gros 2020; Hendrickx and Martin 2017; Sekercioglu 2026; Tagliabue 2019; de Marina and Smeur 2019; Bulka 2022; RotorTM; RotorS; Cruz and Fierro 2017). The magnet hold stays at about 9-10 N straight off the face (measured 7 Oct) until the 14 Oct spring scale replaces it.
NOTE    2026-10-12  Wesley: do not raise R1673's tumble (his controller, after his takeoff, 1 of 43; R1681 no repeat) with Tejen yet; only if it shows up again in the twin.
DECIDED 2026-10-12  week5 created (Wesley ~10:50: "ideally I want to use this for testing"): week5 = rig-wed ba40bd0 + m1-race (29b2d7b the take-back race fix, 46ac389 the m1c_abort_negx config), worktree mdc_wt/week5, built and prebuilt; gate --quick passes (585 tests). Wednesday 14 Oct flies week5 for everything (W4/W5 and the combined flights; rig-wed stays as the claim batch's record). m1-rec and m1-video are superseded (the recorder came in as adaf2c5). The W4/W5 bands (R1626/R1627 on week4) get one same-tree check each on week5 tonight.
DECIDED 2026-10-12  Rig plan front-loaded (Wesley ~10:40): every rig item is tried as early as possible so failures can be fixed, and the last weeks are for repeats and data collection; the twin likewise runs everything as soon as it is ready. Wednesday's sheet is drafted ambitiously with simple labels (props-off P1.., flights R1..); no sheets are drafted ahead for later days. Thursday 15 stays a rig session for retries and fixes. The full M1 on the rig is Wednesday's last flight, gated (R11b clean twice, and the 3-min hover shows the packs last). The lab PC is used only in the evening, when it is free (from 19:00 today).
DECIDED 2026-10-12  Tejen note: add the thesis credit wording for his OK (plan D11); no request for extra rig time.
DECIDED 2026-10-12  Repo tidy (Wesley ~11:30): back up everything first, then a clean, current working tree. The backup: every local branch and tag pushed to GitHub (the public repo is fine: Tejen said he is OK with his code public, and so is Wesley) under backup-2026-10-12/<branch>, plus a local bundle ~/backups/multi_drone_control_2026-10-12.bundle (verified, all 32 refs). The push is Wesley's (the permission check refuses it for the assistant). Never-committed work committed first (rig-prep 885ffbc, m1-rig df38269); paper PDFs, Tejen's twin logs and flight data stay local. Then: .gitignore stops ignoring docs/configs/tools; rig-prep merges into week5 and the main tree moves onto week5 (rig-prep retired); merged worktrees and branches removed; old docs to docs/archive; uncited configs and unused tools removed; the lab copy's stale folders cleaned at 19:00.
NOTE    2026-10-12  Wesley: Tejen is OK with his code (our fork) being public. The Tejen note still asks his OK on the changes themselves.
OPEN    2026-10-12  From the P6 offline analyses (docs/analysis/p6_offline_2026-10-12.md, merged 6985d74), for Wesley:
  (1) Rig delta-theta null: with nothing happening the rig reads 4.8 deg (hover) to 6.3 deg (path), about 3 sigma_pre. r303_4, r305 and r0012 can't be told from it; the frozen W4/W5 bands (R4/R5 Wed) reach below it. Bars stay frozen; for later runs: report every rig delta-theta against the null.
  (2) The frozen T6 definitions need amending for later runs: time back in band needs LAND >= 10 s after the event (scorable for 1 of 8 rig rows; Wednesday's 15-20 s hold makes R4/R5 scorable); an uncommanded release has no event time (use the slip onset, labelled); sigma_rig 1.57 is not the null's spread (0.8-1.5); the resolution rule's range bar makes a pair harder to resolve at n = 3 than at n = 2.
  (3) The double-count hypothesis (C2 "when the resize comes") is not in the logs: by the resize the measured pull already holds the step (unannounced, rig median 93 %) and the thrust never takes it twice. New, rig only: the survivors' thrust swings AGAINST the step when the plan arrives (9 of 9, 0.14-1.40 N, ~1 Hz). Reword the claim; the counter-swing is worth a look.
  (4) Latency: command to body rate 20-25 ms, gain 1.0, no lag after it; the tracker assumes no delay and a 0.12 s rate lag. Explains only 20-25 ms of the 90-160 ms rig-twin path-lag gap. A tracker model change would need a card and a critic.
  (5) Stretch detector: 0 false alarms at 2-12 cm over 660 s of rig flight; 8 cm costs 0.13-0.14 s against 6 cm. A threshold change is Wesley's.
  (6) MDE at n = 3: ~4.5 deg in hover, 7-9 deg on a path; the ramp's ~0.8 deg resolves in 1-2 % of pairs: the rig ramp pair (28-29 Oct) can't show it.
  (7) T8 corrections: four uncommanded releases, not three (r0013 lost a second survivor at +0.397 s, ~46 deg); delta-theta of the five carried-on releases 5.3-16.9 deg (the plan's 4.6-17.8 are absolute peaks); r200006 detection 0.28 s at the planner tick (0.38 s was the log stamp).
DECIDED 2026-10-12  Lane 6 answers (Wesley ~16:15): the counter-swing filter NOT NOW (the finding stays in docs/analysis/counter_swing_2026-10-12.md; the tracker unchanged). Tonight's lab adds 8 runs after the 24: Tier A pair T3 (arms A1 measured / A2 planned, unannounced hover release, n = 3, configs ta_a1_r1-3 / ta_a2_r1-3 from w4_da_release_hover, release at 20.0/20.5/21.0 s) and the L5 twin pair re-flown on one tree (w4_3915_release x2). Tier A's announced half (A3/A4: the measured soft release does not unload the leaver) is decided after tonight's results. The T6 amendments A1-A4 need a plainer explanation: brought back after Wednesday with Wednesday's numbers; Wednesday is scored on the frozen rules. Routine calls taken as lane 6 recommended: T7 one definition; T8 keeps the uncommanded releases labelled; wed-pipeline, counter-swing and tier-a-card merged into week5.
NOTE    2026-10-12  Wesley (~20:30): the twin tumble under Tejen's controller showed up again (R1683, at the pickup lift, after R1673 after his takeoff): the question goes into the Tejen note (item 6). Supersedes the 12 Oct "not yet" note.
DECIDED 2026-10-12  Generated output has one home (Wesley ~20:00): generated/ at the repo root (MDC_ACADOS_ROOT on the lab's slots) holds every acados solver's c_generated_code_* and *_ocp.json / *_sim.json, and the offline harness's PNGs (generated/dissipative_verify/); every path goes through utility_objects.run_context.acados_root(). The top level is the tracked folders, generated/, build/, install/, the log folders and shortcuts.md.
CLOSED  2026-10-12  The full M1 on the circle in the twin: 1 of 4 reattached on the Wednesday tree (R1700; R1656, R1690, R1691 missed) against 6 of 6 on the earlier trees (R1523, R1541, R1564, R1579, R1631, R1633). No flight code differs between adaf2c5 and ba40bd0 (only the fleet manager's startup announce and a path helper); his kT and loop timing match the successes. Every miss is his controller on the moving plate: MATCH_VELOCITY in its gate 66-82 % of the time (tip-to-plate median 1.5-1.7 cm against the 2.5 cm gate; the successes read 1.1-1.3 cm), descents aborted on his 'loss of relative alignment', give-up 30 s after the first descent. The circle attach from a mat start works on the same tree (R1646, R1647) and the hover full M1 passes (R1655, R1685). For Wesley: the thesis's full-M1 circle count is 6 of 10, tree-dependent; the rig stays hover-only for his attach; whether to spend lab time on the circle full M1 (e.g. his ATTACH_READY lead or gate) or report it as marginal.  [CLOSED 16 Oct, Wesley: T12 = two hover runs under a card; the circle full M1 reported as exploratory and marginal (R1700, R1656, R1690, R1691 against R1523, R1541, R1564, R1579, R1631, R1633); no lab time on his circle match.]
RESULT  2026-10-12  Tier A pair T3 (unannounced hover release, measured against planned pull; card 2026-10-12_tier_a_crossover rev 2; week5 fc9fb04): A1 measured 5.37 / 6.15 / 6.12 deg, A2 planned 17.13 / 17.21 / 17.02 deg (R1706-R1711). RESOLVED with measured < planned under the frozen rule (sign, no overlap, 11.2 deg >= 1.6). The frozen prediction for T3 is confirmed; the reviewer runs on it tonight. T2/T4 (A3/A4) wait for Wesley's word.
NOTE    2026-10-12  Tier A T3 reviewer (12 Oct night): SUPPORTED with corrections, applied to the card: the tree is week5 70d525f (the manifests; fc9fb04 has the same code), the section 5 side columns added (detection 0.33 vs 0.28-0.30 s, the planned arm detected sooner; dip 2.9-3.1 vs 4.7-5.0 cm; lean 1.2-1.3 vs 1.8-1.9 deg), and the cell's limits stated: the planned-pull arm hovers in a steeper formation (rods 67.0 vs 60.6 deg) with a 2 deg steady ring tilt, so the pair settles the sign of T3, not its mechanism. Registry: R1706-R1711-corr for the tree label.
DECIDED 2026-10-14  Rig QUAD4 attach values from the props-off tests (Wesley, 08:55): W = weld_point_z 0.024 (test 2: 0.0239 / 0.0244 / 0.0238; was the 0.031 placeholder); L stays 0.575 (the lifted read moved, sd 27.7 mm, peak 0.572; Wesley accepted it within 1 cm without a still re-read). Plate 9's point moves to where the 8 level catches centre, attach_x_offset 0.013, attach_y_offset 0.210 (nominal 0, 0.225: the caught tip sat 11-37 mm from it, 2-19 mm from the centre), with K = weld_radius 0.024 by the sheet's rule (furthest catch 19.2 mm + 5). Typed on the combined launch only (the sheet's X, Y, K and M); it moves our contact check and the release guard's weld point, not Tejen's aim, which stays (0, 0.225) from his geometry, the outer -x corner of the catch area. A ring tilt of 14-20 deg raised the caught tip 7-9 mm in the ring frame. Record: results/rig/2026-10-14/tip_calibration.txt, t3_catches.txt.
DECIDED 2026-10-14  The props-off load test (test 4, card 2026-10-21_combined_m1_rig F1 ">= 10 min") is scored on a 3 min window from his TAKEOFF (Wesley, 09:21, against the recommended 5 min = one full mission). Same bars. The flights' own bags are scored on the 5.1 bars as the card already says ("any flight whose laptop load is new"), so test 5 is the second load sample. The registry row says F1 was scored on 3 min, not 10.
DECIDED 2026-10-14  F1 override (Wesley, after RIG-1014-lt_6): the props-off load test fails F1 on one line (/drone_3/ELRSCommand_diss 0.160 s against 0.15: our drone-3 tracker's warm start 0.8 s after his TAKEOFF, not forwarded to QUAD4's radio before a take-back); every radio-bound stream passed by >= 35 ms with no fault, pose-timeout or his-disarm line. The card's "combined flights stop for the day" is overridden: test 5 flies, with fleet_viz at nice 19 and the routine T3/T4 commands under nice -n 19 (stop commands and RViz at normal priority). Test 5's bag is scored on the same 5.1 bars, our tracker at the take-back included. The lt_5 failure (two controllers starved 0.32 s at his TAKEOFF, load 16 on 12 cores) is the reason for the priorities.
DECIDED 2026-10-14  drop_on_loss is on in every mode, attach included (Wesley ~10:45, CRITIC WAIVED, after RIG-1014-t5_2 where attach mode had it off and a released carrier ended in a fleet disarm in the air). graph_attach.py no longer forces it off (departed_land stays off: a drone detached in M1 is the partner's); the carriers' trackers get the tilt rule (tracker_drop_options); a drop also sends /magnet/command OFF (at most 1 Hz) so a welded QUAD4's coil releases (the coil gate ignores it before our span). Open: a drop while Tejen flies QUAD4 does not hand QUAD4 to our take-back (his mission carries on): to build and twin-test before tests 6-7. launch tests 97 pass, gate --quick passed 11:09; twin check pending.
DECIDED 2026-10-14  Rig load cuts (Wesley ~10:45; docs/analysis/rig_load_2026-10-14.md): fleet_viz viz_hz 10 and path_dt 0.5 on the rig path; trackers' RViz plan and path every 5th tick (VIZ_EVERY, tracker_node.py); attach_target_publisher off on the combined rig path (take-back falls back to the weld point); radio telemetry printed at most 1 Hz per frame type (mode changes at once); his rig nodes: visual-only A* off, planner CSV every 6th update, ring bridge trail 0 (CHANGES_FROM_TEJEN.md). No second machine for his stack (Wesley). Desk measurement 11:12 (fake mocap, no radios, armed 60 s): 795 % total, so his A* and the trail were not the cost; the py-spy profile shows rclpy's executor (wait sets, QoS event handlers) as the largest share in most nodes.
DECIDED 2026-10-14  More rig load measures (Wesley ~11:45): the lean rclpy switch on for the rig (rig_flight.sh exports MDC_LEAN_RCLPY=1 with tools/lean_rclpy on PYTHONPATH; desk graph 795 -> 710 % CPU); the bag drops /tejen/pendulum_swing_state and the ring bridge's /fake_payload/pose and /fake_attachment_point/pose (MCAP not used: -10 % only, five replay tools read .db3); the mocap node's map->drone_<id> TF off on the rig (publish_drone_tf, real_io_launch; RViz uses fleet_viz's drone_<i>_mocap); tools/rig_pin.sh --apply --boost after each launch (display and recorder on the E-cores, the control path at nice -10). Mocap capping NOT done (the mocap velocity is averaged over a fixed frame count: 60 Hz would lag it; twin first). Tests 120 pass; not yet flown.
DECIDED 2026-10-14  His rig pickup tuning (Tejen's suggestion, Wesley ~14:42, after RIG-1014-t5_4): rig overrides in partner_mission.launch.py, pickup_alignment_low_xy_m 0.025 -> 0.045, pickup_alignment_high_xy_m 0.05 -> 0.06, pickup_alignment_low_dwell_s 0.3 -> 1.0; the capture gate (0.025) and the retry settings unchanged ("retry later": not decided, ask Tejen which knob). CHANGES_FROM_TEJEN.md updated.
DECIDED 2026-10-14  His rig pickup tubes widened again after RIG-1014-t5_5 (Wesley 14:54, against the recommendation to keep 0.045 and work on the catch at the bottom): pickup_alignment_low_xy_m 0.10, pickup_alignment_high_xy_m 0.12. Evidence: with 0.045 the descent reached the bottom twice, the magnet sat 2.5-4.5 cm off the puck and the puck never rose; Tejen to look at the final alignment before the attach.
DECIDED 2026-10-14  Card docs/experiments/2026-10-14_tracker_timing.md (Wesley ~15:05: "make those changes now, with critic"): A, the next-tick throttle (`cmd_next_tick`, env TRACKER_CMD_NEXT_TICK; throttle only, once airborne; QUAD4 pinned off) and B, the carriers' mocap velocity filter (MDC_MOCAP_VEL_FILTER / MDC_MOCAP_VEL_DRONES on the rig, SIM_MOCAP_VEL_FILTER in the twin; `slope` = 4-frame least squares), both switches OFF by default; rig_flight.sh unsets them unless RIG_TIMING_SWITCHES=1. Critic ~15:20 (rev 1: do not run as written); rev 2 answers all 17 points (the delay sweep as the trigger, motion bars frozen, B replayed offline on the rig bags: slope 11-23 ms lag vs ema 44-46 ms, lower deviation). Twin tonight: stage 1 the delay sweep, stage 2 arms t0a/t1/t2/t3 at +40 ms (~/handoff_2026-10-14/lab_tonight.sh, ALLOW_DIRTY_SRC=1). Rig only after the twin and with Wesley's word per switch. Gate now lists test_velocity_filter, test_next_tick_command, test_real_mode_launches (701 pass).
DECIDED 2026-10-14  His mission without the pickup (Wesley ~15:40, after t5_4/t5_5's failed pickups): two new mission modes in his planner, both from the floor with his normal WAIT_FOR_TAKEOFF -> TAKEOFF: floor_join (B, tried first: straight to the ring rejoin and contact attach, no pickup, no drop) and delivery_rejoin (A: takes off already holding the ball, magnet ON from boot, then the transit, the drop into the net and the rejoin). partner_mission.launch.py `mission_mode` argument (pickup_delivery default / floor_join / delivery_rejoin; set through partner_args); under delivery_rejoin the twin keeps the ball's spawn weld (no start release). A twin world variant for A (Wesley's word): three_rigid_ground_rig_attach_held_pickup.sdf, generated by tools/make_attach_worlds.py (the ball on QUAD4's tip; the other worlds unchanged). Twin configs m1c_floorjoin_hover / m1c_delivery_hover; each mode flies the twin before the rig. The existing 'join' mode (no takeoff, unsafe from the floor) is unchanged and not used.
DECIDED 2026-10-14  Tejen's upstream controller (Wesley ~16:05, after reading his repo at d00934c1): port his current tejen_mpc controller (12-14 Oct weights, the d2eaf41e partial-step command execution on all four channels, attitude derivative bounds 3/s, Z 80/120) behind a profile switch, default off (today's controller); twin pickup runs first, the rig only with Tejen's OK. His fast LOW stroke (93d70bde) and its overtravel 36 mm / support allowance 24 mm are NOT ported: he put the speed and entry back to ours (40 mm/s, 80 mm), and t5_5 missed in XY, not depth.
DECIDED 2026-10-14  His rig pickup after the props-off check (Wesley ~16:25, RIG-1014-puck): rig overrides in partner_mission.launch.py, pickup_lift_xy_dwell_s 0.3 -> 1.5, pickup_lift_xy_tolerance 0.08 -> 0.04, pickup_overtravel / max 0.028 -> 0.036, support allowance 0.016 -> 0.024 (his 13 Oct values; this replaces the 16:05 "skip the overtravel"). Flown in RIG-1014-t6: the lift gate refused a lift 6.4 cm off the puck as intended.
DECIDED 2026-10-14  His rig match gate for the ring attach (Wesley ~16:50, after RIG-1014-t6f: the tip swung a median 3.8 cm / 0.105 m/s over plate 9 and the 2.5 cm / 0.08 m/s gate never held 1 s): rig overrides contact_match_xy_threshold 0.025 -> 0.05, contact_match_xy_velocity_threshold 0.08 -> 0.15, contact_abort_xy_threshold 0.05 -> 0.08, contact_abort_xy_velocity_threshold 0.15 -> 0.25; our weld radius K unchanged (a contact still has to land within 2.4 cm). Offline replay of t6f: the new gate holds 1.2 s within the 8.6 s flown.
DECIDED 2026-10-14  floor_join on the rig (Wesley ~16:40, twin skipped for its first flight; twin smoke R1717 later with oct14): flown RIG-1014-t6f..t6h5_3. His TAKEOFF is sent with 'ros2 topic pub -w <subscription count>' (the take-off race, t6/t6f). His controller's mpc_profile oct14 flown once (t6g_2, Wesley ~17:05): no swing reduction, back to oct10 for t6h. Target shift contact_dx +0.044 / contact_dy -0.005 (new launch arg contact_dy, default 0; Wesley flew it in t6h_4..t6h5_3).
CLOSED  2026-10-14  proof_taut_len 0.572 -> 0.562 (proposed after RIG-1014-t6h_4: the magnet held plate 9 but the proof failed on the taut check only; the tether reads 0.570 taut in flight). Flown once in t6h5_3 (no contact, so not exercised). The sheet's P = L - 0.003 needs changing if it is kept; Wesley to confirm.  [14 Oct evening: superseded by the co-motion confirmation (plan WP3, eacab58): the taut check uses this flight's own free-hang length minus 3 mm, measured while QUAD4 hovers over the plate, so no typed length; the replay proves t6h_4's catch (tools/proof_replay.py); twin R1718]
OPEN    2026-10-14  A rig ARM gate on missing pack-voltage telemetry (RIG-1014-t6h: no battery telemetry from QUAD1-3, the takeoff hopped); offered to Wesley, not built. Meanwhile preflight --real before every ARM.  [14 Oct evening: the console (tools/console.py, plan WP2) is NOT READY and greys out ARM without a fresh pack voltage >= 24.0 V from every radio; the manager itself still arms without it]
DECIDED 2026-10-14  The plan after the rig day (Wesley ~20:30): docs/plan_after_rig_2026-10-14.md, its status table kept current. His answers: the arm refusals are the disarming issue (FC settings by someone else: docs/rig/fc_request.md); T2 cleanup + a console, both the console and the RViz panel keep the buttons and SPACE stays an instant, unguarded DISARM in both; every wind fix allowed (QUAD4 changes by Wesley; a bigger plate only as a last resort); the planner folded into mpc_planner, mode dissipative -> detach, the old network controller to the `legacy` branch (made at a0d3963); the attach confirmed Tejen's way; Thursday 15:00-18:00 tests whatever is done.
DECIDED 2026-10-14  Attach confirmation by co-motion (plan WP3, eacab58; proof_rules comotion is the default, contact_lift stays selectable): contact = the tip in the plate frame within the catch radius, normal -15..+12 mm, <= 0.03 m/s for 0.30 s; proven 1.0 s after his lift-done signal with drift <= 6 mm, the tether taut against this flight's free hang, and the plate moving with it; failed on separation or loss (his 45 mm / 0.12 s). His planner holds at the bottom with the magnet ON (contact_bottom_hold, rig on) and retries on his quiet-miss rule. Replay: t6h_4's catch proven, no false proofs in the other 14 Oct bottoms; twin R1718 PASS (proven ~1.9 s sim after contact). Card + critic before the rig flies it as a claim. Which plate point QUAD4 aims at is attach geometry: Wesley's call (OPEN below).
DECIDED 2026-10-14  QUAD4's aim on plate 9 for the next flights (attach geometry, Wesley's call). The plate point is the sheet's X/Y (test 3's catch centre 0.013, 0.210, as on 14 Oct; his bridge now aims there too, eacab58). The tip's own offset (it hangs 3-5 cm -x of QUAD4) was the hand shift contact_dx/dy +0.044/-0.005 on 14 Oct (t6h_4, the one contact); tonight's rig default is his measured-tip trim with no shift instead, but replayed on the 14 Oct MATCH windows the trim learns <= 1 cm in 7 s (the tip moves 0.16-0.20 m/s, his learning gate is 0.08), so it would fly like t6f/t6f2_4/t6g_2 (tip 4-5 cm off, no contact). Options: (A, recommended first) t6h_4's aim with the trim off: target_alignment_mode:=legacy contact_dx:=0.031 contact_dy:=0.010 (the same body target as t6h_4, now from the new plate point); (B) the trim with trim_speed_gate:=0.20 (replay: learns +4.3..+5.5 cm in 4 s; never flown); (C) tonight's default (trim at 0.08): not recommended.  [15 Oct ~15:20 Wesley: B, Tejen's method (trim_speed_gate:=0.20), straight to the attach flights from a1; the bench (b2/b2d) and the dry hold (d1) skipped, his call]
WITHDRAWN 2026-10-14  Plates numbered counter-clockwise again (plan WP6): plate k at +30k deg from the ring's +x (plate 1 = 30 deg), as the azimuths, Tejen and the thesis. f3b3072's clockwise numbering (the 8-14 Oct sheets, the RIG-1008/1014 rows) maps new = (12 - old) mod 12: carriers 3/11/7 -> 9/1/5, QUAD4 9 -> 3, his plate 3 is now ours too; azimuths unchanged. docs/rig/plate_numbering.md. Old rows, cards and configs keep their own numbering.  [later that evening, Wesley: wrong, he wanted the angles changed, not the numbering; the renumbering is reverted, see the next line]
DECIDED 2026-10-14  Arming and landing fixes (plan WP8, 3297746 + dade70d): the fleet ARM is refused at once while QUAD4's FC is not armed (his ARM first); only the refused drones are named; a DISARM stops an ARM sequence; a DISARM burst is announced once; QUAD4 counts as down 0.10 m above its rest when stalled, so it disarms alone after a take-back; a refused ARM before takeoff is cleared with LAND (not DISARM), no relaunch.
DECIDED 2026-10-14  Plate numbers and angles (Wesley, 14 Oct ~22:00, correcting WP6): the plates stay numbered clockwise from the ring's +x seen from above (carriers on 3/11/7, QUAD4 on 9 = Tejen's 3). Every angle that is typed (launch arguments, configs) or shown (log lines, the console, RViz, the sheets) becomes clockwise degrees too, so the angle is 30 x the plate number (carriers 90/330/210, QUAD4 270). The maths inside (ROS frames, mocap, the optimiser, Tejen's code) stays counter-clockwise, converted at one boundary. Thursday flies the clockwise angles if they are done and twin-checked, else the 14 Oct conventions.  [~22:40 Wesley: the same argument names with the meaning flipped (over new names); done in 305e50c, boundaries bringup.profiles.apply and partner_mission _his_ring; twin R1720 PASS]
DECIDED 2026-10-16  Morning report answers (Wesley ~07:40-08:15, docs/reports/morning_report_2026-10-16.md): (1) overnight-1015 merged into week5 (fast-forward to 4b3560e). (2) A ring drop during his flight: drop_takes_back ON at the rig through params_file:=configs/rig/drop_takes_back.yaml (code default stays off), AND a "his LAND" console button that asks his planner to land QUAD4 (/join_planner/land_now; in our launch a straight-down descent where QUAD4 is, magnet OFF: his ground-ring exit back to his mat is not switched on, so over a fallen ring it lands on it; Wesley ~08:17 kept the button with that label), flown once after a drop in the twin first. Closes the locked rules' "Open" line. (3) Zero plate velocity (attach_twist:=zero): hardware first (90 mm discs, 1 m tether), then an alternating A/B if the hardware go/no-go holds and Tejen agrees. (4) Console --slot-gate ON at the rig (typed on the sheet; code default off). (5) results/ backup: Wesley, this evening. Work today on branch day-1016 (from week5), Wesley merges.
DECIDED 2026-10-16  Thesis inventory decisions (Wesley ~08:05; docs/analysis/sim_results_inventory_2026-10-16.md): (1) carry and detach tables freeze on week5 now (4b3560e); the join tables fly on a second tree after the 21-22 Oct rig day. (2) T11 and F14 re-flown on the co-motion proof (join tree). (3) The 4-5 Oct appendix tables and figures re-flown on the freeze tree and regenerated. (4) T8b's L5 twin column switches to R1712/R1713 (against the recommendation to keep R1140/R1145: those flew int_mode reference as the rig flight; R1712/R1713 flew model; numbers agree, 6.0-6.4 vs 6.04-6.34 deg; the caption says so). (5) Twin baseline cable 0.55 m, 1 m as a design row. (6) Twin bands on the 1 m worlds frozen for every cell the next sheet lists, before it flies (N2). (7) Carry card bars frozen as drafted (docs/experiments/2026-10-16_carry_speed_envelope.md); the claim flies twice per cell on the freeze tree. (8) Tier B at 120/150/180 plus 135/165 deg. (9) T12: two hover runs under a card; the circle full M1 reported as exploratory/marginal. (10) Tier A: the no-new-code cells now; the announced pair waits for his leaver rule. (11) The push-abort clearance bar becomes QUAD4's own clearance during the abort, for the new runs only. (12) Backup: Wesley tonight. Lab PC: usable now, but someone may need it soon (check before every batch, yield).
DECIDED 2026-10-16  Corner cutting, how fixes are judged (Wesley ~17:20, plan of 16 Oct): a fix is judged clock-free, on the ring's signed distance to the path (the cut at the tight bends, the across-path mean and rms), with the lag in seconds reported separately (the 8 Oct rule: path error and lag apart); a one-clock total is secondary. The frozen payload_rmse_m / payload_rmse_sweep_m and every frozen carry bar stay as they are. Reason: the frozen RMSE scores against the desired published once per 10 Hz tick and held (a staircase 0.04-0.06 s behind the schedule), so an on-time ring scores ~17 mm at 0.3 m/s and a ring ~0.05 s late scores best; on the clock-free ruler today's timing combination removed the cut (figure-8 bends 19-30 -> 3-6 mm, R1986/R1987) and turned the lag into a lead (~0.04 s ahead on the circle and orbit, R1988/R1990).
DECIDED 2026-10-16  Corner cutting, the timing fix is back in play in the twin (Wesley ~17:20): exploratory, behind off-by-default switches (ref_time_shift, plan_ref_lag_s, plan_sqp_iters; a predictor of the plan's start state if the lead sweep says the leftover is pure delay). Defaults unchanged. On the rig it reopens the 8 Oct "lag: report it, do not fix it" (line 112) and needs his word.
DECIDED 2026-10-17  Morning answers (Wesley ~10:30, docs/reports/morning_report_2026-10-17.md):
  (1) False touchdown: once every drone is low enough it should just disarm, ~30 cm is fine. Seen in R2032/R2044/R2045
      (and R2033): the stall test fired with the drones 0.46-0.53 m up.
      Asked again (~10:40), he chose: stall AND every drone within 0.30 m of its own takeoff height. A drone that
      stalls higher (on the fallen ring, a mat) still announces LANDED after a fallback time, so the fleet never hangs
      armed. Card + critic first (a gate that lands the fleet), behind a switch, twin before the rig.
  (2) Vertical thesis claims (C-D2/C-D3/C-D5 falsified on the freeze stack): confirm the cause first, then rewrite.
      4 runs with cable_source model on D-Z0 and R-m+20-0 (app_pushz_none_cm, app_rob_m+20_z0_cm).
  (3) Corner cutting: build the measured lead (plan_ref_lag_s from the measured plan age, with ref_time_shift),
      behind an off-by-default switch, card + critic first.
  (4) N2 band rule signed as drafted: band = [max(0, min_twin - 2 sigma_rig), max_twin + 2 sigma_rig], f = 1,
      sigma_rig from RIG-1014-lift3 (docs/experiments/2026-10-16_bands_1m.md).
DECIDED 2026-10-17  Corner-cutting build (Wesley ~11:00, after the measured-lead critic said rewrite): sweeps first, then decide.
  No new code. Fixed leads 0 / 0.04 / 0.075 (ref_time_shift + plan_sqp_iters 2) on the orbit at 0.125 m/s, and on a
  delayed twin (orbit 0.3, SIM_MOCAP_DELAY_S 0.035 + SIM_MOCAP_VEL_FILTER ema). This tests lead = delay against
  lead = kappa x delay. Options afterwards: the stamped shift (wire format, his word), the measured lead, rung c.
DECIDED 2026-10-17  Touchdown (card docs/experiments/2026-10-17_touchdown_height_gate.md, twin R2088-R2108; Wesley ~14:00):
  keep A (`land_rearm_touchdown`, re-arm at ring touchdown) and delete B (`land_height_gate`: it never acted, 15.8 cm
  < 30). A becomes the default, rig included, once the reviewer supports it. The residual (a drone 3 % stronger than its
  map hovers 16 cm above rest with the target at the floor, then disarms and tips) gets a new card + critic: after ring
  touchdown the LAND target keeps descending below rest so the position loop brings it down, ground_idle idles it.
DECIDED 2026-10-17  Corner cutting build (Wesley ~14:29, after the sweeps R2109-R2132): no new code. The combination
  ref_time_shift + plan_sqp_iters 2 + plan_ref_lag_s 0.075 becomes the twin default candidate: neighbours (hover,
  lift and LAND, detach, attach on the orbit, figure-8), the reviewer, then Wesley's word for the rig. The measured
  lead is dropped (lead = delay falsified: 0.035 s more delay moved the crossing ~0.003 s). Pushed back once on the
  7 Oct "no bandaid" point; his call.
DECIDED 2026-10-17  Evening answers (Wesley ~19:45, after the batch R2133-R2171 and its reviewers):
  (1) Touchdown re-arm A (`land_rearm_touchdown`): default ON in the twin now (src/bringup/config/sim.yaml); the rig
      after a critic look at its wall-clock fresh-pose check (LAND_POSE_FRESH_S 0.2 s, never run at RTF 1), then
      through a configs/rig/ params file. Reviewer: supported for the twin (rig LAND path off 2 of 4 false touchdowns,
      A 0 of 5; R2133-R2137, R2164-R2167).
  (2) `land_descend_to_stall`: re-card and re-fly. The residual is gone (F1), but F2 fired as written (drone 0 lands
      at rest, 4.7 cm below the base) and floor contact speed rose to 0.17-0.35 m/s (0.04-0.14 off). New revision: a
      one-sided height bar and a contact-speed bar, then fresh runs. Stays off.
  (3) Corner cutting closed: today's defaults stay, the lag is reported (8 Oct decision stands). The fixed-lead
      combination failed N5 on spin in 2 of 3 pairs (R2150/R2151, R2168-R2171); its switches stay exploratory.
  (4) Next: freeze the N2 1 m bands (signed rule), then the 21-22 Oct rig-day sheet with today's LAND findings.
DECIDED 2026-10-21  The supervisor's results spec (Wesley ~18:50, plan docs/plan_thesis_results_2026-10-21.md approved):
  (1) Baseline: hover, figure-8 and line_spin at 0.2 m/s with 2 (plates 0/6), 3 (0/4/8) and 4 quads (0/3/6/9); one
      rig flight each, twin 2 each. line_spin = 1 m out and back while the ring turns 1 rad per 0.5 m travelled
      (~229 deg), landing by itself at the end like the figure-8.
  (2) Novelty: 4->3->2->3->4 in the twin on the three trajectories x 3, one lap per stage (4, 3, 2, 3, 4 quads),
      releases UNANNOUNCED at a fixed mid-lap point (no load shifted before a release: accept the tilt, minimise it
      after detection), re-attaches with the path paused between laps.
  (3) Drop rule (switch, card + critic): a loss leaving a T or an opposite pair keeps flying; a non-opposite pair,
      one quad or ring tilt > 30 deg still drops the ring.
  (4) Rig 4->3->4: Tejen's controller re-attaches QUAD4 with the path paused; start in the M1 layout (carriers
      3/7/11, QUAD4 on 9), the square 0/3/6/9 once the twin fix works; widen to more re-attaches as they work.
      The basket version (K) on the baseline three trajectories.
  (5) Every link becomes a slack ~1 m cable (the same kind as the 0.55 m ones), typed at its measured length.
      Every 0.55 m twin table in the thesis is re-flown at 1 m (supersedes 2026-10-16 decision 5).
  (6) Sim: the magnet caught off its slot (two arms: measured catch point and slot assumed); scalability to 9 quads
      at fewer solver iterations if quality holds (the 9-quad Gazebo run gives way first if time runs short; the
      payload bigger, not much heavier); the paper's method against our planner in the twin (4 quads; hover, line
      0.2 and 0.4 m/s; 3 repeats), retuned fairly, time-boxed to ~2 days.
  (7) Videos: every twin run gets one fixed wide-view mp4 (720p, real speed) rendered after the batch, plus an
      evening reel; state logs are deleted once their video is checked; disk space from a delete list he approves.
  (8) Lab: every Wed/Thu 8-11 am, a couple of Fridays if needed; results freeze Sat 7 Nov. Tomorrow's first
      priority: the offset check and the 3-quad baselines.
DECIDED 2026-10-21  Evening answers (Wesley ~22:45-23:00, after the chain card's runs R2249-R2258):
  (1) The T to the pair stays on Thu 22's rig sheet (Part C, last), gated on the ring centre-of-mass fix first
      (Motive pivot down to the ring's centre, attach_z / payload_rest_z typed), B1 clean, and the detector branch.
  (2) A carrier magnet switched off on the rig detaches at once ("because of the angle the quad would be at"), so the
      magnet-marker check should see a release within tens of ms (card 2026-10-21_tip_detect rev 2).
  (3) The trackers' measured cable-pull cap goes from 6.6 to 8 N: cable_accel_cap 12 -> 14.5 m/s^2 (common.yaml), a
      default, applied after the overnight batch; the trackers also log the raw measured pull (acm_x/y/z).
  (4) Drop rule (Wesley, before the chain card's runs; card 2026-10-21_chain_release): a loss keeps flying while
      the survivors' largest gap is at most 200 deg (for a pair, its larger arc), switch `drop_gap_deg` (default off,
      typed on the rig through configs/rig/chain_pair*.yaml) with min_survivors 2; the trackers' ring-tilt drop goes
      to 45 deg in those files. The cables stay 1.0 m for the chain.
  (5) Part C's detector branch: the magnet-marker check flies ON only if it passes its critic and the A/B bags replay
      clean (tools/tip_replay.py). Critic round 2 (~23:00) said NO-GO, so Part C flies chain_pair_noxy.yaml (the
      stretch check alone) with the markers recorded; the check's rev 3 waits for the chain's step-2 card.
  (6) Part C, no detection within 2 s of the release: LAND straight away (Wesley: unlikely).
  (7) Overnight work stops starting new work at 06:30.
DECIDED 2026-10-21  The magnet-marker check after critic round 2 (Wesley ~23:30, disagreeing with its no-go: our 46 rig
  flights, 42 min of airborne ring pose at ~88 Hz, show no ring-attitude glitch in normal flight; every glitch and
  frozen pose came after a failure with the ring at 55-120 deg; scan in docs/experiments/2026-10-21_tip_detect.md):
  (1) Add the cheap guards (rev 3: 0.1 s persistence on stamps, act only with another armed marker fresh, reject
      frozen repeats, hold off after a ring attitude step), then compare it against the stretch check in the twin.
  (2) The comparison runs after the overnight batch (solve sweep, 1 m videos, 4-quad re-runs). "Both methods should
      work; it would be nice to have both available": the check goes onto the rig tree as a switch (tip_detect_m,
      default off).
  (3) The twin's rod-pose marker publishers go into the existing T, 2-quad and 4-quad worlds (0.55 m and 1 m).
  (4) Overnight work on the laptop runs until 07:30 (Wesley ~23:55; supersedes the 06:30 halt), so the queue goes on
      after the comparison: tomorrow's A/B flights in the twin with the check on, the check's neighbours, then the
      videos and the morning reel.
DECIDED 2026-10-22  The ring's centre of mass (Wesley 08:20, bench item 0): the ring is 4.5 cm tall with its centre of
  mass ~2.75 cm above its bottom (top-heavy), so the plates sit 1.75 cm above it. The rig types attach_z 0.0175 and
  payload_rest_z 0.0275, with the ring's Motive pivot moved down to the centre of mass (it should read ~2.75 cm at
  rest; it read 5.16 cm before the move and 2.76 cm after, ~08:35). The twin's worlds keep 0.02 / 0.02 until they are
  regenerated.
DECIDED 2026-10-22  The rig's path baselines fly with no hover hold (Wesley ~08:59, after A2: the integral's lean ~1 deg):
  the path starts as the lift ends, as in the twin's baseline B configs, so the twin-vs-rig table compares like for
  like.
DECIDED 2026-10-22  Path flights fly a 5 s hover hold again, and the figure-8 is the 7 Oct size (Wesley ~09:20, after
  RIG-1022-f3big; supersedes the no-hold line above). Every flight today lifted 2-5 cm toward -x of its rest spot and,
  with no hold, the model integral never runs on a path flight (its gate opens only in hover), so the whole lap kept
  the offset (f3big lap centre -4.7, -1.3 cm; 7 Oct with its 5 s hold -0.8, +0.9). A2's hold leaned ~1 deg after the
  pivot fix. The figure-8 becomes traj_radius 1.0 (x +-1.0, y +-0.5 m, corners 0.21 m): the size 0.5 one has 0.10 m
  corners that ask 0.38 m/s^2 at 0.2 m/s, where the rig's ~0.19 s lag cuts them by ~4 cm (A3 f3). Today it is typed on
  the sheet (configs/rig/traj_hold.yaml, two_quad_hold5.yaml, traj_radius:=1.0). After the rig: a figure-8-only size
  default (traj_radius also sets the circle, spin and orbit radius, which stay 0.5), the twin's baseline B configs at
  the same size and hold, and the figure-8 bands re-derived for size 1.0 (the frozen 0.55 m and the 1 m ones are size
  0.5), so the rig's size-1.0 figure-8s stay exploratory until those exist.
DECIDED 2026-10-22  The spinny line spins at a constant yaw rate (Wesley ~09:33, after RIG-1022-ls3): one full turn
  (360 deg) over the out-and-back, the rate ramped up and down once (smootherstep, 4 s as the orbit and figure-8; ~24.4
  deg/s held through both legs and the turnaround), REPLACING the 1-rad-per-radius law (229 deg, the rate through zero
  at the turnaround). ls3 showed the ring's yaw 0.43 s behind the desired (error -5.6 mean, max 11.5 deg, growing with
  the rate) and a +-8 deg/s yaw wobble set off around the turnaround. Expect the stop-start wobble gone and the delay
  error steady at ~0.43 s x the rate (~10 deg); removing that needs a yaw lead (separate card, critic). Built after the
  rig (src stays as flown today), twin first; RIG-1022-ls3 and the overnight twin line_spin runs are the old law.
OPEN    2026-10-22  The carriers' mocap velocity filter `slope` (card 2026-10-14_tracker_timing B; T1 env
  MDC_MOCAP_VEL_FILTER=slope MDC_MOCAP_VEL_DRONES=...) as the rig default? Rig evidence 22 Oct: every two-quad lift
  over two days swung at 1.3-1.6 Hz; the plan's age (ref_time_shift, h2rts) did not help; with the slope filter (the
  quads' velocity 45-47 -> 16-17 ms behind) the two climbs had about half the 1.4 Hz swing and it decayed (h2slope),
  and the pair flew the first full two-quad figure-8 (f2slope, 3-D RMSE 3.7 cm, path lag 0.10 s). Reviewer [weak]: one
  flight per config, drained packs on h2slope, not interleaved, c2q (ema) as calm, mechanism unproven (the throttle's
  lag behind -vz fell 41-72 -> 22-24 ms; the commanded-position trail did not change). Wesley ~11:50: Wed 28 flies the
  carriers on slope if the twin and the reviewer agree; the twin copies the rig's filter from now on; the ring on slope
  twin first. Wed opens with the interleaved A/B (bars frozen first).
DECIDED 2026-10-22  Rig-day review answers (Wesley ~11:50-12:05; docs/reports/rig_review_2026-10-22.md):
  (1) Wed 28 carriers fly the `slope` velocity filter if the twin and the reviewer agree; the reviewer rated today's
      evidence weak, so Wed opens with the interleaved two-quad A/B (old filter / slope x2, plus P4 on drained packs,
      >= 10 s at the top, bars frozen in the card first).
  (2) The twin copies the rig's mocap velocity filter from now on (SIM_MOCAP_VEL_FILTER = the rig's filter: ema while
      that is the rig default). Bands are frozen for both filters (the nine 1 m cells x {ema, slope} x 2).
  (3) The ring's own velocity on slope: twin first, then the rig.
  (4) Four-quad flights are approved for Wed 28.
  (5) Releases on the rig: a button in the RViz panel that releases the rider (Wesley prefers RViz to the console),
      not a T4 line. Part C again on Wed 28 at 1 m, after the pair A/B.
  (6) The Wed sheet carries everything in priority order; overfull is fine.
  (7) QUAD2's FC `!ERR*` ARM refusals: live with it (a re-ARM clears it).
  (8) The spinny line's ~0.45 s yaw lag: see what the slope filter does first; card a yaw lead (critic first) only if
      the lag stays above ~0.3 s.
  (9) The -x push: accept the 5 s hold (the integral learns it); no rig time on its cause.
  (10) Chain step 2a (the force-vector split) is DROPPED: the rig T hung 2.6 deg mean, 2.2 median (t3p_2, 49 s,
      int_k_xy 0), as the twin's 2.3; the chain work goes to step 2b (the reaction to a loaded release).
  (11) If the interleaved repeat confirms it, the velocity-filter finding is a thesis result (the twin had no filter
      lag and missed the pair swing).
  (12) The magnet margin (releases at ~6.4 N under a swing, ~5 N steady per magnet in a pair): nothing for now.
DECIDED 2026-10-22  Band scoring for path flights (Wesley ~14:15: "path only, and in general I am not worried about error
  due to lag, as long as the trajectory is followed closely, that is fine"), from the Wed 28 flights on (this morning's
  RIG-1022-f3 score stays as scored): the N2 band's path error for a figure-8 or spinny line is CLOCK-FREE, the ring's
  distance to the path (sideways, from the path the reference traces, and height), RMS from the path's first move to
  the start of the planner's own descent (or LAND if earlier), for the twin and the rig alike; the tilt peak over the
  same window; the lag in seconds is reported beside it, not banded. Hover cells unchanged (2 s after the top to LAND).
  Reason: the old window ran to LAND, which in the twin takes in the descent and 17-31 s on the floor (reviewer 22 Oct)
  and on the rig depends on when LAND is pressed; and a time-indexed error counts the lag twice.
DECIDED 2026-10-22  Ultracode push scope (Wesley ~14:20): Wednesday readiness (the clock-free bands, the Wed sheet, the
  RViz release button's visibility; the 1 m defaults and a figure-8-only size default of 1.0; finding what makes the
  rig pair swing), and the thesis sim results U (magnet off its slot), S (scalability), C (the paper's method against
  ours) and R (the 0.55 m twin tables re-flown at 1 m). Chain step 2b not in this push. The lab PC may be used whenever
  it is free (who / load / containers checked before every batch, at most 4 slots, yield to anyone). Code: "as far as
  you like, just make sure to report stuff that doesn't work and delete it, and inform me if something you do
  improves results" (this push's word for code and defaults; still no commits until Wesley says).
DECIDED 2026-10-22  Push plan answers (Wesley ~15:30; docs/analysis/push_1022_plan.md): (1) the R, U, C and S claim tables
  fly the UNDELAYED twin (today's bands hold); the delayed twin (~50 ms extra command delay, calibrated on R2397-R2404
  against rig f3h / ls3c) is a named rig-timing arm for the pair work and key cells, once its landing no longer faults.
  (2) S: 8 quads WITH the return: 8 -> 4 (-> 2) -> 8, so rejoin_departed (code design B) is built in this push (card +
  critic); a 9-quad steady hover only as an extra. (3) Up to 3 agents implement in parallel git worktrees (code and
  offline tests only, today's uncommitted diff applied first, integrated into the main tree between runs, nothing
  committed). The remaining 24 questions: asked one by one.
DECIDED 2026-10-22  Push answers 1-6 (Wesley ~15:40): (1) the rig on Wed 28 / Thu 29 gets the 1 m defaults and the
  read-only tools, plus anything found that improves results; (2) attach_cable_len stays QUAD4's rod from its pivot, a
  config that disagrees with its world is refused, old configs keep a named exemption and run as flown, new 1 m configs
  leave it untyped; (3) the ring's measured centre of mass becomes the rig default (real.yaml attach_z 0.0175,
  payload_rest_z 0.0275; the attach/M2 modes' forced rest 0.05 -> 0.0275); the twin keeps 0.02/0.02; (4)
  DEPARTED_CLEAR_R stays 1.2 m for the rig ring (scaled for S's bigger ring), the path-clear and take-back distances
  scale with the formation's reach, after one 1 m twin take-back test; AND a general limit: any quad's x or y beyond
  +-1.75 m (the mocap/world origin) makes the fleet LAND (an envelope LAND, not a disarm), with every commanded or
  landing point kept inside it; (5) THE FIGURE-8 IS SIZE 0.9 at 1 m (x +-0.9, y +-0.45 m; the outboard quad peaks at
  +-1.625 m), not 1.0: the twin's figure-8 bands are re-flown at 0.9; (6) one approval for every generated world and
  model in the push (1 m attach family and QUAD4 models, the M1 layout at 1 m, R's 1 m layouts, S's bigger-ring worlds
  and drone models 4-7); params.py gains fig8_size (0 = follow traj_radius), load_ixx/load_izz (0 = from the mass) and
  prebuild_sizes.
DECIDED 2026-10-22  Push answers 7-24 (Wesley ~15:45-16:05):
  (7) one frozen code tree for the R, U, C and S claims (S's new code behind its switch); the carriers' velocity filter
      is typed in every claim config after Wed 28's A/B. (8) The twin may keep the size-1.0 figure-8 for its own tables
      ("it is fine to run 1 meter on sim, keep those results": R2329-R2364 stand); the rig flies size 0.9 with bands from
      twin runs at 0.9; T7t on the 22 Oct clock-free path-only ruler with the carry card's values. (9) A 1 m band for
      Thu 29's QUAD4 releases (Nr hover): 3 twin runs as soon as possible, 2 sigma_rig from Wed's first 1 m hover; and
      in general NO time constraints: work through the plan in order as things get done. (10) R: the 0.55 m twin-rig
      pairs and the attach-sequence figures stay 0.55 m history; the 1 m tables replace the 0.55 m twin tables, one
      appendix table compares them, the 22 Oct bands replace the old 1 m band table, F12b drops the 0.55 m rig points.
      (11) The integral gains stay (revisit if Wed's 1 m holds oscillate). (12) Tip-hinge twin world approved; Wesley
      measured the cable's rotation point 4 cm above the magnet face, ~8.5 cm above the floor with the ring down, i.e.
      ~5.75 cm above the ring's centre of mass (the rig types 1.75 cm today); the marker bodies' origins are not the
      rotation point; rig typing later, as its own A/B. (13) Betaflight read-only diff: Wesley can do it today (no
      ring needed). (14) Wed flies the pair A/B as carded, readouts from its logs. (15) drone_mass 0.583 at 1 m now
      (rig and prebuilt solvers). (16) QUAD4's 1 m tether: our typed QUAD4 record feeds his launch (after the sync2
      merge, Wesley's call). (17) U as designed (M1 layout; the catch point moves with the aim, weld radius 0.025;
      along the rim and inward, outward once at 8 cm; a newcomer within 0.30 m of a carrier after the weld fails;
      extend to 20 cm; the critic waived for the twin-only slot arm). (18) S's ring R 0.5 m, 0.9 kg, and its 'not
      heavily worse' bars as proposed. (19) Two 12-core lab slots for S's 8-quad batches. (20) S's speed-ups default in
      the twin after a smoke run and identity tests, on the rig after the reviewer. (21) C flies both network forms.
      (22) C's headline: clock-free path error plus a reach term, lag and time-indexed error beside; hover from the
      hand-over with the first 5 s reported; 'our port of Quan et al.'s method' with its deviations listed. (23) The
      twin network mode's defaults approved, and may be changed where something seems off or to test improvements.
      (24) C2 after R, if there is room. Disk: delete nothing; Wesley is getting 400 GB of storage, will update.
DECIDED 2026-10-22  Evening answers (Wesley ~22:05, after the critics):
      (1) The rig's paths shrink to 0.75 m so the position limit has margin: real.yaml fig8_size 0.75 and traj_distance
      0.75 (outboard quad ~1.475 m at 1 m cables, 27 cm inside +-1.75). The twin profile keeps 0.9 / 1.0; claim configs
      type their sizes. How the rig turns pos_limit_m on (real.yaml or each rig params file) stays open until the LIM
      runs (critic M2). (2) The rig gets its own solver folder: rig_flight.sh and the rctl/rdiss shortcuts export
      MDC_ACADOS_ROOT=~/mdc_rig_generated, filled by tools/prebuild_rig.sh at 0.583 kg; twin runs keep <repo>/generated.
      (3) QUAD4 landed by Tejen's own LAND (before we fly it) may land outside the limit: accepted, noted in the card.
      (4) At most 2 agents running at a time (token control).
DECIDED 2026-10-22  The 1 m clearance scaling stands (Wesley ~22:40, push answer 4's condition): TB-1m R2422 kept every
      clearance (QUAD4 1.39 m, its ball 0.63 m from any carrier); its tip at touchdown is the twin's rigid 1 m rod
      (TB14 mechanism: the ball reaches the floor first), recorded as a twin effect, no twin fix. The rig's tether goes
      slack.
DECIDED 2026-10-22  After batch 2 (Wesley ~23:20): (1) a dropped quad landing near or on the ring does not matter on the
      rig ("in real life it does not matter if the quads land on top of the payload"): no fix to the drop step-out;
      LIM-c's clearance bar failures (R2441-R2444, both arms) are recorded, not acted on. (2) No 1 m push world: the
      WRENCH breach test LIM-e2 is dropped (its config removed); the limit's trip rests on its unit tests, on LIM-e
      R2439/R2440 (it fired once, as built, on a quad that started outside 1.2 m) and on LIM-a R2435-R2437 (no trip
      at 0.75 m, reach <= 1.469 m).
DECIDED 2026-10-22  The position limit is on for the rig through real.yaml: pos_limit_m 1.75 (Wesley ~23:40, critic M2).
      The twin and legacy profiles keep it off (their tables fly paths past it). Evidence: LIM-a R2435-R2437 (no trip on
      the 0.75 m paths, reach <= 1.469 m), LIM-e R2439/R2440 (one LAND when a quad was past the line), unit tests.
DECIDED 2026-10-23  Morning answers (Wesley ~07:25, after the overnight report):
      (1) Wed 28: MDC_PLANNER_CPUS=0-7 (the planner pinned to the rig laptop's P-cores), no desk check first. (2) Wed flies
      the full merged tree (every new feature behind its switch, off on the rig). (3) Pair-swing round C approved: the
      tip-hinge twin world (hinge 5.75 cm above the ring's centre of mass); the rig-timing switches stay until it reads.
      (4) R2511 (runner stall after landing) stays a valid run. (5) S: 8 quads reported as "twin only, not real time",
      the real-time limit (~4 quads on the rig laptop) its own number; its bars accepted ("not so important as long as it
      works"). (6) C: reported as is ("the network as ported cannot hold the ring's yaw in the twin"; our planner 46 mm vs
      156 mm on the 0.4 m/s line); no harness fix; its seven card values accepted. (7) U: separation reported apart from
      the arm comparison; the 11.6 cm wrong-plate cell x2; "model-error tolerance" wording past ~2.5 cm; the mechanism
      bar kept. (8) R: build all five missing 1 m worlds (Tier B's four gap layouts and the push world) and fly those
      tables. (9) The 2-quad landing (L-S) at 1 m is not reported in the sim results. (10) Rejoin: fly the leaning-pair
      neighbour. (11) Videos: render the overnight highlights plus a reel.
DECIDED 2026-10-23  The rejoin leaning-pair test's world approved (Wesley ~08:00): the 1 m world the rejoin E2 test flies,
      with Gazebo's ApplyLinkWrench (the push plugin) added (it is the 3-quad T world, three_rigid_ground_rig_T_push_L100), so a steady 0.0066 N m push holds the pair 3 deg toward the
      rejoin plate (and -3 deg the other way); no config-only lever exists (the ring pivots on two ball joints).
DECIDED 2026-10-23  Late-morning answers (Wesley ~10:00-11:10):
      (1) C: fly the network search's real best point in the twin (`c_net_portfinal`: diss_c 12, k_pay = k_anchor 80,
      k_ring 40, k_slot 18, ki_load 0.5, vel_k_att 8, vel_kp_pos 1, vel_kv 3; J 115, every offline guard passes). The
      "retuned" runs R2468/R2469 flew the search's stage-1 point (only diss_c and vel_kp_pos typed; the emit filled the
      rest from DEFAULTS), so (6) of the morning answers was decided on a wrong premise; hover, line 0.2 and line 0.4 x2
      in lab batch 17. (2) Thesis text corrected to the measurements (thesis worktree, uncommitted): system_overview's
      latency (radio + FC ~10-15 ms; throttle-to-lift 44-67 ms rig vs ~15-25 twin; body rate 17-24 vs 49-57) and
      reconfiguration's drop rule (default: fewer than three drops; the chains' 200 deg gap rule). (3) Pair swing:
      round C null (R2598-R2601, typed0175 M1(b) 0.09/0.08 deg < the 1.0 testbed bar), so round D is reduced to the
      config-only rig candidates on the thr060 testbed (slope x2, cable_source model x2; bars frozen 11:14); D3-D5 not
      flown; Wed 28's 1 m rig pair is the test. (4) Delete rate_kff, thr_tau_s, the body-rate filter emulation and the
      tip-hinge world (the card's §8 rule; backups in the scratchpad's del_rt/); keep the per-channel command delay
      (default off) as round D's testbed.
DECIDED 2026-10-23  Thu 29 (Wesley ~11:20, after the readiness pass: tejen-sync2 merged into the tree, uncommitted):
      (1) QUAD4's transit at 1 m with carry_clearance 0.25 (body ~3.25 m, 0.75 m under the ceiling; the tether tip
      0.25 m over the carriers' bodies; the default half-cable gave ~3.5 m): in the sheet's $H and the twin rehearsal
      configs. (2) Nr flies our announced DETACH (his unannounced release is not built); the sheet orders his planner
      before his controller and gives the QUAD4 dip > ~0.3 m -> GIVE UP rule.
DECIDED 2026-10-23  U after batch 16 (Wesley ~14:01): (1) the 6 owed U runs (u_meas_t08 and u_slot_t04 re-flies, the
      pre-registered 20 cm radial extension x2 per arm) fly on today's tree beside batch 16's 54 counted runs, the table
      noting the two trees (what U executes is unchanged: graph_attach's changes are partner-only, the deleted
      switches were off in every U run); U re-flies whole on the final tree at the results freeze, like R. R2634 is a
      runner race (mocap staleness raced the 2 s stop after /fleet/landed), VOID and re-flown; the runner no longer
      checks staleness after /fleet/landed (tools/experiment/runner_node.py, test added). (2) Along the rim the mechanism
      is NOT SHOWN (the scorer's pre-written code; radially in it is shown); the thesis says the slot arm folds in more
      gently along the rim, not that the model point does not matter.
DECIDED 2026-10-23  Lab busy and E5 (Wesley ~14:05): (1) another user took the lab PC at ~14:00 (15.6 -> 23 of 24 cores:
      a DOPE detector, a relative-position node, their Gazebo); batch 18u was stopped at 14:03 under the "only when
      nobody else is on it" rule (R2714-R2724 VOID, containers removed). Wait until it is free (a watcher probes every
      5 min), then relaunch U's 6 owed runs + our planner's 5 C cells, then S's 8-quad batch. (2) E5 reported as is:
      1 of 2 clean toward the weld plate (R2712: fold-in 5.28 deg for 0.19 s, the weld-height cap's 1.1 cm dip, the
      critic's M1); no change to the cap.
DECIDED 2026-10-23  Thu 29, his install gate (Wesley ~14:28): the merged C1F backend discards any finished transit plan
      when the world version moved since its final check, and our carrier (50 Hz) and ring pose (20 Hz) updates move it
      constantly: 3,533 of 3,541 accepted plans discarded in the five twin rehearsals (R2695-R2699), grants after 8 s to
      over 3 min (pre-merge handoff < 0.6 s); the rig runs the same code. (1) The fix is Tejen's: his install re-checks
      the finished plan against the current world instead of discarding it (note for Wesley to send:
      docs/rig/note_tejen_install_gate_2026-10-23.md; a critic looks at the change when it comes). Meanwhile the Thu 29
      sheet has a stop rule: GIVE UP if no TRANSIT_TO_REATTACH 60 s after his 'settled' line. (2) carry_clearance stays
      0.25 (asked again with the new number: ~5-10 cm magnet to a carrier's body in his model on the transit, ~30-35 cm
      at 0.5). The twin A/B (0.25 vs 0.5, x2) and the R2696/R2697 re-flies fly when the lab is free (wall cap 60 min).
DECIDED 2026-10-23  S (Wesley ~14:33): (1) N4's window opens 2 s after the last REJOIN_BLEND_OVER for the not-yet-flown
      8-quad claim runs (the 10 s rejoin blend postdates the card and put the last fold-in inside the original window;
      the fold-in is scored by N3 and the rejoin card); the original window is reported beside it (s_metrics n4 /
      n4_resize; test added). Exploratory R2597: revised 6.3 mm / 0.89 deg, original 19.3 mm / 1.65 deg (bars 5.72 mm /
      1.44 deg). (2) Design harder S cases (the chain on trajectories, departed quads doing tasks before rejoining,
      overlapping events): an agent drafts a ranked, PROPOSED addendum in the S card with configs where no code is
      needed; Wesley picks what flies.
DECIDED 2026-10-23  S's iteration count (Wesley ~14:37): the 8-quad claims fly at 5 SQP iterations, as the card's frozen rule
      says after Q6 (n = 8: i1 misses Q1-Q4, i2 misses Q4 by 0.016 m/s^2; i5 passes all): configs
      s8_hover_R50_L100_it5 and s8_chain4back_R50_L100_it5 (N1's comparator s4_hover_L100_it5, R2700/R2701), x2 each,
      lab --slots 2 --cpus 12 --wall-cap-min 150 --warm-from-config; still "twin only, not real time". The it1 8-quad
      runs stay exploratory; the nine-flight replay (Q1-Q5) is reported when it ends.
DECIDED 2026-10-23  S's harder cases and N2 (Wesley ~15:12): (1) the five no-code cases fly as exploratory first passes in
      the addendum's rank order (case 1's neighbours s8_chain4b2b and s8_orbit before its orbit chain; then the figure-8
      pair, two-at-once, release-in-blend, relay), 9 runs after the 8-quad claims (batch 18, now 13 runs). (2) N2's
      comparator for the claims (5 iterations): the 5-iteration release cell, mean(R2708/R2709) + 1 = 8.83 deg (was
      9.23 from the it1 cell); written in the card before the runs. (3) Cases 6 (a freed quad's task before the rejoin)
      and 7 (the lap-by-lap chain): critic WAIVED by Wesley; built behind switches (default off, twin only) when an agent
      slot frees; the waiver goes in their registry rows.
DECIDED 2026-10-23  Network yaw, figures (Wesley ~15:26): (1) Why the network lets the ring's yaw grow: Quan et al. model
      the load as a point with every cable at its centre of mass (their Eq. 1: load position only), while our ring hangs
      from four rim plates, so it has a yaw mode the method has no term for; our port's yaw-following formation (the
      locked yaw-only design) feeds it (offline: x1.6/s from a 0.5 deg kick; formation held in yaw: x1.1-1.3/s). No
      coding bug found yet. Wesley: fly the paper's OWN geometry, all four cables hooked at the ring's centre, both
      controllers, in the twin (world SDF and attach geometry approved for this test); an agent builds the world and
      configs (PROPOSED section in the C card). (2) The ring is black in every thesis figure: plot_style.PAYLOAD_LINE
      '#000000' (was gold); existing thesis figures change on their next render. (3) The rig pair-swing figure is like
      for like: ema h2t9p vs slope h2slope (h2, h2rts, f2slope in the text). (4) Time axes read "t after <event>".
DECIDED 2026-10-23  C and the drafts (Wesley ~15:44): (1) Fly the paper's own FORM of the network (Eq. 9 damping relative to
      the formation + Eq. 10 acceleration) at its search-best gains (configs/twin/c_net_paperfinal.yaml: diss_c 3, k_pay
      = k_anchor 80, k_ring 40, k_slot 18, ki_load 0.5, vel_k_att 16, vel_kp_pos 1.5, vel_kv 3; offline J 12.1 vs the
      port's 115, excluded only by the 0.6 m/s line's tilt guard): hover, line 0.2, line 0.4 x2 in batch 18u. (2) C's
      draft: the grid's-point twin rows (R2468/R2469) and the inferred trade-off sentence cut; the paper's form and
      geometry tests named as pending. Also (my calls, told to Wesley): system_overview's body-rate numbers now the
      pair-swing card's (loop_timing: 25-35 vs 48-62 ms, was the Betaflight note's 17-24 vs 49-57); tab:faults' tilt and
      drop-off rows carry the 1 m results (G165 drop; G180 disarm 1.84-2.00 m). Thesis builds (148 pp, 0 undefined).
DECIDED 2026-10-23  The paper's own geometry and the figures (Wesley ~16:22): (1) the centre-hook test: world
      four_rigid_ground_rig_hub_L100.sdf (every rod on a 1 mm hub on the plates' plane, 2 cm above the ring's CoM, as
      built), configs c_{net,pl}_hub_{hover,linex_v200,linex_v400}_L100 (attach_radius 0.001; zero-radius planner code
      NOT funded), bars G/P/A/O FROZEN as written; smoke pair first (front of batch 18u), then the 6 cells x2 + the
      network as first ported on the hub x1 each (c_net_smoke_hub_*; batch 18h). (2) Best-results figures: detach
      r302_3; the spin goes in the summary; spinny line ls3c; "too much black": colour the data, one palette for fleet
      size, grey for references, black only for the ring among drones (memory updated).
DECIDED 2026-10-23  Quan et al. exactly (Wesley ~21:10): the comparison arm should be their method as published, not
      our port; changing the world's geometry to match their model is fine (his word for the world SDF and the attach
      geometry of this arm). Next: a gap analysis of every remaining deviation (card §3.1) against the paper
      (docs/papers/quan2025_self_organizing_aerial_swarm_2509.03563.pdf), each marked config / switch / code; code in the
      flight loop gets the critic, switches default off; then a card with frozen bars before any run.
DECIDED 2026-10-23  Quan et al. exactly, the choices (Wesley ~21:45, on docs/analysis/quan_exact_gap_2026-10-23.md):
      (1) "as published" = the paper's TEXT (Eq. 3, 7-10, a per-robot force law), their code only to settle what the
      text leaves open, each reading stated in the card and thesis; built as a new module behind `net_law: port|quan`
      (default port): new flight-loop law -> critic before any run. (2) World C: every cable on a 1 mm hub AT THE RING'S
      CENTRE OF MASS (--attach-z 0) with the cable joint at the drone's centre (--pivot-dz 0); 4 quads, 1 m kept; the
      ring's tilt is then free, so the 30 deg tilt rules (scorer's LOSS, our arm's drop) come off for BOTH arms on this
      world (the card defines the loss without them). (3) E1 + E4 (the paper's form on today's hub, hover + line 0.4,
      once each, laptop) now; E2/E3 not flown. (4) Port switches S1-S4: not built ("whichever matches the paper more
      closely": the exact law). Goal in his words: "we want to be able to get their results".
DECIDED 2026-10-23  C's port runs and the locked "yaw-only" line (Wesley ~22:00: "whatever matches the paper the
      closest"): the paper's formation does not turn with the load at all (a point load), so neither yaw-only nor the
      full-attitude frame C flew (diss_balanced_tensions false) is the paper's. C's port rows STAND AS FLOWN with the
      difference stated (card corrections; row R2685c); no yaw-only re-fly. The exact law (net_law quan) uses no load
      attitude. The locked rule stays for the network mode in general; C's port arm is a recorded exception.
DECIDED 2026-10-23  Results backup and laptop space (Wesley ~22:10; supersedes "delete nothing" for these runs only):
      copy ALL of results/ to his UNSW OneDrive (5 TB; rclone remote, folder multi_drone_control/results), verify it
      file by file (rclone check), THEN delete from the laptop the sim run folders dated before 2026-10-15 that no figure
      script, paper, thesis draft or tool names (1,009 folders, 18.7 GB as of 22:05); keep the 317 cited ones, every
      rig run, videos, analysis, logs and the registry. Restore with rclone copy from OneDrive.
      UPDATE 22:20: UNSW blocks rclone ("admin approval required"; not worked around). Instead: per-day archives
      ~/mdc_onedrive_upload/results_<day>.tar.zst (pack.sh: tar | zstd -6, each verified against the disk with tar -d,
      sha256 + size + file count in MANIFEST.txt), uploaded by Wesley through the OneDrive web page, sizes checked there;
      only then the uncited pre-15 Oct sim runs of those days are deleted. Rounds: Aug-Sep, then 1-14 Oct, then the rest
      (backup only). Wesley may also ask UNSW IT to approve rclone for later syncs.
DECIDED 2026-10-23  Quan exact law after its critic (Wesley ~23:20; card .claude/worktrees/agent-a4e026ca138b17b9f/
      docs/experiments/2026-10-23_quan_exact.md "## Critic"): (1) R7 -> R7b: v_d = measured velocity + a_cmd*dt (no
      accumulation), a_cmd as the acceleration feed-forward (S1: PX4 takes velocity AND acceleration commands); the
      critic's offline checks (a) self-stress wind-up signature, (b) 3 quads, (c) R7b with 4 first; R4 (h_c)
      re-derived from init.m. (2) Nodes: a reading switch, algebraic (main text Eq. 3) and dynamic (S2/S4 with init.m's
      masses), compared offline; the one that holds flies, the card says which and why. (3) The trackers' 60 deg ring
      envelope OFF on world C (four_rigid_ground_rig_hubcom_L100), twin only, both arms, by a switch in that world's
      configs; the rig and every other world keep it. (4) S: the chain claim's N4 miss (6.36 / 6.35 vs 5.66 mm) is
      reported as missed (frozen bars); s4_hover_L100_it5 re-flown x2 on the current tree for one tree per claim table.
DECIDED 2026-10-24  Wesley's answers to the morning report (~09:10): (1) the rig keeps 5 SQP iterations ("it is already
      working as is"): the 2-iteration idea is closed; it2 stays a twin result. (2) Quan readings: vel_k_att 8 for the law's
      arm; the vertical by the critic's split of R6's modelled pull (vertical (m_L/n) g, horizontal (m_L/n + r) g cot phi,
      no seed, no estimator); R6b's integral recorded and its code deleted. (3) Quan bars: freeze after he has seen videos
      and accuracy figures of the trajectories. (4) Test the line overshoot's cause. (5) R4's conflict: Claude's call ->
      h_c from init.m (3.41 cable lengths: what sets the node plane in their simulation), the Fig. 5 / S6 '2.02 m mean
      cable' stated beside it as their hardware's. (6) Commit the Quan code (authored by Wesley, no attribution). (7) Backup
      round 2 uploaded by him. (8) S's chain reporting after he sees videos. (9) Tejen's note sent to him to forward.
DECIDED 2026-10-24  Quan R7c (quan_vcmd feedforward: v_d = the measured velocity, a_cmd only in the acceleration
      feed-forward; the offline cause of the line overshoot is R7b applying the path's acceleration ~1.3x): critic WAIVED by
      Wesley (~09:55); T2 (c_net_quan_r7c_hubcom_linex_v400_L100) flies exploratory; its row carries "critic waived by
      Wesley 24 Oct".
DECIDED 2026-10-24  Tejen's completion-wakeup fix 1cfc7a90 ported (Tejen via Wesley ~10:55: "port that targeted change
      first ... then repeat one identical twin case"): his commit (waitable, backend integration, tests, timing diagnostics;
      not his docs or other changes) mapped onto the fork (dynamic_planner_ros -> tejen_dynamic_planner, drone_communication
      -> tejen_mission); one hunk ported by hand (the drain before the state sample: the fork's stateCallback predates his
      ego-orientation sample) and his source-text test's first assertion adapted to it. Built, his completion test 10/10, gate
      green (1952). Twin check: thu29_floorjoin_hover_L100 x2 (vs R2695 / R2738), compare installed/accepted,
      completion_to_collection_ms, certified vs installation world versions. Uncommitted.
DECIDED 2026-10-28  Velocity filter: slope (Wesley ~10:00 in the lab, 0.55 m pair QUAD2/QUAD3 on plates 11/5, after hE1 ema
      and hS1 slope: pair swing 0-6 s 2.69 against 1.06 deg rms, hover 3.98 against 0.74, see-saw growing 7 -> 12.5 deg p-p
      against decaying 8.7 -> 3.5, velocity lag 40 against 20 ms). One flight per arm, not the card's E S E S at 1 m
      (docs/experiments/2026-10-22_velocity_filter.md): his decision on this pair plus 22 Oct's (slope 1.35-1.46 against
      ema 2.60-3.72 deg). Today every T1 types MDC_MOCAP_VEL_FILTER=slope. After the lab (not mid-session): slope as the
      real_io default and the twin's SIM_MOCAP_VEL_FILTER, the working rules' twin-filter line updated.
DECIDED 2026-10-28  The new payload and the slope filter as defaults (Wesley ~19:00, after the lab): (1) the real-world
      defaults are the new payload (1.15 kg, 510 mm, slots ~100 x 85 mm; RIG-1028-payload: magnets at r 0.204, Motive
      pivot within ~5 mm of the centre at rest z 0.013; plate faces ~0.025 above the pivot and the inertia 0.024/0.049
      are estimates) in real.yaml, with the rig planner prebuilt at 1.15 kg; the M2A ring's values stay in
      configs/rig/ring_m2a.yaml and its solvers in ~/mdc_rig_generated_m2a_0p86 ("keep all the old stuff", Wesley).
      (2) slope is the mocap velocity filter default in the real world (every carrier; attach mode leaves the newcomer
      on ema) and the twin (SIM_MOCAP_VEL_FILTER). (3) Thu 29 flies 4 quads on 1 m cables: baselines, the release chain,
      Tejen's attach, the pickup and basket. Open from today's pair (RIG-1028-f2b, -ls2c): the ring sank 17-57 cm when
      the trackers' measured cable pull passed the 14.5 m/s^2 cap (cable_accel_cap); the twin p115 runs test it before
      any change.

- DECIDED 2026-10-29 (Wesley, lab): the trackers' measured cable pull gets a first-order low-pass, `cable_lp_hz`
  (tracker parameter, common.yaml default 0 = off; typed `cable_lp_hz:=2.0` on the rig first, h4f). Why: RIG-1029-h4,
  the 1 m ring bent at 5.3 Hz (opposite plates in phase, neighbours anti-phase) with every tracker feeding its own
  measured pull back one tick later; a magnet released at the top of the lift. The planned pull (cable_source model)
  ruled out by Wesley ("we already know the planned pull is not good"). Critic waived by Wesley ("we dont need critic").
  Risk noted: ~80 ms added lag at low frequency (22 Oct's see-saw came from 40 ms of velocity lag).
- DECIDED 2026-10-29 (Wesley, lab, after RIG-1029-h4f): `cable_lp_hz` 2.0 is the rig default (real.yaml, section
  tracker) for every mode; common.yaml (and so the twin) stays 0. Revert: `cable_lp_hz: 0.0` in a params_file or real.yaml.
- DECIDED 2026-10-29 (Wesley, lab, after RIG-1029-ch1b/-ch1b_2): `cable_lp_order` 2 is the rig default (real.yaml): the
  measured pull through two 2 Hz stages; the 5.3 Hz bending fell to 0.07-0.09 m/s^2 (one stage: 0.43-0.60). Open: plate 0
  releases under load in 4 of 4 runs, magnets swapped included (the plate, not the magnet).
- DECIDED 2026-10-29 (Wesley, lab, after RIG-1029-sq2_2/-sq2_4: the triangle swung at 1.1 Hz with the two low-pass stages'
  ~120 ms of lag, QUAD4 still on its mat in sq2_4): the rig's measured pull goes through a band-stop at 5.3 Hz (Q 1;
  cable_notch_hz in real.yaml), the low-pass off (cable_lp_hz 0; kept in the code). Critic waived (Wesley).
- DECIDED 2026-10-29 (Wesley, afternoon, after Tejen's / Codex's review, docs/tejen/reply_2026-10-29/):
  (1) Tejen's timing fix (`mpc_command_execution:=partial_step`, oct10 weights unchanged): a twin check of the send
  path first, then ONE matched rig rehearsal with nothing else changed. (2) His hand-over drift on the rig path stays
  0.30 m (speed 0.12 m/s); Tejen recommended 0.15 m. (3) The twin's cables: every link (carriers and QUAD4) becomes a
  cable like the rig's, all the same (supersedes the 9 Oct rigid-link rule); QUAD4's first. (4) Commit the working copy
  on week7 now.
- DECIDED 2026-10-29 (Wesley, ~15:10; revises (3) above): the twin's links stay hinged rods (they already swing as
  pendulums; the rig's cable is straight and taut in flight). First add the rig's delays to the twin (command/radio,
  motor lag, mocap) so it reproduces the rig's 2-5 deg in-flight swing and Tejen's timing fix can be judged offline;
  segmented cables for every link later, as a second step if needed.
- DECIDED 2026-10-29 (Wesley, 16:45; after the rig-timing twin, R2866-R2871): Tejen's timing fix is judged on the 60 ms
  twin (E2c: 2 runs, bars frozen 16:46 in the card 2026-10-29_rig_timing_twin_attach, with Codex's send-path checks),
  then the one matched rig run. The 50 ms twin did not reproduce the rig's swing (E1 falsified); 60 ms partly does.
- DECIDED 2026-10-29 (Wesley, 17:20; after E2c, R2872-R2875): Tejen's timing fix does not fly on the rig for now.
  With partial_step, QUAD4 flipped in its climb in all three twin runs with a 50-60 ms delay and was fine without it;
  his current send climbed normally at every delay. Next: an exploratory channel split in the twin (the 60 ms delay on
  the throttle only, then on the rates only, one run each), and a note to Tejen with the twin result.
- OPEN 2026-10-29 (19:35, for Fri 30 morning): type the debounce on Tejen's descent restart on the rig (sq6:
  `descent_debounce:=1.0`, 8 cm; sq6b: + `descent_abort_xy:=0.10`)? Replay of sq5_2: full descents surviving 0 % as
  flown, 49 % / 85 %; twin R2878-R2885 (exploratory). A gate in his code: Wesley's word, Tejen told.
- OPEN 2026-10-29 (19:35): the 2-quad baselines (B2) on the 0.86 kg ring (flights under ~90 s), or skip: the pair cannot
  carry the 1.15 kg ring under 0.95 (docs/analysis/rig_1029_replays.md W1).
- DECIDED 2026-10-29 (Wesley ~22:00, after the twin preview of the drop-off R2895, QUAD4's body at 3.30 m): "I never
  want the incoming quad to fly that high". Supersedes 23 Oct (1) (carry_clearance 0.25 over the carriers' bodies,
  body ~3.25 m): QUAD4 comes in between the carriers 0.30 m over the plates, with the object as for the attach
  (`transit` low, the pickup default; `transit:=over` keeps the old rule, typed only). The rig launch refuses any planned
  QUAD4 body height over 2.6 m unless `transit:=over` is typed. Same evening: the pickup object in the twin and RViz is
  the rig's 100 x 100 mm square magnet plate, not a ball (Wesley), placed in plate 9's gap as on the rig.
- OPEN 2026-10-30 (02:15, overnight; for Wesley before sq6b): the attach's gates after the descent
  (docs/analysis/attach_gates_2026-10-30.md): (1) our co-motion proof on body 24: type G = `tip_skip_repeats:=true
  comotion_d_ok:=0.008 comotion_speed_window_s:=0.5` (replay of f4's caught magnet: 0 of 108 start times prove as
  configured, 107 with G; twin R2906/R2908 still weld); (2) K = the bench catch spread + 5 mm, else 0.035 (0.024 is the
  old 60 mm plates'); (3) his `contact_give_up_s:=120` (45 s fits ~4 attempts). sq6b0 is the line without them.
- OPEN 2026-10-30 (02:15): h3tw, the planner's ring-position weight x2 in the triangle's hover (plan_w_pos_scale 2.0,
  configs/rig/hover_wpos2.yaml; docs/analysis/ring_drift_2026-10-30.md), bar frozen on the sheet; a planner gain.
- DECIDED 2026-10-30 (Wesley ~10:55, the supervisor's OK: "Yep. Also find the minimum arming angle in the configuration
  and either turn that off or increase it"): calibrate the accelerometer on QUAD1 and QUAD2 (it reads ~1.08-1.10 g at
  rest; their FCs drift to 25-112 deg believed tilt and refuse ARM, RIG-1029-h4, RIG-1030-h3t/-h4n) and set
  small_angle 180 on the FCs (the arming tilt limit; they fly rate mode, the console still blocks a quad on its side
  from mocap, TILT_DOWN_DEG 25). Steps and check: docs/rig/fc_request.md. Console: `--small-angle 180` from then on.
- 2026-10-30 ~11:15: QUAD1 done (Wesley): accelerometer calibrated, small_angle 90 (his value; 90 still refuses a quad on
  its side or upside down). Bench check RIG-1030-q1_acc: 1.00 g, FC tilt steady at the rod's 11.6 deg for 3 min.
- DECIDED 2026-10-30 (Wesley ~11:40): Motive streams fewer bodies (only those a flight uses), checked before T1 with
  tools/mocap_stream_check.py (rule in the working rules). Why: the mocap forwarder saturates at ~2,250 packets/s; 10
  bodies at 240 Hz delivered 225 Hz per body with every pose 95 ms late (bench: body 11 alone 7.5 ms, all 10 95 ms;
  QUAD4 blackbox in flight 95 ms; all carriers ~75-113 ms later than 29 Oct), and every ring lift of 30 Oct heaved.
  Wesley: on 29 Oct markers 21-23 were probably not streamed.
- OPEN 2026-10-30: the 4-quad T1 (tools/t1_four_markers.sh) uses 9 bodies (8, 11-14, 21-24): 2,160 packets/s at 240 Hz,
  96 % of the forwarder's limit. Drop the markers where a flight does not need them, or Motive at 200 Hz (Wesley).
- OPEN 2026-10-30: does the heave go with the delay gone? Re-fly h3t (and sq6b) with the reduced body set when 3 quads
  may fly; the twin (R2909-R2914) says 90 ms alone gives a slower rocking, not the fast heave.
- DECIDED 2026-10-30 (Wesley ~12:30, on docs/plan_rescope_2026-10-30.md): (1) the thesis's added value headlines X1 (one
  unannounced release: ours vs no resize, resize 0.5-2 s late, the planned pull, the paper's hand-over) and X4 (the
  8-quad endurance relay, carriers out to charge or on errands, ~30 min); (2) freezes: "as long as we need, so for now
  until 19 Nov, but ideally earlier" (plan to Thu 12 Nov, hard limit Thu 19 Nov); (3) real-world results report both
  setups for now (0.55 m / 0.86 kg and 1 m / 1.15 kg); (4) the comparison with Quan et al.: our port is the compared
  arm, the law as published one row (it loses the ring, R2744-R2749). Wesley also: Motive streams only the bodies a
  flight needs, his to choose. The rest of the plan's section 7 stays OPEN.
- DECIDED 2026-10-30 (Wesley ~12:35, rescope section 7): (a) new twin-only code behind switches (default off) approved,
  critic waived for `resize_delay_s` (X1's late-resize arm), the X4 relay rota and the random-failure generator; X8's
  network loss handling still gets the critic if it touches the law. (b) X4: pack budget B = 5 min, swaps announced
  (planned soft detach). (c) The chain's square-to-T step stays unannounced as decided 21 Oct (the gap rule drops the
  ring there; reported as the rule's boundary). (d) The twin's ring for claims: 0.86 kg, plus one 1.15 kg cell per
  table. Still OPEN from section 7: contributions C1-C4, re-fly scope (cited tables only), the real-world order, the
  RA-L drafts (park), and the final-tree re-fly's velocity filter (slope default vs ema as flown).
- DECIDED 2026-10-30 (Wesley ~13:10): the rescope (docs/plan_rescope_2026-10-30.md) is approved except X9 (delay
  tolerance), which is dropped; the real-world 2-quad case uses Wed 28 Oct's pair flights on the 0.86 kg M2A ring with
  0.55 m cables (RIG-1028-f2a, -ls2a, -ls2b_4, -f2b, -f2c), labelled as that setup. "I approve everything else":
  contributions C1-C4, re-fly only the cited tables, the real-world order of section 4, the RA-L drafts parked to after
  20 Nov. (The final-tree re-fly's velocity filter was not in the rescope: still OPEN, recommended slope.)
- DECIDED 2026-10-30 (Wesley ~13:40, X4 built in a worktree): the pack budget B = 10 min (the rota needs ~9.7 min:
  each swap holds the path ~46 s with one quad off the ring at a time; the 30-min mission = 3 B); the new world
  eight_rigid_ground_rig_R50_m115_L100.sdf (the 1.15 kg cell) may fly; a hover dwell at the pad stands for charging;
  the 0.9 kg R 0.5 m ring is the 8-quad claim ring.
- CORRECTION 2026-10-30 (~13:50, the pack's fact-check, docs/thesis_factcheck_2026-10-31.md): the decision above on the
  Quan et al. comparison said the law as published "loses the ring, R2744-R2749". Those runs flew the paper's form with
  its Eq. 9 damping and Eq. 10 acceleration; the exact law at its best reading (R2805, R2812, exploratory,
  docs/experiments/2026-10-23_quan_exact.md) held the ring ~0.11 m off and landed. The decision (our port compared, the
  published law as one row) stands; which published arm is the row is asked of Wesley.
- OPEN 2026-10-30 (Wesley's measurements ~14:00): the 1.15 kg ring is a thin disk 5-10 mm thick with 12 plates on top
  (both payloads 12 plates; the M2A's circular, 55 mm); a UAV with its 1 m cable 0.583 kg; magnet hold ~10 N. real.yaml's
  attach_z 0.025 ("the plate faces above the pivot", an estimate) is likely too high for a thin disk: measure the plate
  face above body 8's pivot on Wed 4 Nov (tools/payload_geometry.py, as 29 Oct) and type the measured attach_z
  (attach geometry: Wesley's word).
- DECIDED 2026-10-30 (Wesley ~13:50, "Just use the Oct 29 reading"): the 1.15 kg ring's attach_z in real.yaml 0.025 ->
  0.028, the cables' ends on the magnets above the ring's pivot as QUAD4's magnet marker read on its plate on 29 Oct (W).
  The planner's attach points are the cable ends (attach_radius 0.204 also comes from the markers); the thin disk's plate
  faces themselves sit ~6 mm below the pivot (W 0.028 less the then marker-to-face 34.1 mm). A runtime parameter: no
  rebuild. Closes the OPEN of ~14:00.
- DECIDED 2026-10-30 (Wesley ~14:00): the Quan et al. comparison shows BOTH published arms as rows beside our port: the
  exact law at its best reading (R2805/R2812: ring ~0.11 m off, landed) and the paper's damping form (R2744-R2749: lost
  the ring). G5 is narrowed: simulation predicts the trends of the real flights; the prediction-band hits are reported
  as they are (2 of 11 on path error, 2 of 11 on tilt), no band claim for the 4-12 Nov flights.
- DECIDED 2026-10-30 (Wesley ~14:05): the release chain after the square-to-T step (unannounced; the gap rule drops the
  ring there): the rest of the chain (T -> pair -> T -> square) flies from a fresh start in the T layout, and the
  square-to-T drop is reported as the drop rule's boundary.
- DECIDED 2026-10-30 (Wesley ~14:15, on the draft's open items): the final-tree re-fly flies the slope velocity estimate
  (closes the OPEN of 30 Oct); the thesis states the catch radius as 45 mm (the next attach flights type weld_radius
  0.045); the ring inertias stay labelled model values; the later ring's reference point is taken as its centre of mass;
  the magnet's hold against the angle of pull is not claimed.
- DECIDED 2026-10-30 (Wesley ~15:10): (1) X1 gains an uneven cell, the square to a T (one unannounced release in hover,
  the same seven arms x2), with the simulation's tilt drop raised to 80 deg (drop_tilt_deg) so the arms are compared
  on recovery, not all dropped (the square-to-T release peaks 57-61 deg on 1 m cables). (2) The full square chain
  4 -> 3 -> 2 -> 3 -> 4 on the figure-eight in one continuous run, in two variants differing at the square-to-T step:
  announced (soft detach) and unannounced with the drop limit raised in simulation; every other step unannounced. The
  morning's restart-from-the-T version stays as the boundary case.
DECIDED 2026-10-30  QUAD4's MPC cable with the new body 24 (Wed 4 Nov sheet): our attach launch passes Tejen magnet_drop_below_quad = proof_taut_len + 0.003 = L (~1.0525 with the new marker), and his MPC cable is then the drop - 0.044 = ~1.009 m unless typed (0.997 on 30 Oct). Recommend typing mpc_cable_length:=1.0 in the sheet's partner_args (Tejen's review 30 Oct, section 3: the marker moved, not the cable). Attach geometry: Wesley's word.
  -> Wesley 30 Oct 15:27: type 1.0 m (the sheet's $H carries mpc_cable_length:=1.0).
DECIDED 2026-10-30  (Wesley ~15:46) X3b bars FROZEN as proposed, with the lab runs at plan_res_every 1 (S's N6 scorable; the plan flown unchanged) and the tuf timing runs x3b_s{8,9}_chain4back_rt0 at 0; X2-full (the 4->3->2->3->4 chain on the figure-eight, A announced / B unannounced, both at the 80 deg drop file) bars FROZEN as proposed. The flight arena is 6 x 6 x 5 m (the wind-mapping paper; the draft and notes said 6 x 6 x 4). Not asking Torok about the Motive stream path: stay at <= 8 bodies. The supervisors' five papers are in docs/papers/supervisors/ (review docs/analysis/supervisor_papers_review_2026-10-30.md).
DECIDED 2026-10-30  (Wesley ~16:18, after X1's first pass R2915-R2978) (1) X1 is about the resize; the measured pull stays the default and measured vs planned pull is not a thesis comparison ('measured has always been better'). (2) The square-to-T result is a defect to fix before the code freeze (Fri 6 Nov): our planner settles the T 7-13 deg off level where no resize settles at 2.2 deg (sqT R2965/R2972 vs R2966/R2973; 'it shows our method is worse than doing nothing'). Diagnose from the logs first; the fix behind a switch with the critic, then the twin, then better becomes default. (3) The generated world four_rigid_ground_rig_3915_L100_p115.sdf (the M1 layout on the 1.15 kg ring) is accepted; X1's h4p115 rows stand.
DECIDED 2026-10-30  (Wesley 16:26) X8: ours against the port now, no code; the exact law's loss row to P3. The port arm is told 0.35 s after the release (the authors' own failure model), the never-told port beside it as a neighbour. Bars and loss rules frozen as proposed (docs/experiments/2026-10-30_x8_network_loss.md). X8 flies after the square-to-T fix (its rim cell is that release). X11: the exact law's re-fly (R2805/R2812) flies the kept force reading R6c (configs/twin/c_net_quan_const_katt8_r6c.yaml; R6b was deleted 29 Oct; R2813 gave the same result).
DECIDED 2026-10-30  (Wesley 16:45) X11: of the 39 cited exploratory runs, re-fly x2 those a thesis table claims (the 1.15 kg fixed-team cells of 4.1's later setup, the simulated joins of 4.3; the 8-quad set flies in X3); the 30 Oct delay runs, the 1.15 kg triangle joins and the network smokes stay cited as exploratory. Of the 18 runs from 16 Oct: drop those with a 1 m version (ta_*, tb_*: the 1 m r1m_* rows re-fly); dl_d1 and w4_gi are cited as the earlier setup, as flown. S2 (Thu 5 Nov): 2 flights per 1 m baseline cell, the third in S4 for the cells the thesis leans on. QUAD4's fold-in weld-height cap (R2829 peaked 9.9 deg; the rejoin has the cap): a card and the critic after the T fix, before the code freeze.
DECIDED 2026-10-30  (Wesley 16:52, the supervisors' review's facts) Hardware for the thesis setup table: QUAD1 and QUAD2 are CineLog35 V2, QUAD3 and QUAD4 CineLog35 V3; packs 1350 mAh; the UAV masses as measured (0.55 kg each, 0.583 kg with pack, 1 m cable and magnet; the packs are newer than the papers'). Motion capture: ten OptiTrack PrimeX 13 cameras at 240 Hz in the 6 x 6 x 5 m arena; the Motive version taken from the supervisors' paper or omitted. Every real flight ran from this laptop (the ground computer). The tracker's rate-model constants (tau 0.12 s, 100 deg/s) are not sourced in the thesis. Push week8 once the T fix is merged on top.
DECIDED 2026-10-30  (Wesley 17:04, before leaving at 17:30) Commit the work since 5a7ccd0 at the 17:30 wrap-up (no push). Overnight authority: run the critic on the T fix's card, merge the fix into week8 behind its switch with the default OFF (gate green, committed), queue its claim runs; no default changes without Wesley; a push notification and a morning report with the decision for him.
DECIDED 2026-10-31  (Wesley ~10:38, after the T variants R3206-R3216 and the critic) (1) The T fix stays OPEN, nothing merged: first investigate why the sideways integral is needed at all ("I feel like there could be something else that is wrong that causes those offsets in the real world, either the planner is wrong or the trackers"). If nothing is found, the automatic rule (the planner turns the sideways integral off when a formation has a zero-share carrier; new flight-loop code, so card and critic) is the preferred fix. The zero-minimum-pull switch was not chosen: both together settled 3.33 deg (R3206) and it was worse on the figure-eight (R3210 3.07 vs 1.57); its code stays unmerged in A's worktree. (2) Settling time is reported as a number with no bar for the new T runs; X1's first-pass U5 miss stands as scored. (3) The 1.15 kg square-to-T release re-flies at throttle_max 0.95 after X3b (R3213-R3216 sat on the 0.8 cap: not comparable). (4) The laptop stays on: lab batches may run overnight. (5) At most 2 agents at once, critics and reviewers included ("There is no rush", ~09:58).
DECIDED 2026-10-31  (Wesley 15:23, after the two "why the integral" investigations) (1) Build the cable-length re-read at the top of the lift behind a switch, default off (pytest, gate, a Gazebo run with a wrong length, critic); it flies on the first lab day it has passed those. (2) Add to Wed 4 Nov: a spirit level where the ring rests, and one extra hover right after h3t with the triangle turned 180 deg in the room (25 s, only if h3t shows no heave). (3) One new simulation world with 1 g rods and two exploratory runs (a new file, no claim, no existing world changes). (4) The stale ring pose at 4 carriers (RIG-1031-stale_pose): bench the 4-carrier tick with the existing switches on the laptop and bring a proposal for h4m1 by Monday; no flight change yet.
DECIDED 2026-10-31  (Wesley 17:16, after R3400-R3411 and the second critic) (1) The simulated T's fix: put the rod in the plan (fix balanced_tensions and the references so the planner's rod term gives a level plan), simulation only, behind a switch, default off, critic first; if it is not clean by Wed 4 Nov the fallback is int_k_xy 0 typed in the cells that form a T, with the cancellation stated. The automatic rule and the zero minimum pull are not chosen. (2) rod_len_reread reads in the pretension's hold, not at the top (this replaces the top-of-lift wording of today's earlier entry: the logs show the hand-over reading is too early, the cables do not change in flight). Wed 4 Nov flies with it off and those logs test the reading offline (R1). If R1 holds, Thursday's first triangle hover flies with the switch; if R2 and R3 hold, all of Thursday's baselines fly with it on, otherwise all off (one setting per table). Bars frozen in docs/experiments/2026-10-31_rod_len_reread.md. (3) X3b's three missed-release runs stay void by the frozen rule: re-fly them (R3218, R3221, R3227), report the misses plainly as a limit, and look at the detector's 3 cm test at 8-9 carriers read-only; any threshold change comes back to Wesley.
DECIDED 2026-10-31  (Wesley 20:04, after R3412-R3427) (1) All simulation claims fly the rod in the plan (plan_rod_level, the world's rod mass typed; default on in simulation once the soft detach is right, its runs hold and the reviewer has seen the table; real flights keep it off and refuse it). Whether int_k_xy goes to 0 everywhere in simulation waits for the rod card's 8-quad chain. R3419 (drone 0 as the T's rider, 3.37 deg) is reported as the limit. (2) X3b's 8-9 carrier chains type the marker check (tip_detect_m 0.04, those cells only), one exploratory 9-carrier chain first, instead of the plain re-fly of R3218, R3221, R3227; if it fails, back to Wesley with the 4 cm stretch check. (3) X2-full, later runs only, frozen before the re-fly: the largest tilt from +6 s to the next event replaces T6 on the path, and the ring against lap 1's reference is reported beside lap 6's bar; T6 stays for hover cells; the first pass stays recorded as missed. (4) The tracker's cable clamp (tracker_node.py, ac[2] = min(ac[2], 0.0)): an offline look at how often it binds, then a switch default off with a card and critic if warranted; no real-flight change without his word, and only after Thursday.
DECIDED 2026-10-31  (Wesley 20:37) X4's weld kick (docs/analysis/x4_weld_kick_2026-10-31.md): new relay worlds with a universal joint at the plate, damping 1e-4 (existing worlds untouched). First the 3-quad second-visit test on today's world and the new one, then the 8-quad test, then X4 and the fixed team re-fly under the same frozen bars; a floor start has to prove it. R3405, R3407, R3409 stay as scored.
DECIDED 2026-10-31  (Wesley 21:05, after R3428) With the rod in the plan, int_k_xy goes to 0 in every simulation claim (sim.yaml, flipped with plan_rod_level, after the reviewer); a table that pushes the ring types its own. Real flights keep their settings; the height integral stays until it has its own look. This overrides the 5 Oct model-integral default for the sideways axes in simulation only.
DECIDED 2026-11-02  (Wesley ~22:00, thesis) Gaps G2 (attitude through a removal on a rigid-body payload) and G5 (simulation scored against predictions frozen before the flight) are out of the literature review. Renumbered: G1 stays, old G3 (nonlinear MPC switched between team sizes at an unscheduled event) is G2, old G4 (off-board MPC on closed flight controllers with a cable's pull) is G3. C1 and C2 both point at the new G2. Thesis pushed as f77549b. Older cards and notes use the old numbers.
DECIDED 2026-11-02  (Wesley 22:12, thesis; replaces the numbering in the line above) G2 is back, G5 stays out: the gaps are G1 to G4 with their original numbers. Wesley does not want the frozen simulation predictions: the thesis drops the frozen bands of the 11 earlier-setup flights and their table (tab:res_ledger, figure F3), the prediction wording in the protocol, and the fixed-team question about them. Experiment-card bars frozen before runs are a different thing and stay. Lab chat: nothing in the thesis now depends on freezing a simulation prediction before a real-world flight.
DECIDED 2026-11-02  (Wesley 22:29, thesis) Contribution C4 changes from the composition of events (it only showed the others work) to the answer to G4: a tracking MPC per quadrotor on the ground, on unmodified flight controllers, with each cable's pull measured from motion capture. The measured pull moves from C2 to C4. Chains and the relay (X4) remain results and now show C1 to C3; X4 stays a headline result of the rescope plan. No new runs needed.
DECIDED 2026-11-02  (Wesley 23:03, thesis) The hardware setups are "Setup 1" (0.86 kg ring, 0.55 m cables) and "Setup 2" (1.15 kg ring, 1 m cables) in the thesis. Planned pull is not necessary anywhere in the thesis: no row, arm or comparison against it, and nothing rests on "the measured pull cuts the tilt by a third" any more. C4 therefore rests on the fixed-team tracking and the check against the ring's weight. Lab chat: X1 and later tables need no planned-pull arm for the thesis.
DECIDED 2026-11-02  (Wesley 23:14) (1) Simulation runs no longer need bars frozen first: "it is fine to decide after a run if it is an acceptable result". Real-world go/no-go bars stay frozen before their flights (rod_len_reread's R1-R3 stand). Bars already frozen stay on their cards as written; what was decided after a run gets said so. (2) Wed's h4m1 types plan_fast_refs:=true plan_res_every:=0, after one simulation run. (3) X4 adopts both fixes: the simulated pack does not drain (sim_pack_drain 0: B is a budget, not a battery model), and the home point is held (attach_datum_shift false), in X4's arms only (x4_*_v3); defaults unchanged.
CLOSED  2026-11-03  The rod in the plan: flip in simulation with int_k_xy 0, taking the T's dip (+5.5 to +9.0 mm)? Lab chat recommends yes, by the Fri 6 Nov freeze (docs/experiments/2026-10-31_rod_in_plan.md sections 10, 11; docs/reports/morning_2026-11-03.md).  -> CLOSED 2026-11-04: Wesley ~22:20, flip it (DECIDED below).
CLOSED  2026-11-03  X4's mission length for the claim runs: 29 leaves (about 1860 s, 3.1 B, estimate) or keep 26 and report 2.78 B with R1's miss? Lab chat recommends 29 (card section 9).  -> CLOSED 2026-11-04: 26 leaves (DECIDED 4 Nov ~07:45).
CLOSED  2026-11-03  weld_vel_filter_s 0.1 as the simulation's default (a gate): R3458, R3475 timed out on the raw gate, R3485-R3488 welded with the filter. Lab chat recommends yes, with the flip.  -> CLOSED 2026-11-04: Wesley ~22:20, yes (DECIDED below).
DECIDED 2026-11-03  (lab chat, routine) Thu 5 Nov's sheet: a missing release flies before the baselines, h3o is added for R3's same-day flight, f4 x2 is added last because RIG-1029-f4 flew the 2 Hz low-pass since removed. The plan's order is otherwise kept. Wesley can reorder it.
DECIDED 2026-11-03  (Wesley 19:16, thesis) Results are split into 4.1 Simulation and 4.2 Real World, the real world opening with what was flown in both. The fixed-team set is 2, 3 and 4 carriers on hover, the spinny line and the figure-eight, on the 0.86 kg ring with 1 m cables, 2 runs each in simulation (1 each in the real world if time allows); the real-world pair uses its best flights of 28 Oct (f2c, ls2b_5, hS2). Setup 1's October flights stay as the real-world comparison until new flights exist. Setup 2's subsection (bending, thrust, its figure-eight) is out until its fixes fly. The thesis does not mention objectives or limits set before the runs. The conclusion is emptied and renamed Conclusions and Future Work. Runs owed: docs/lab_requests.md Q007.
DECIDED 2026-11-03  (Wesley ~19:25, lab chat) Build the re-attach on any plate (QUAD4 starts welded on one plate, re-attaches on another; rig and twin). Critic waived "for all" (note it in each registry row). Q007's runs start now on the current tree (lab chat flagged once: if the rod flips before the freeze, these re-fly).
DECIDED 2026-11-04  (Wesley ~07:45) X4's mission length (OPEN 2026-11-03): keep 26 leaves; "don't worry about the time limitation for the 1700 s thing". The relay's claim is 26 of 26 swaps, about 1670 s = 2.8 B (R3519, R3564, R3565); the card's written 1700 s stays recorded as missed and is not a reason to re-fly. Card section 10, feed F038.
DECIDED 2026-11-04  (Wesley ~10:00, his supervisor: loosen the attach confirmation as much as possible) The rig's co-motion check is typed loose on the attach lines (G): contact xy 0.06 (weld_radius), normal -0.03..+0.04, contact speed 0.06; proof drift 0.02, |normal| 0.04, taut margin 0.05, speed 0.06, dwell 0.5 s, inconclusive 6 s, lost/sep 0.06, sep normal 0.04. Kept: the plate must move with the tip (require_motion), so a resting, uncaught magnet is not folded in. Defaults unchanged (typed only; new knobs in graph_attach COMOTION_KNOBS). Reason: RIG-1104-nr1_2 (taut 1-9 mm short), RIG-1104-nr9m (normal +12.1-12.5 mm against 12) rejected real catches.
DECIDED 2026-11-04  (lab chat, Wesley's request in the session to remove the dip and the stall after the release) Our fork of Te-Jen's planner (online_join_planner.py): a TAKEOFF that starts more than 0.5 m above the boot ground starts where the vehicle is and only needs a 5 cm climb to latch (TAKEOFF_FROM_AIR_M, TAKEOFF_FROM_AIR_CLIMB_M). NOT behind a switch: it changes every air start. Simulation R3634, R3635 (no dip, no swing back); real world nr9m_4, nr9m_5 (his takeoff latched in 0.6-0.9 s, against nr9m_2 stuck 25 cm low for 38 s). Kept as the default because it clearly beat the old start; Wesley can ask for a switch.
DECIDED 2026-11-04  (Wesley ~09:45, "go around the others") `partner_release_around` (default off) and `partner_release_around_r` (1.2 m): on a release with `reattach_plate`, our tracker flies QUAD4 out to r and round the outside, the shorter way at the release height, to the new plate's line, then hands it over. Simulation R3633 (route OK), R3634 (with the takeoff from the air, re-welded on plate 1 first try). Not flown on the rig (configs/rig/attach_m1_1m_m2a_around.yaml ready).
DECIDED 2026-11-04  (Wesley, the session) Typed-only knobs added for the rig: Te-Jen's pickup proof (`pickup_proof_drift` / `_speed` / `_loss`, flown 0.05 / 0.08 / 0.08 from k1m_2) and his descent (`descent_abort_speed`, `descent_hold`, `descent_gross_xy`). His defaults are unchanged.
OPEN    2026-11-04  For Wesley, from 4 Nov's logs (registry RIG-1104-*): (1) the 60 mm contact radius typed this morning declares contacts off the M2A plates (27.5 mm radius): nr9m_4/_5 contacts at 24-59 mm, each lost within 0.2-2.3 s. Proposal: plate radius + magnet face radius, from Wesley's caliper and catch-window measurement. (2) His drop never starts with the object on a 1 m cable: over the ring its magnet sat 0.23-0.26 m (median) from the centre, so his arrival check (0.18 m, held 0.5 s) passed at most once (k1m_2/_3/_4). Proposal: a typed `drop_on_arrival` that releases as soon as the magnet is inside a typed radius over the basket at the transit height, skipping the settle and descent (the supervisor accepts the drop from that height). (3) An operator OFF cannot reach QUAD4's coil: our magnet_gate holds his OFF in TRANSIT_TO_DROP_POINT and ignores ours before our span (k1m_3, k1m_4). Proposal: a typed operator topic the gate always passes. (4) k1m_3: LAND's take-back lifted QUAD4 with the object still on its magnet and down at the ring's height inside it, and the ring tilted to 30.4 deg: the automatic drop. Proposal: when the object is held and the magnet is inside the ring, the take-back drops the object first. All four behind switches, default off, simulation before the rig.
BUILT   2026-11-04  (lab chat, behind switches, default off; for the OPEN 4 Nov line) (3) `magnet_operator_topic` on the attach launch: magnet_gate passes an OFF there in any phase and holds every ON until an operator ON (tests: test_magnet_gate.py, test_real_mode_launches.py). (2) `drop_on_arrival` / `drop_on_arrival_xy` in our fork of his planner (tests: test_drop_on_arrival.py, test_partner_mission_rig.py); simulation k1_5_drop_on_arrival. Neither flown on the rig; both typed only on Thursday's sheet if Wesley wants them.
CORRECTION 2026-11-04  (Te-Jen's review of the bags; checked) The OPEN 4 Nov line's item (4) said his transit reference sank in k1m_3: it did not (2.43 m held); QUAD4 sank at his OCP's 0.60 throttle bound with the object on board (17 s at the cap; RIG-1104-k1m_3c). Built (default his 0.60): `mpc_throttle_max` (his launch) -> `throttle_state_max` set at run time on his OCP (tejen_mpc/throttle_bound.py, no rebuild), and `lift_full` (his loaded lift ends at its full height: up, then over; Wesley's supervisor). Simulation k1_5_lift_full.
DECIDED 2026-11-04  (Wesley ~16:10) The pickup line may fly QUAD4 up to ~2.8 m (his supervisor: drop the object 0.5-0.6 m above the net; the 2.6 m ceiling of 29 Oct stays for every other line). The net sits about level with the ring's bottom and is shallow. The supervisor's smaller object is 54 mm across (thickness, mass and contact height to measure). The square detach and re-attach (sheet section 6, sq9; simulation R3677) is to be tried on Thursday.
BUILT   2026-11-04  (for Wesley's ~16:10 yes) `drop_height` and `max_body_z` on his launch (typed only; tests in test_partner_mission_rig.py). Thursday's k1d types `drop_height:=0.48 max_body_z:=2.8`: on the M2A ring QUAD4 plans 2.58 m for the lift and the travel and 2.78 m for the bounce after the release (desk check 16:42); 0.50 m plans 2.80+ and is refused at 2.8. Simulation k1_5_drop048.
DECIDED 2026-11-04  (Wesley ~17:25, the lab) Measured: QUAD4's magnet face 20 mm across, the M2A plates 55 mm, the net's lowest point 5 cm below the ring's bottom when loaded, the small object 54 mm across and 22 mm thick (mass in the repo: Te-Jen's commissioning puck 0.060 kg, if the same object; to confirm). Body 6 defined for it; body 24 (QUAD4's magnet) redefined, so 08:44's W/X/Y and the typed L are void: re-measured before Thursday's attach lines. The contact radius stays 0.025 (the magnet is wholly on a plate within 17.5 mm, touches its edge at 37.5 mm; held catches 4-19 mm). With drop_height 0.48 and the net's 5 cm sag the object falls ~0.53 m.
DECIDED 2026-11-04  (Wesley ~17:40) The small object weighs 60 g (as Te-Jen's commissioning puck in the repo). From Wesley's photos it is a 54 mm steel disc in a printed holder about 0.1 m across with 4 markers: Thursday's k1d types pickup_object_size 0.10 (his collision box: the holder), thickness 0.022. Our take-back model of a QUAD4 holding an object stays RIG_PUCK_MASS 0.15 kg (graph_attach, not typable): it over-estimates this 0.06 kg object, so a take-back with it held climbs a little more; rare with drop on arrival.
DECIDED 2026-11-04  (Wesley ~17:50) The small object is 70 mm across overall (the 54 mm is its steel disc): pickup_object_size 0.07 on Thursday's k1d. It is his planner's payload collision box (OnlineSafetyPlanner keeps the magnet, cable and quad clear of it, margin 0.05 m), not the magnet's target, so the whole object.
DECIDED 2026-11-04  (Wesley's re-measure, ~17:50, the new body 24) QUAD4's magnet on plate 9 of the M2A ring: X -0.0000, Y 0.2293 (radius 0.2293, plate 9), W 0.0491 above the ring pivot; ring rest 0.0237; L (body to marker, the magnet hanging free) 1.034 m (median over the lifted samples 1.0340, a still 1 s window 1.0337, sd 1.3 mm). Thursday's sheet types them; 08:44's values are void.
DECIDED 2026-11-04  (Wesley's measure, ~18:00) The small object's contact height (magnet marker over the object's marker, the face flat on it, the marker 0.6 cm off its centre): 0.0378 / 0.0379 m. Thursday's k1d types PC 0.038 (with PT 0.022, PS 0.07).
DECIDED 2026-11-04  (Wesley ~21:15) Every detach that is followed by a re-attach stays ANNOUNCED (the DETACH: 3 s unload, step-out, hand-over to Te-Jen's mission); an unannounced release lands QUAD4 alone and has no hand-over, and from the square it peaks 57-61 deg in simulation. No re-attach-after-unannounced code. The unannounced release (rel1/rel2, X1's real-world partner) goes on Thursday's sheet as "if time".
BUILT   2026-11-04  (evening, for Thursday) `line_up` in our fork of his planner (typed `line_up:=true`; R3682: entered the ring's 1.2 m circle 0.09 m off plate 9's line, no stall). Our planner: the partner release's step-out target is clipped to the rejoin limit like the departed step-out (dissipative_node `_resize_and_release`; R3683 on the orbit never released because the target sat past the 1.70 m clip; a no-op at hover). The full M1 with Thursday's switches flew in simulation (R3684: release, pickup, drop, re-attach on plate 9, first time). The orbit with a held circle is NOT built (needs a hold on his TAKEOFF, a slow-down to rest and a resume; the position limit at r 0.5): off Thursday's sheet. Gate --quick passed 21:35 (2107); desk check of every Thursday line passed 21:24-21:28.
BUILT   2026-11-04  (Wesley ~21:45: "make the fix for the orbit now") `attach_pause` (default off; `attach_pause_ramp_s` 3.0) in our planner: once his mission flies QUAD4 after a release (his TAKEOFF) on a running path, the path clock's rate slews to 0 over the ramp, holds while QUAD4 is pending, and slews back to 1 after the weld (or a take-back, LAND, drop); /payload/trajectory_state reads hold_s 60 meanwhile. The shared reference code (mpc_planner reference_builder / planner_node) now scales the horizon's path times, velocities and accelerations by that rate (rate 1 is today's arithmetic bit for bit; tests test_traj_rate.py), so a pause is a real stop, not the frozen clock that coasted 162 mm in R3683. Tests: test_attach_pause.py, test_real_mode_launches (typed only). With the release-clip fix of 21:30. On the rig the orbit line also needs `contact_rig_orbit:=true` (Wesley's word, 11 Oct rule) and a 0.3 m circle.
DECIDED 2026-11-04  (Wesley ~22:20, closes OPEN 2026-11-03 "the rod in the plan") Flip it: `plan_rod_level` on in simulation (sim.yaml) with the sideways integral at 0 wherever the rod resolves on (int_k_xy 0 unless typed; SIL and the network mode keep the rod off and int_k_xy 5.26; real flights refuse the switch). Evidence: the level T 0.01-0.07 deg against 8.3-10.5 (R2965, R2972 vs the rod arms), the 8-quad chain back to 0.37-0.50 mm against 6.4 (R3472, R3473 vs R3476); the price: the release dip +5.5 to +9.0 mm on 0.86 kg (+11.1 at 1.15 kg) and the n = 8 solve p95 233-250 to 329-394 ms (card 2026-10-31_rod_in_plan sections 8-11). X3b's real-time numbers re-checked with it; every claim re-flies on the final tree after the freeze; the reviewer on the final table.
DECIDED 2026-11-04  (Wesley ~22:20, closes OPEN 2026-11-03 "weld_vel_filter_s") `weld_vel_filter_s` 0.1 s is the simulation's default (sim.yaml; the twin's weld gate only, the rig confirms by co-motion): the raw gate never welded in R3458, R3475; with the filter R3485-R3488 all welded.
CLOSED  2026-11-03 OPEN "X4's mission length": decided 4 Nov ~07:45 (26 leaves; line above).
DECIDED 2026-11-04  (Wesley ~22:20) X2-full's later-runs bar is 5 deg (the ring settles within 5 deg after each later release; frozen 31 Oct with its value left to Wesley). R3586-R3591's readings 2.66-3.10 deg pass. The value was set after the runs (simulation: allowed, Wesley 2 Nov).
DECIDED 2026-11-04  (Wesley ~22:20) The tracker's pull clamp (could bind for one rider of the T; the critic's prediction, untested) and the low-pack ring fall (R3444: the ring fell over 37 s, under the 30 deg drop rule, nothing reacted) are looked at now, not after the freeze.
BUILT   2026-11-04  (~22:35, for the four answers of ~22:20) (1) The flip: sim.yaml `plan_rod_level: true`, `weld_vel_filter_s: 0.1`; `bringup.real_mode` sets int_k_xy 0 wherever the rod resolves on unless the launch chose it (typed or params_file: `profiles.chosen_knobs`). Nine configs pinned to what they flew (rod with 5.26: rodl_u_meas_d00, um2_rodl, um_rodl, x4_relay_short_v3_xy, x8_rim_ours_rodl; no rod: s8c_base, s8c_noxy, um_base, um_noxy). (2) The tracker's pull clamp as a bound, `cable_pull_up_max` (default 0 = as before, every graph; option B of docs/analysis/tracker_clamp_2026-10-31.md; its runs 1-2 R3441/R3442 had confirmed the cause on 31 Oct). Runs 3-6 at +0.5 with same-tree baselines queued on the lab PC (lab_1104e), bars as frozen 31 Oct. (3) `ring_low_land_m` / `ring_low_land_s` (default off) in the planner: the ring that far under its height reference for that long, after the lift and before any LAND, descent or drop, lands every drone once (R3444's gap; analysis defect 4). Numbers from a replay of every rig flight (running), then a twin test. Real flights: nothing changes unless typed. Tests: test_real_mode_launches.py (3), test_cable_pull_bound.py, test_ring_low.py. Critic: waived (3 Nov, all).
DECIDED 2026-11-04  (Wesley ~22:50, reverses the flip of ~22:20; his question: "Is the method not modelling the cables as taut massless cables? Maybe the simulated cable mass is too high") The plan stays massless in simulation, as on the rig: sim.yaml `plan_rod_level` back to false (the weld filter stays). The twin's links are fixed instead ("Try the fix now"): on the rig the magnet is about 95% of magnet + cable (Wesley) and sits fixed on the plate, so its weight is ring load while attached; the twin hangs the 0.035 kg magnet (and a 0.040 kg rod) on the link's side of the ring's ball joint, which is about 2/3 of the 0.54 N outward term that leans the T (docs/analysis/why_integral_code_2026-10-31.md). Next: twin links with the magnet fixed to the plate while attached and a ~5% cable, then the T and an even formation with the integral on and off. Note: real.yaml's drone_mass 0.583 scaled the 0.55 m cable's 0.040 kg to 0.073 kg as if all cable; at 95% magnet a 1 m cable and magnet is ~0.042 kg (to weigh). The pinned configs and the coupling (rod typed -> int_k_xy 0 unless chosen) stay.
CORRECTION 2026-11-04  (~23:00, lab chat, before any world change) What I told Wesley at ~22:50 was wrong: the twin's magnet is a point mass at the plate's joint, so it acts on the ring exactly as the rig's magnet fixed to the plate does while attached; the sideways pull comes from the magnet's weight hanging on the cable while the plan books it on the drone (drone_mass), on the rig too. Only the twin's 0.040 kg rod is a twin error (about a third of the 0.54 N term). With magnet + 1 m cable ~0.040 kg (Wesley) the rig's term is ~0.23 N at 60 deg against the twin's 0.31.
DECIDED 2026-11-04  (Wesley ~23:00, after the correction) "Only lighten the twin's rod": the massless plan stays everywhere (sim and rig), and the T leans in both; reported as a limitation. Magnet + its 1 m cable about 0.040 kg (Wesley), the magnet about 95 %.
BUILT   2026-11-04  (~23:15, for the line above) The twin's links: tools/make_carry_worlds.py ROD_MASS 0.040 -> 0.002, MAGNET_MASS 0.035 -> 0.038, AIRFRAME_MASS 0.475 -> 0.510 (each drone stays 0.55 kg, so the plan's drone_mass is unchanged); every rig-twin world family regenerated (make_carry_worlds, _p115, make_attach_worlds, _L100, the layout worlds L100/p115, make_weld_variants: 80 files; all --check ok; old worlds in old_worlds/ untouched; pre-change snapshot in the session scratchpad). sim.yaml's rod values for the (off) switch: 0.040 kg, CoM 0.025 x cable_len from the ring end. The rod-in-plan configs that type the old 0.075 / 0.267 rod now fail --check against the new worlds (as they should); the clamp's runs 3-6 (lab_1104e, rod-in-plan T's) were cancelled before they ran. Every twin result before this flew the old link. Queued: lab_1104g (the T with the integral on x2 and off, the full M1, the ring-low pair).
PROPOSED 2026-11-04  (~23:20) The ring-low gate's numbers: `ring_low_land_m` 0.30, `ring_low_land_s` 3.0 (results/analysis/ring_low_gate_2026-11-04/, a replay of every rig flight and R3444). On flights from 7 Oct it fires on h3tp alone (the ring stalled 0.33-0.55 m under its reference for 11.7 s, LANDed by hand: a correct fire); worst elsewhere 0.70 s over 0.30 m (ls2c); deepest dip on a flight that held 0.186 m (r0007). On R3444 it fires at 815.8 s with the ring at 0.69 m (it reaches the floor at 848). The 30 Sep / 1 Oct commissioning flights (integral off, the ring truly low) would have fired on 8 of 17 windows. Off by default; the twin test is queued (lab_1104g); a rig default is Wesley's word.
BUILT   2026-11-04  (~23:50, finishing the new links) Five rig-twin worlds no generator makes kept the old 0.075 kg rods on the regenerated 0.510 kg drone models (each quad 0.585 kg): three/four_rigid_ground_rig_push.sdf, three_rigid_ground_rig_wind.sdf (0.55 m, Oct), three_far_ / three_plate1620_rigid_ground_rig_attach_pickup_L100_p115.sdf (30 Oct). Their 16 rod inertial blocks were checked identical to the old generator's and swapped for the regenerated ones (no other line changed); a scan finds no old rod left on a rig-twin world. Tests that pinned the old link updated (attach world digests: masses and inertias only, checked line by line; rod check 0.040 kg, CoM 0.025 x L). Gate --quick PASSED 23:50 (2117).
CORRECTION 2026-11-05  (~00:45, lab chat) My prediction for the new twin links was wrong: the T with the integral on leans MORE, 17.83 / 16.50 deg (R3699, R3700) against 8.30 / 10.52 on the old links (R2965, R2972, an older tree); with the sideways integral off it settles at 1.77 deg, 47 mm off its point (R3701; old R2998/R3202 0.58/0.60 deg, 64 mm). The 31 Oct audit had the mechanism: the plan's 0.1 N floor on a zero-share carrier and the sideways integral run away together, and the heavy rod partly offset it (R3401, 1 g rods, integral on: +19.0 deg). The full M1 holds on the new links (R3702). For the rig: the real links are the light ones, so a real T with the integral on may lean well over the twin's old ~7 deg (R3677); sq9's twin re-fly on the new links, integral on and off, is queued (lab_1105b).
PROPOSED 2026-11-05  (~01:10, replaces the 0.30 m / 3 s of ~23:20) The ring-low gate at 0.15 m for 5 s (or 0.20 / 5): on the drained-pack twin it fires with the ring still at 0.83 m (0.78), ~25 s (~10 s) before 0.30 / 3 did (R3705, R3704, R3697); the replay of every real flight since 7 Oct fires on h3tp alone at either (over 0.15 m at most 2.02 s, ch1b_2; over 0.20 m at most 1.40 s). In all three twin runs the LAND then ended with one quad tipping during the last of the descent (the pack too weak to brake): a LAND-on-a-weak-pack problem, not the gate's. Off by default; a rig default is Wesley's word (critic waived 3 Nov for all).
DECIDED 2026-11-05  (lab, ~09:00) Cause of today's lift bounces found by Wesley: a Motive recording running on the mocap PC. In-flight command-to-response delay (throttle sent -> the quad's measured vertical acceleration) 75-135 ms on every quad today against 45-65 ms on 4 Nov; every lift bounced at ~1 Hz (k1d 18 deg; fm9_2 and fm9s automatic drops at 30.6 / 30.9 deg; fm9_4 LANDed); mocap_stream_check said OK (it measures packets, not age). Recording stopped: fm9y_2 delay 45-60 ms, lift tilt max 10.2 deg, 0.24 m/s. Rule (CLAUDE.md, the sheet): never record in Motive during flights. fm9y_2 flew 4 Nov's QUAD4 values (old body 24) and its re-attach never touched the plate (3 descents): back to the 4 Nov evening measurements for every line.
DECIDED 2026-11-05  (Wesley, lab ~09:30) The ring-tilt drop at 55 deg for the square's detach and re-attach (sq9), through configs/rig/attach_m1_1m_m2a_drop55.yaml (drop_tilt_deg 55.0; the carriers' trackers read it, launch evaluation 09:31; the trackers' 60 deg max_payload_tilt_deg fleet disarm unchanged). Asked for 60; at 60 the drop and the 60 deg disarm coincide, so 55 (his choice of 45 / 55 / 60+70). Why: the square's release swing reached ~28 deg in simulation (R3708, R3709), 2 deg under the default 30.
DECIDED 2026-11-05  (Wesley, lab ~09:58) The cable-length loss check at 15 cm for the rest of today's square lines (detach_detect_m 0.15 in configs/rig/attach_m1_1m_m2a_drop55_det15.yaml; common.yaml keeps 0.06). Why: sq8_2 was the first real re-attach after a detach on the square (CONTACT try 1 xy 17 mm normal +0.4 mm with contact_dz -0.03, proven, re-welded 14.4 s after the release), then 1 s into LAND the check read QUAD1's cable 1.098 m (9.8 cm over) and dropped the ring; QUAD1's measured pull stayed steady (-2.7..-3.2 m/s^2) and the mocap reconstruction has every cable 1.01-1.03 m up to the drop: a false loss during the 0.18 m/s LAND descent (a drone pose ~0.5 s older than the ring's reads 8-9 cm long). Also seen: the real T (11/2/5) leaned 21-28 deg while QUAD4 was away (the twin: ~8.5), so the 55 deg drop is what let the re-attach happen. To look at after today: the check's pose ages during LAND.
FINDING 2026-11-05  (lab day, registry RIG-1105-*) (1) k1d: QUAD4 alone picked up the small object and dropped it into the flying ring first try (drop on arrival 8.5 cm from the drop point). (2) sq8_2: the first real detach and re-attach (square on plates 11/2/5, QUAD4 on 8): contact try 1 at 17 mm and +0.4 mm with his descent aimed 3 cm below the plate (contact_dz -0.03; at the rig default -0.01 nr1r_3's magnet stopped 1.7-2.3 cm above plate 1), proven, folded in 14.4 s after the release. The real T leaned 21-28 deg while QUAD4 was away (the twin ~8.5): the default 30 deg drop would have fired. (3) Re-attach on the triangle (nr9o_2 on the paused orbit, nr9m in hover): the descents reached plate height 3.5-10 cm off the plate centre (the catch needs 2.5 cm); no contact in ~30-60 s. (4) The orbit pause worked on the rig (nr9o_2): the circle stopped at his TAKEOFF, the ring held at 0.01-0.07 m/s, the circle resumed at the end. (5) k1ob_2: the bigger object (contact height 0.0181 m with the new body 24, 100 x 10 mm) dropped on arrival into the moving ring, in the net 4 cm from the centre; 0.6 s later the ring jerked 7 -> 54 deg in 0.2 s (93 deg by +0.8 s): the drop at 60.1 deg. Not explained (the impact in the net?). (6) Pickup failures k1o_2: Motive's body 6 re-enabled with its centre several cm off (tip-object 7.3 cm median against 1.7 in k1d).
OPEN    2026-11-05  For Wesley, from the lab day: (a) BUG: QUAD4's own tracker kept the trackers' 60 deg ring-tilt envelope while his stream flew QUAD4 off the ring and DISARMED QUAD4 at 80.3 deg (k1ob_2): QUAD4 fell ~2.5 m. Proposal: QUAD4's tracker skips the payload checks while it is not on the ring (a gate change: critic or waiver, your word). (b) His TAKEOFF before PARTNER RELEASE on the round-the-outside route sent QUAD4 1.3 m across the ring (nr1r): proposal: hold his TAKEOFF at the rig until the release (the twin's start gate). (c) The 6 cm cable-length loss check fired falsely 1 s into LAND (sq8_2: pose ages at a 0.18 m/s descent); 15 cm typed for the rest of the day: look at the pose ages before any default change. (d) Re-attach accuracy on the triangle (3.5-10 cm off): his match_xy 0.08 starts descents loosely aligned; Wesley's concern: tighter may stop attempts. (e) The ring jerk after the bigger object's drop (k1ob_2).  -> Wesley ~11:15: "1) Yes. 2) Ok. 3) Sure. 4) Yes" (+ he may 3D print a funnel for one plate). (a), (b) BUILT, (c) investigated and BUILT behind a switch (BUILT line below); (d) open (the funnel is his hardware option); (e) open. -> (e) EXPLAINED 5 Nov ~18:34 (RIG-1105-k1ob_2c): the bigger object's impact in the net pulled the plate 7 and 3 magnets off, then plate 11's; not a control or mocap fault. (d) stays open (the funnel, match_dwell).
DECIDED 2026-11-05  (Wesley ~11:05, low disk: 596 MB free) Deleted: the Gazebo replay recordings (gz_record) of every simulation run (315 folders, ~10.4 GB; no scorer, table or figure reads them; run.csv, logs, metrics and plots kept), ~/mdc_onedrive_upload's .tar.zst archives (10 GB; every original still in results/, checked; MANIFEST.txt kept), the two 4 Nov zips for Te-Jen (0.3 GB), caches. 22 GB free after. Simulation runs can no longer be replayed in Gazebo for a video; a new run records one again.
BUILT   2026-11-05  (after the lab day, ~11:15-11:45; card docs/experiments/2026-11-05_rig_fixes_quad4_takeoff_skew.md, critic pending; uncommitted) (a) QUAD4's tracker reads /drone_3/on_load (graph_attach, the newcomer only, with newcomer_fault_land): while off the ring its ring-tilt envelope and payload checks stand down and its drop rule does not run; never heard = as before. (b) His TAKEOFF before our release is held by his planner (our fork: takeoff_gate_topic, typed /partner/release by our combined rig launch when partner_attached; his launch passes it only when typed) and goes at the release from where QUAD4 is; while held his reference is QUAD4's own pose (his MPC gets the same TAKEOFF and flies; our mux drops its stream until the release); the console's 2 s TAKEOFF alarm waits for the release while QUAD4 is ours. (c) sq8_2's false loss: the planner's carrier poses ran ~0.6 s behind the bag in the lift and from the end of the hover through LAND (ring and QUAD4 ~0.15 s; every other flight that day 60-150 ms on all five bodies; cause of the extra 0.5 s not found: one mocap node, the bag on time, depth-5 subscriptions). Built: the planner keeps each pose's mocap stamp; detect_skew_s (default 0 = off; proposal 0.05 typed on the rig, Wesley's word) skips a drone whose pose and the ring's are stamped further apart; always on: a log line when a slot's pose is >0.25 s behind the ring's. Tests: test_off_ring_checks (4), test_takeoff_gate (4), test_detect_skew (5), launch and console tests (4). Gate --quick PASSED ~11:35 with (a)+(b), and again ~11:43 with all three (2130 passed).
BUILT   2026-11-05  (~11:45-11:55, after the critic of card 2026-11-05_rig_fixes_quad4_takeoff_skew: "rewrite the card") (a) as first built did nothing on welded starts (the planner publishes on_load only with welded_on_load, off): QUAD4's tracker now reads its mux (/drone_3/mux_state 'partner' = his or the approach stream flies it: ring checks down; 'ours'/'cut'/'latched'/unheard: as before), welded and unwelded starts alike; welded_on_load untouched. (b) a TAKEOFF refused in the air (e.g. no fresh ring pose at the release) no longer DISARMs QUAD4 (his MPC may already fly it through our mux): it holds where it was and is tried again at each release message; on the mat as his. (c) a skewed tick keeps its count (a flickering gap cannot hold a release off); log.csv gets skew0..skewN (ring stamp minus each slot's) to judge 0.05 against normal jitter. His MPC in RIG-1105-nr1r (TAKEOFF 16 s before the release): kT estimate 22.0-22.5 while his commands were dropped, 22.19 at the hand-over.
FINDING 2026-11-05  (~12:30, twin checks of the rig fixes, R3712-R3715) The fixes behave as built: QUAD4's tracker stands its ring checks down at the mux's release and back at the re-weld (R3712, R3714) or the take-back (R3713); the pickup on the orbit matches R3711. But the square's T stage on today's tree leans more than R3708 (5 Nov 01:30, same config and node parameters): T median 11.2-12.0 deg (R3712, R3714, R3715) against 8.5, the fold-in max 14.8-15.8 against 6.8; the runs part ~4 s after the resize, where today's planner integral keeps growing and R3708's held. Not the cpus (4 and 12 agree), not the tracker change (R3715 with it off leans the same); cause not found. For the sheet: the twin's T lean on the square is 8.5-12 deg (the rig's sq8_2: 21-28).
DECIDED 2026-11-05  (Wesley ~12:55: "Skip the loss check thing, I dont want it") No timestamp skip in the loss check: detect_skew_s and its skip logic removed from dissipative_node._detect_tick, test_detect_skew.py deleted (record kept: card 2026-11-05_rig_fixes_quad4_takeoff_skew F3, its critic points 5-6). Kept, diagnostic only: the planner's per-slot mocap stamp gap in log.csv (skew0..N) and its late-pose warning (>0.25 s). The rig's loss-check margin stays as typed per line (15 cm in the _det15 params files of 5 Nov; common.yaml 0.06).
BUILT   2026-11-05  (~12:55-13:15, an agent; card docs/experiments/2026-11-05_quad4_foldin_weld_cap.md; critic running; uncommitted) QUAD4's fold-in weld-height cap (handin_plan l.25, Wesley 30 Oct): `foldin_cap` ('' off by default; common.yaml, real_mode PLANNER_OPTIONAL), reusing _rejoin_cap: 'weld' = the rejoin's cap exactly (no node above a level ring's height and never above the weld's), 'level' = the level-ring limit only. Finding from the logs: only R2829 has the rejoin's E2 problem (QUAD4's plan followed its body up 3.9 cm, ring 9.9 deg); the square re-attach peaks R3712/R3714/R3715 (14.8-15.8 deg) are the T's own lean at the weld (plate 9 5.7-6.1 cm low; the fold-in adds 0.1 deg and levels the ring in 3.3-3.8 s), where 'weld' would hold plate 9 low until +10 s, so the agent recommends 'level'. Tests test_foldin_cap.py (12) + a launch test; gate --quick PASSED (2140). Configs S1 (R2829's chain), S2 (the square re-attach), S3 (the triangle re-attach), S4 (orbit) x off/weld/level, validated; runs after the critic and the landing batch.
DECIDED 2026-11-05  (Wesley ~14:00, on the weekend claim batch docs/claim_batch_2026-11-07.md) (1) The 4.3 joins that lift a T (sq_floorjoin x2, sq_pickup) fly int_k_xy 0, as every T cell (the 31 Oct fallback applies: rod-in-plan reversed 4 Nov). (2) The gap table re-flies on the final tree with the integral as flown (its 180 deg rows tip within ~1 s, before the integral acts): chunk 9 on, 255 runs. (3) No more 1.15 kg cells: the text says which tables have one (X1 h4p115, X4's relay). Taken on the recommendation (the main chat; Wesley may override): X8's hub as flown; the switch-of-solvers count from X1's resized runs; the methods' delay and filter runs cited as flown; X4 int_k_xy 0 without the rod as prepared (one short relay in the pre-freeze smoke); X3b's extras R3686-R3693 not re-flown; X2's unwritten hover and spinny-line cells dropped.
BUILT   2026-11-05  (~12:45-13:55, an agent; card docs/experiments/2026-10-17_land_ref_below_rest.md Rev 4 + results; critic running; uncommitted) The landing's slow final descent: `land_slow_vel` (0 = off) and `land_slow_z` (0.40 m) under land_descend_to_stall; the stall test judges drones against the slowest commanded speed (the trap: it fired in the air on a slowed descent); the max-drop clock runs at the commanded speed while slowed; the launch refuses a slow speed above land_vel. Tests test_land_slow.py (11), gate PASSED (2127). Twin R3716-R3733 (A today / B land_descend_to_stall / C + slow 0.10 below 0.40): on the unwind path C lands at 0.10 m/s, every drone at rest, stall exit; on the rig LAND path every arm meets the floor at 0.25-0.45 m/s (read after the runs: below 0.25 m the xy follows the drone and the twin's rigid rod drops it the last ~15 cm), so C misses the 0.13 m/s contact bar there; with a carrier 0.03 stronger than its map (S4) A stops it 26.6 cm up, disarms it in the air and it tips 90 deg (the 5 Nov rig hover reproduced), B and C land it. Proposal (Wesley's word, after the critic): fly C on the rig through a configs/rig params file, first a plain three-carrier hover LAND with frozen bars. Also: the max-drop backstop fires ~12.4 s after LAND from a 1 m hover against today's stall at 11.0-11.9 s: thin on 1 m links.
BUILT   2026-11-05  (~13:20-14:00, an agent; uncommitted) Committed-tool scorers for every claim (tools/x1, x2, x3, x3b, x4, x8, rejoin, s41 _metrics.py; ids as arguments), each reproducing the registered numbers on the old runs (docs/claim_batch_2026-11-07.md section 10; tests tools/test/test_claim_scorers.py). Reviewer notes: X3's bars never frozen (four definitions matched to the registered numbers); X3b's registered fold-in figure is the tilt rise, not the J bar's peak (both pass 5 deg); chain4task /drone_5 20.4 mm against a 2 cm draft bar.
DECIDED 2026-11-05  (Wesley ~14:40, the rig sheets for S3 Wed 11 / S4 Thu 12 Nov: docs/rig/wed_2026-11-11.md, thu_2026-11-12.md, scripts results/rig/2026-11-1{1,2}/scripts/, desk-checked offline 14:19-14:27) (1) The 1 m baselines and the release pair fly the 0.86 kg M2A ring on 1 m cables (the text says which ring). (2) The real-world re-attach claim (Nr x3 in hover) flies the square (carriers 11/2/5, QUAD4 on 8, as sq8_2). (3) The pickup and drop (K x2, the small object) flies the ring at 1.2 m with drop_height 0.28 and max_body_z 2.8 (k1z; QUAD4 planned 2.56/2.56/2.76 m, desk 14:45; the object falls ~0.33 m, under the supervisor's 0.5-0.6). (4) Nr on a path flies the orbit with contact_rig_orbit:=true typed (the figure-8 stays refused under partner_contact). Earlier today (~12:56): the small object for the pickups.
BUILT   2026-11-05  (~14:20-15:05, an agent; card docs/experiments/2026-11-05_carriers_down_alone.md rev 2 after its critic; uncommitted) `carriers_down_alone` (off by default; refused without land_descend_to_stall): while a LAND waits for a departed drone (QUAD4 taken back), each carrier is flagged /departed once the touchdown came by the stall, the ring is down, and the ring and every carrier stayed <= 0.05 m/s for 0.5 s, and never while QUAD4's take-back is in its hold/guard/guard_follow; the tracker disarms it inside its departed band, the fleet manager alone (its LAND text now 'on the landed ring, or off it'). The claim is 'carriers disarmed sooner', not 'drift fixed': in every 5 Nov wait the carriers were in the air, which land_descend_to_stall (B) fixes; F-rig: a B rig wait with the armed-idle carriers within 5 cm and no tip means the switch adds nothing (delete the code). Gate PASSED (2191). 14 twin configs cda_{a,b,d}_* queued.
DECIDED 2026-11-05  (Wesley ~15:05) (1) land_descend_to_stall becomes the RIG default ("just make it the default, automatically tested when I do the first run"): no separate barred flight; the twin stays off for now (the weekend claim batch's landings unchanged; Wesley may extend it). Evidence: R3745 (B with land_rearm_touchdown off, as the rig) lands every carrier at its rest, stall exit, no false touchdown; rev 3 R2174-R2200. Said once: the LAND that waits for a taken-back QUAD4 has not flown under it (the reference frozen below the floor up to 15 s; ground idle latches within 3 cm of the spawn height). (2) The slow final descent (land_slow_vel/_z) is deleted: it doubled the time below 0.25 m on the rig path (recorded: card rev 4, R3716-R3733, R3746).
DECIDED 2026-11-05  (Wesley ~15:15) The landing combination for the rig: land_descend_to_stall (the rig default, ~15:05) plus carriers_down_alone with a 15 cm disarm height for the carriers ("now that we get the quads to go lower maybe we can make the disarm height 15cm"): once the ring is down, a carrier still for 0.5 s within -3..+15 cm of its own rest is disarmed alone instead of waiting armed for a taken-back QUAD4 (a drop of at most 15 cm, inside the 9-16 cm accepted at LANDED on 10 Oct); QUAD4's own band unchanged. Being built (card 2026-11-05_carriers_down_alone rev 3); the twin runs (incl. a carrier 0.03 stronger than its map) first; if clean, the rig default alongside land_descend_to_stall.
BUILT   2026-11-05  (~15:15-15:25, an agent; card 2026-11-05_carriers_down_alone rev 3; uncommitted; not a default yet) Wesley's landing combination: carriers_down_alone with carriers_down_band_m 0.15 (default with the switch; 0.03-0.20 accepted): a carrier released by the planner (its own latched topic /drone_d/carrier_released, so the tracker tells it from QUAD4's /departed) is idled and disarmed alone within -0.03..+0.15 m of its rest, still (whole speed <= 0.05 m/s) with its reference at the floor for 0.2 s; QUAD4's departed band unchanged (compared tick by tick with the committed code); the manager says 'on the landed ring (still on its plate): disarmed alone'. On the rig it is valid alone (land_descend_to_stall is the rig default). Gate PASSED (2201). 10 twin runs cda_{b,d}_* chained on the lab PC after the running batches.
DECIDED 2026-11-05  (Wesley ~15:57) QUAD4's fold-in cap: keep `foldin_cap` 'level' OFF by default (an exploratory finding: in R2829's case, QUAD4 re-welding onto a settled pair, it cut the fold-in tilt kick 5.5 -> 2.6 deg, R3743/R3748); delete the 'weld' mode (R3747 3.1 deg, misses; R3754 slowed the square re-attach's settle 5 -> 14 s). Neither changes the 4.3 claim join (fri30_sq6b R3751-R3753). No default change. Found on the way: sq_floorjoin_hover_L100_p115_noxy (a weekend claim cell) never welds on today's tree: his C++ transfer stalls in TRANSIT_TO_REATTACH (R3738, R3749, R3750; as R3706): under investigation before the weekend batch.
DECIDED 2026-11-05  (Wesley ~16:00) The weekend's 4.3 floor joins stalled (R3738, R3749, R3750) because with int_k_xy 0 the T hovers 0.05 m off the planner's hover point and Te-Jen's C++ backend latches itself off on its carrier-tracking check (+-0.05 m, tejen_dynamic_planner config c1f6_moving_rendezvous.yaml; COOPERATIVE_SHARED_INPUT_INVALID_DISABLED ... tracking_mismatch at ~50-54 s). Wesley: "That is way too tight by tejens planner, extend the +-5cm to +-20cm": the bound becomes 0.20 m in his package (our fork; recorded in CHANGES_FROM_TEJEN.md), tracking_check stays on. And the plain square floor-join (it flew the twin's old over route at 3.25 m) flies the LOW route, as his 29 Oct rule (QUAD4 never over the carriers). Verify runs of the three join cells before the batch.
DECIDED 2026-11-05  (Wesley ~16:15) Te-Jen's C++ route planner: the carrier-tracking check widened to 0.20 m (new parameters cooperative_tracking_check_half_{x,y,z}_m in tejen_dynamic_planner c1f6_moving_rendezvous.yaml; the default -1 = the margin, so nothing else changes), the collision margin (cooperative_tracking_half_*) kept at 0.05 m: at 0.20 m the low route's end point (the tip 0.30 m over plate 9) falls inside the carriers' cable boxes (it clears them by 0.018 m at 0.05; R2846 stalled when blocked). Critic waived by Wesley (the verify runs test it). The plain floor-join cell flies the low route (sq_floorjoin_hover_L100_p115_noxy_low: QUAD4's planned body 2.33/2.38 m, was 3.20/3.25). C++: the lab must rebuild (no --no-build). Recorded in CHANGES_FROM_TEJEN.md.
DECIDED 2026-11-05  (Wesley ~16:25, after R3755-R3764) (1) carriers_down_alone (15 cm band) becomes the RIG default alongside land_descend_to_stall (H1-H4 hold: where the LAND waited for QUAD4 every carrier was disarmed alone 0.60-0.66 s after the touchdown instead of 5.8-7.9 s later at LANDED, within 0.2 cm of rest; nothing where nothing waits). (2) land_rearm_touchdown becomes the RIG default too: with the rig's LAND settings (descend_to_stall on, re-arm off) a carrier 0.03 stronger than its map gave a false touchdown 1 s after ring-down with the carriers ~0.8 m up (partly the twin's rigid rods) and LANDED disarmed them in the air (R3757, R3758, both arms); the re-arm (twin default since 17 Oct; its rig-path must-fixes met) fixes it in the twin. An agent sets both in real.yaml with launch tests and the re-arm's two should-fixes; twin check cda_{b,d}_k1s_rearm queued after the join verify runs.
DECIDED 2026-11-05  (Wesley ~17:40) The 4.3 pickup-then-re-attach cell (sq_pickup_hover_L100_p115_cppreattach_bd_noxy) flies OVER the carriers (transit:=over, QUAD4 ~3.25 m, as R2863-R2865 flew): an exception to his 29 Oct rule for this one simulated cell. On the low route his C++ planner carries the object to 0.70 m short of the drop point and stops (105 of 127 failed plan checks: the plate-3 carrier's tether against QUAD4's cable; R3767 today, R2904 on 30 Oct). Config ..._noxy_over.yaml, a verify run tonight. Also found (an agent, ~17:35): R3765's abort at LAND is the known twin artefact (QUAD4's own tracker: tilt 84 deg > 60 during the post-touchdown unwind; its rigid universal-joint arm on a grounded ring; 2 of 14 such landings, R2864 too; decisions 9-10 Oct: the landing is scored apart): its join passed. BUILT ~17:35 (an agent): the rig LAND defaults land_rearm_touchdown + carriers_down_alone in real.yaml (with land_descend_to_stall; a params_file turning land_descend_to_stall off must turn carriers_down_alone off too, else refused), the re-arm's two should-fixes (logging only: stale-pose tick counts on the touchdown line, a Motive dropout warning), launch tests; gate PASSED (2200). Motive repeats a lost body's last pose bit for bit, so the 0.2 s freshness check catches only a stopped stream (recorded in the card; no behaviour change).
DECIDED 2026-11-05  (Wesley ~18:00) No frozen bars on the rig either: "it makes no sense to freeze arbitrary bars unless I am comparing my method/results to another paper". Rig results (the re-attach on the square, the pickup) are reported as measured; only a comparison with another paper's numbers fixes its bars first. CLAUDE.md updated; the sheets' draft bars (N1-N3, K1-K3) are notes, not gates.
FINDING 2026-11-05  (an agent, ~18:30; scratchpad k1ob/) RIG-1105-k1ob_2's ring jerk: the bigger object (~0.15 kg) fell 0.53 m into the net and hit 14 cm off centre at ~2.8 m/s: the plate 7 and plate 3 magnets were pulled off (~16 N on plate 7 against their ~10 N hold), the ring swung down from plate 11 and that magnet peeled off too; our drop at 60 deg came after. k1d's 60 g object (same fall, 11 cm off centre) held. For the rig: never drop the bigger object into the net with these magnets; the small object needs no change (Thu 12 Nov's k1z, ring at 1.2 m, drop_height 0.28: about k1d's impulse or less).
FINDING 2026-11-05  (~18:37, planner log vs bag, scratchpad pose_lag_day.py) The planner's late carrier poses recur: of 19 rig flights on 4-5 Nov, nr9m_4 (4 Nov: one carrier, ~680 ms) and sq8_2 (5 Nov: all three carriers, 610-670 ms; the false loss in LAND) had a carrier's poses 0.6-0.7 s behind the bag, while the ring and QUAD4 stayed ~0.15 s; every other flight had every body within 60-150 ms. The bag (another subscriber) had them on time; the planner's subscriptions are depth-5 RELIABLE. Cause not found (a candidate: a RELIABLE reader blocked on a lost sample's repair). OPEN for after the claim batch's freeze: look at the planner's pose QoS (best effort for mocap) and its receive path; meanwhile the loss check stays at 15 cm on the rig lines.
DECIDED 2026-11-05  (Wesley ~19:40) (1) The ring-low landing rule (ring_low_land_m/_s, 0.15 m / 5 s proposed 4 Nov) stays OFF on the rig. (2) The planner's late-pose fix (FINDING 5 Nov ~18:37: the pose QoS / receive path) is left until after hand-in; the rig keeps the 15 cm loss margin. (3) The funnel version (v1/v2) on 6 Nov. (4) "You may as well freeze the code now if its ready, we dont need to be so strict on the freeze, lets just get the results": commit the tree the claim batch flies (no push).
DECIDED 2026-11-05  (Wesley ~21:48: "Ok skip match_dwell") No match_dwell 1.0 try on the rig: his MATCH dwell stays 0.5 s (typed since 29 Oct). Why (the lab chat's recommendation after RIG-1105-joins): QUAD4 was already still before its descents (0.05 m/s over the last second), a 1 m pendulum (~2 s period) does not settle in another half second, and the misses came from the 8 cm match window and the magnet's swing adding up, which the funnel addresses; one change on Thursday's triangle try (nrtf). Thursday's nrt line and both copies of rig_nrt.sh removed (Wednesday's at 22:56, found by a review); desk check 16 OK (22:56). Closes the match_dwell half of OPEN 2026-11-05 (d); the funnel half stays open (Wesley, Fri 6 Nov).
DECIDED 2026-11-05  (Wesley ~21:55, X4 on the frozen tree) X4 is reported as flown: R3772/R3773 (x4_relay_30min_v3_norod) did 26/26 swaps with no fault, mission 2109-2118 s (R1 now holds), R2-R6 hold, but R7 MISSES: every pack flew 695-707 s against B 600 s. Cause (measured): the rejoin takes 35 s median against 18 s on 3 Nov (R3519/R3564): the twin's hardware links of 4 Nov 23:15 put the magnet's 0.038 kg at the cable end (~3x the swing inertia of the old 0.040 kg rod, same joint damping), so the end settles on its plate more slowly in the end hold (the short relay shows it too: R3741 vs R3483). The miss stays recorded with its cause; no swing-damping or weld-gate change, no re-fly. The thesis says a 35 min mission with 26 swaps on ~12 min packs (10 min packs would need faster swaps or more quads). Rejoin times in the other chunks (X2, X3, X11 rejoin) will show the same slower rejoin.
CLOSED  2026-11-05  (the lab chat, ~23:00, after a read-only audit of decisions against the defaults; nothing new decided here, each OPEN line was settled by a later line that never closed it) 22 Oct OPEN "the carriers' mocap velocity filter slope" -> DECIDED 28 Oct (slope on every carrier, both sides; CLAUDE.md). 29 Oct OPEN "type the debounce on Tejen's descent restart" -> typed on every attach script since (descent_debounce 1.0, descent_abort_xy 0.10). 29 Oct OPEN "the 2-quad baselines B2 on 0.86 kg, or skip" -> DECIDED 30 Oct 13:10 (the 28 Oct pair flights). 30 Oct OPEN "the attach's gates after the descent" -> the contact radius loosened 4 Nov, K 4 Nov, tip_skip_repeats / comotion_speed_window_s 0.5 / contact_give_up_s 120 typed on every attach script. 30 Oct OPEN "h3tw, plan_w_pos_scale 2.0" -> moot: never flown, not on any later sheet (corner cutting closed 18 Oct). 30 Oct OPEN "the 4-quad T1 streams 9 bodies" -> the Wed 11 four-quad T1 streams 7 (rig_t1_q4.sh). 30 Oct OPEN "does the heave go with the delay gone?" -> FINDING 5 Nov: the heave was a Motive recording (fm9y_2 clean once stopped). 30 Oct ~14:00 OPEN "the 1.15 kg ring's attach_z" -> closed by the 30 Oct line that says so. 4 Nov OPEN "four proposals from 4 Nov's logs" -> (1) DECIDED 4 Nov (contact radius stays 0.025), (2)(3) BUILT 4 Nov, (4) CORRECTION 4 Nov. 4-5 Nov PROPOSED "the ring-low gate's numbers" -> DECIDED 5 Nov: stays OFF on the rig (Wesley); the twin default stays off (no decision to adopt it).
OPEN    2026-11-06  For Wesley (morning), from the 5 Nov night reviews (docs/reports/reviews_2026-11-05/; lab chat's recommendation first): (1) QUAD4's tracker gets the carriers' drop rule (card docs/experiments/2026-11-06_quad4_tracker_drop_rule.md; a gate change: critic or waiver; build after the last claim chunk starts, before Wed 11). (2) The thesis headline: the real-world attach and detach first with their denominators (1 re-attach caught of 43 descents, 1 attach from the mat, 6 of 15 pickups proven), composition at scale second, X1 as a mechanism result (its B1/B2 falsified). (3) Wednesday's order: one plain four-carrier hover LAND before the claim's first flight, and rel right after the re-hang. (4) The 0.86 kg ring on 1 m cables named "Setup 3" in the thesis. (5) The catch radius stated as flown (rig confirmation 25 mm; the twin's weld 10 mm), not 45 mm. (6) The funnel, v1 (recommended) or v2. Also for the record, no rush: the 22 dead switches (review C, section 3a) deleted after hand-in; WP7 (fold our planner into mpc_planner, 14 Oct) withdrawn; z_i_max 0.3 recorded as flown since 4 Oct (decisions.md said 0.15); the rel line's tip_detect_m 0.04 on the rig (no line approves it).
DECIDED 2026-11-06  (Wesley, the morning of 6 Nov, the funnel; closes the funnel half of OPEN 2026-11-05 (d) and item (6) of OPEN 2026-11-06) He prints two: v2 (mouth 174 x 144 mm, 45 deg wall, 75 mm tall, 1.6 mm across the wall, a 15 mm round where the cone meets the base, a 2 mm lead-in, four ribs; 93 g in PETG) and v3 (v1's mouth 214 x 164 mm at 45 deg, 97 mm tall, reinforced less for its weight: 1.2 mm wall, the same rounds, two radial ribs; 97 g). cad/funnel/funnel.py VARIANTS; files in ~/Downloads/funnel/. By geometry on the 24 plate-9 descents of 4-5 Nov (results/analysis/rig_joins_2026-11-04_05/funnel_catch.md): into the mouth / disc fully inside v2 20 / 11, v3 22 / 15. The funnel's weight sits on plate 9 (carriers 7 and 11 carry ~half each with QUAD4 away); no ring-mass change (Wesley 5 Nov).
CORRECTION 2026-11-06  (the X4 reviewer, ~10:00; registry R3772c2) Of DECIDED 2026-11-05 ~21:55 "X4 on the frozen tree": its numbers and verdicts stand, but (1) its "R2-R6 hold" is R2, R3, R5, R6 hold and R4 not confirmed (R3772c); (2) the stated mechanism is wrong: the old twin link already carried a 0.035 kg magnet at the ring end, and the swing inertia about the pivot fell (0.048 -> 0.039 kg m2), not tripled; the slower return is the link change of 4 Nov ~23:15 by elimination only (the return code, the weld filter and int_k_xy did not change), mechanism not established; (3) the worse 7-carrier laps and release tilts follow the massless plan (Wesley 4 Nov ~23:00), not the slower return; (4) the relays fly the 0.9 kg ring. Wesley's call (report as flown) is unaffected.
FINDING 2026-11-06  (~10:24, exploratory A/B R3907/R3908; feed F058) The slower returns of the final simulation (X3, X4; 35-37 s against 17 s) come from the twin's model change of 4 Nov ~23:15 (cable 0.075 -> 0.040 kg, airframe 0.475 -> 0.510, each drone 0.55 kg), not from the 4-5 Nov code: on the frozen code the 4 Nov models return in 16.6 s, the final ones in 37.0 s. Mechanism not identified. Corrects the cause lines of CORRECTION 2026-11-06 (X4 reviewer: "by elimination") and R3779c ("not established"). For Wesley: report as flown (recommended; the final models are the hardware's), or look for a faster end hold (a change to the return, a critic, and a re-fly of every return claim before the simulation freeze).
DECIDED 2026-11-06  (Wesley ~10:30, after the A/B R3907/R3908: "use the old sim hardware code", then "only claims with returns, no explanation needed") The simulation claims WITH RETURNS re-fly on the quadrotor masses of before 4 Nov (cable 0.040 + magnet 0.035 kg on a 0.475 kg airframe; each drone 0.55 kg): X4 (5 runs), X3 with its bases (32), X3b's two chains (4), X2 (6), the rejoin cells (8): 55 runs, configs/experiments_oldlinks/ (the same names, world -> *_oldlinks.sdf), batches configs/batches/oldlinks1106_*.txt. The worlds and drone models come from the same generator with the old masses (tools/make_oldlink_worlds.py; byte-identical to 9f20f8b's for the worlds that existed then). Every other claim (X1, X8, 4.1/4.2, the joins, U, Quan, the gap) stays on the final models. The new-model runs of X3/X3b/X4 (R3772-R3820) stay in the registry as the recorded comparison. The thesis does not explain the two mass splits (Wesley).
DECIDED 2026-11-06  (Wesley ~11:35, after X1's reviewer: the square-to-T cell settled 0.6 vs 2.2 deg on the earlier masses and the 30 Oct tree, 1.74 vs 1.80 now) X1 also re-flies on the earlier masses: its 44 runs (configs/batches/oldlinks1106_4_x1.txt, staged in ~/mdc_overnight/x1_oldlinks_stage/ and installed by ~/mdc_overnight/claim1106_x1old.sh after the running batch, so the running chunks' recorded diffs stay one tree), with two more *_oldlinks worlds (four_rigid_ground_rig_3915_L100, its _p115). The new-mass X1 (R3821-R3906, F059/F060) stays in the registry.
DECIDED 2026-11-06  (Wesley ~14:20, the laptop at 1.5 GB free: "I dont need replays, any that I want a video for I can just rerun") The batch no longer fetches each run's Gazebo replay (tools/remote/lab_batch.py fetch: --exclude gz_record/; from the next chunk on, the running chunk keeps fetching). The replays already on the laptop (157 runs of 5-6 Nov, 9.0 GB) are Wesley's to delete (he runs the command). Orchestration only: the code that flies is unchanged, but the chunks started after this edit record lab_batch.py in their laptop diff (the one-tree hash moves; noted in the claim batch manifest).
DECIDED 2026-11-06  (Wesley ~14:30, four answers) (1) QUAD4's tracker gets the carriers' drop rule (card docs/experiments/2026-11-06_quad4_tracker_drop_rule.md; critic WAIVED by Wesley): built and tested in a separate git worktree now (the running batch untouched), unit and launch tests and twin checks, merged after the batch ends, ready for Wed 11. Closes item (1) of OPEN 2026-11-06. (2) The thesis keeps its structure, simulation first then real world; within it the recommended framing: the real-world detach, re-attach and pickup as demonstrations with their counts, X1 as a supporting mechanism result (not a headline), the abstract's re-attach singular with its conditions. Item (2). (3) Wednesday's order stays as written (item (3) closed: no change); Wesley wants to review the plan of the four sessions left (Wed 11, Thu 12, Wed 18, Thu 19), and his supervisor asks to prioritise the results for the combined paper with him and Te-Jen. (4) The thesis names the 0.86 kg ring on 1 m cables "Setup 3", and states the catch radius as flown: 25 mm for the real confirmation, 10 mm in the simulation. Items (4), (5). All items of OPEN 2026-11-06 are now closed (the funnel: DECIDED 6 Nov morning).
DECIDED 2026-11-06  (Wesley ~14:45, the combined paper with his supervisor and Te-Jen; its Overleaf clone: multi_drone_swung_payload_item_collection_wesley_version/main.tex; the supervisor: prioritise its results) The paper's real-world sections needed from the sessions of 11, 12, 18 and 19 Nov: attach performance and detach performance, each evaluated separately (reliability, settle time, payload swing; static and moving payload), and the application (the full mission in one flight). Te-Jen already has its single-agent path-planning results. (1) The detach is the UNANNOUNCED release (rel: the magnet switched off). (2) The moving case is the orbit with the attach pause (stop, attach, resume): an attach on a moving ring is unlikely to work on the hardware. (3) The application flies the triangle (fm9), with the funnel once printed: the square's catch (sq8_2) looked lucky on the day. (4) Repeats: 5 static and 3 moving per condition.
DECIDED 2026-11-06  (Wesley ~14:43) The plan of the four sessions (docs/rig/plan_sessions_2026-11-11_to_19.md) is approved as drafted: Wed 11 the triangle all session (rel x5, then the re-attach x5 with the funnel if fitted); Thu 12 relo x3 (the release on the orbit), nro x3, k1z x2; Wed 18 fm9 x3 and the gaps; Thu 19 fm9 on the orbit, video takes, the demo. The 1 m baselines and the square claim leave the plan. Also: if QUAD4 struggles to pick the object up (the other quads' wind in the cage), the drop cases may start with the object already on QUAD4's magnet (his planner's delivery_rejoin mode); and the lab chat works up other ideas to help the attach (hardware or otherwise).
DECIDED 2026-11-06  (Wesley ~15:05, thesis chat) The thesis keeps three real-world setups, to report thoroughly what was tried: Setup 1 the 0.86 kg M2A ring on 0.55 m cables (until 28 Oct), Setup 2 the 1.15 kg ring on 1 m cables (29-30 Oct, sq6b 4 Nov: too heavy for the platform, the quadrotors near their throttle limit, the T out of thrust, and the batteries flat after one or two flights (Wesley)), Setup 3 the 0.86 kg ring on 1 m cables (from 4 Nov). The thesis's tab:setups, 3.1, 3.3 and 3.4 now name all three; Results reports Setup 2 as tried (one figure-eight, the thrust limit, the batteries, the drop rule caught all four of its drops) and the bigger object's impact on Setup 3 (two magnets pulled off at once). Closes the F053 naming question.
DECIDED 2026-11-06  (Wesley ~15:20, the sessions' setup; docs/analysis/attach_ideas_2026-11-06.md) (1) Funnel v3 on plate 9 for EVERY flight from Wed 11 (rel included, one setup for the paper), v2 the spare; a 30-45 min bench check at setup (Wednesday's sheet); flown as the only change (Te-Jen's match, dwell and descent as flown). (2) relo flies without the 5 s hold (no params file for it). (3) fm9h (the object held from take-off: drop into the ring, then attach to plate 9; no detach, no pickup) is the application's fallback: fm9 first, fm9h after two fm9 flights that fail at the pickup; the paper says which flew. The spare steel oval and the swing gate (ideas 4, 5) wait for Wednesday's funnel flights.
DECIDED 2026-11-06  (Wesley ~15:20) With funnel v3 on plate 9 (its inward rim ~11.8 cm from the ring centre), the drops release within 8 cm: `drop_on_arrival_xy:=0.08` typed in rig_k1z.sh, rig_fm9.sh, rig_fm9h.sh, rig_fm9o.sh (was the default 0.12). Why: all three 5 Nov drops released toward plate 9 (+4.1, +9.0, +5.6 cm; QUAD4 arrives from that side), and a 70 mm object released at +9 cm would clip v3's rim.
BUILT   2026-11-06  (~14:35-15:20, an agent in the worktree .claude/worktrees/agent-aa650a10b71d57ce6, branch worktree-agent-aa650a10b71d57ce6; critic waived by Wesley) QUAD4's tracker gets the carriers' drop knobs: graph_attach.py passes tracker_drop_options (and the now unused tracker_land_options is deleted from real_mode.py); test test_quad4s_tracker_carries_the_carriers_drop_knobs (fails without, passes with); launch tests 157 passed, tracker tests 153, gate --quick PASSED (2195). Twin R3915/R3916 (a carrier released 5 s after the re-weld, drop 55): the planner's own loss check drops the ring before any tilt rule in both, so the fast-upset case is not reproduced in the twin; the fix is in effect (QUAD4's tracker reads drop_on_loss true). R3917 (a normal re-attach with the fix): as R3714, no spurious drop request. To merge into week8 after the batch and X1's old-mass re-fly (Sat), then gate, the desk check, before Wed 11.
DECIDED 2026-11-06  (Wesley ~15:30: the magnet's wire fitting does not pass v3's 72 mm base; "make the diameter of the base part of v3 2 cm wider, the height ideally 8.5 cm"; then "92 mm base, 2 outer bolts") Funnel v3 now has a 92 mm base (bore radius 46), 85 mm tall, the same 214 x 164 mm mouth at 45 deg (59.5 deg sideways), 91 g; the disc's play is 14 mm. The ring's inner M4 hole (44.6 mm from the plate centre) falls inside the base, so v3 bolts through the two outer holes only (bosses radius 8 mm); each nut pocket opens ~2 mm into the base below 10 mm up (the nut seats on about two thirds of a full seat). cad/funnel/funnel.py VARIANTS v3 (a `bolts` parameter); files in ~/Downloads/funnel/ (funnel_v3.*, funnel_v3_92mm_base.png); Wednesday's sheet updated.
DECIDED 2026-11-06  (Wesley ~15:40: "85mm is fine for the base" = 85 mm across; "I am using TPU") Funnel v3, printed in TPU: base 85 mm across (bore radius 42.5), 85 mm tall (the mouth trimmed 3.5 mm a side to 207 x 157 mm), the wall kept at 45 deg (59.5 deg sideways): TPU sags on shallower overhangs and its grip needs the steeper wall for the disc to slide (tan 40 = 0.84 against TPU's ~0.5-0.8); the disc's play 10.5 mm; two outer bolts with full nut seats (the inner hole breaks into the base by 0.2 mm); 83.5 g at TPU's 1.21 g/cm3; the inward rim at ~121 mm from the ring centre. Replaces the 92 mm base of ~15:30.
DECIDED 2026-11-06  (Wesley ~15:55: "can you call it v4?") The TPU funnel with the 85 mm base, 85 mm tall, 207 x 157 mm mouth, two outer bolts and the zip-tie notch (around the net's zip-tie hole, which an 85 mm base would otherwise sit on) is v4 (cad/funnel/funnel.py VARIANTS['v4'], 83 g); v3 is back to its 72 mm base (97 mm tall, PETG, 3 bolts) for the record. v2 and v3 have the 72 mm base the magnet's wire fitting does not pass, so v4 has no spare unless a second is printed. The sheets say v4. By geometry on the 24 plate-9 descents: v4 21 into the mouth, 14 fully inside.
FINDING 2026-11-06 17:29  One-quad rig tests (RIG-1106-q4f, -q4f_2, VOID): two of Te-Jen's stacks ran at once on QUAD4's
        radio and `ros2 topic pub --once` reached only the first subscriber (TAKEOFF one controller, DISARM the other): QUAD4
        flipped 3 s after TAKEOFF and its flying controller stayed armed ~37 s, also while it was picked up. Fixed in the
        lab chat's scripts (no src/ change): every one-quad script refuses while any flight stack is up, and a bridge
        (results/rig/2026-11-06/scripts/one_quad_panel_bridge.py, node name fleet_manager) drives his stacks from the RViz
        ArmPanel (ARM / TAKEOFF / LAND / SPACE = DISARM; DISARM x10 to his controller and a disarmed ELRSCommand to his radio
        for 2 s). Desk-checked (domain 98). Never `pub --once` to a topic with several subscribers for flight commands.
CORRECTION 2026-11-06 17:33  (the FINDING of 17:29) QUAD4's flight controller disarmed itself at the flip (CRSF mode AIR ->
        !ERR*) and refused to arm afterwards (q4f_3, q4f_4): the ~37 s "armed" was our controller's command only; the
        motors were off. The bridge now shows the FC's mode on the panel and refuses ARM while it reads !ERR (replug).
DECIDED 2026-11-07 10:10  (Wesley) X3 on the earlier masses: "Those results are fine, you can just change the bars because they
        are arbitrary". The X3 card's bars (N4, J's 20 s, the b2b overlap, P) were frozen 23 Oct, before his rule of 2 / 5 Nov
        (no bars unless compared with another paper). Recorded openly: the card keeps its bars and the misses as scored
        (R3922c, F065 / F066), and Wesley's judgement after the runs replaces the verdicts: X3's results are ACCEPTED (judged
        after the run). The thesis shows the numbers, not the bars.
DECIDED 2026-11-07 10:10  (Wesley) The six X3b runs on the earlier masses stay queued after X1 (x3b_s8_hover_rt, x3b_s8_fig8_rt,
        x3b_s9_hover_rt x2), so C8 and N9 each sit on one quadrotor model.
DECIDED 2026-11-07 10:10  (Wesley) X8's hub: add our arm with the sideways integral off (x8_hub_ours_noxy x2, int_k_xy 0, staged
        in ~/mdc_overnight/x1_oldlinks_stage, flown after X1), so both X8 cells follow the 5 Nov T rule; the as-flown hub
        runs R4008 / R4009 stay recorded.
DECIDED 2026-11-07 16:52  (Wesley) Bars on the simulation claims are arbitrary: "it is fine to state results as is and compare to
        other papers where possible". Applies to every claim of the 5-7 Nov batch (X1, X2, X3, X3b, X4, X8, 4.1 / 4.2, X11 U,
        joins, rejoin, Quan, the gap table): the registry and the cards keep every miss as scored; the thesis states the
        numbers as they are, and compares with other papers' published numbers where possible.
DECIDED 2026-11-07 16:52  (Wesley) Te-Jen's one-quad changes are not needed: the one-vehicle M2 15 deg proof (rig_q4m2.sh, never
        flown) and LAND beside the magnet (landing_side_step_m, desk-checked only), both only in the overlay ~/mdc_m2one_ws
        (src/ untouched). Recorded (results/rig/2026-11-06/scripts/overlay_tejen_mission_2026-11-06.diff), overlay deleted.
DECIDED 2026-11-07 16:52  (Wesley) No one-quad tests in the lab sessions: "that is tejens job and he has done all he needs to do".
        Multi-quad testing is allowed only 08:00-11:00, so those hours go to multi-quad flights; anything one-quad, if ever,
        outside 08:00-11:00. The Wed 11 plan stays as approved (rel x5, nrtf x5, k1z if time).
FACT    2026-11-07 16:52  (Wesley) QUAD4 is fine after the 6 Nov one-quad crash. The 'week8' session on this laptop is the thesis chat.
DECIDED 2026-11-07 17:46  (Wesley) X1 is reported on BOTH quadrotor models, side by side, with the conditions stated: the
        hardware-matched masses (R3821-R3906, F059 / F060) and the earlier split (R4150-R4193, F087 / F088). The square-to-T
        benefit is the earlier model's only (0.6 vs 2.2 deg with the sideways integral off; 1.74 vs 1.80 on the final
        masses). For this table the thesis explains the two mass splits.
DONE    2026-11-07 18:34  QUAD4's tracker drop rule (card docs/experiments/2026-11-06_quad4_tracker_drop_rule.md; critic waived;
        twin R3915-R3917) applied to the working tree after the claim batch, as planned: graph_attach.py gives QUAD4's tracker
        tracker_drop_options (the carriers' drop knobs as typed), tracker_land_options removed; its test added. gate --quick
        PASSED (2190 passed, 17 skipped), the rig desk check 9 OK (results/rig/2026-11-11/desk/check_2026-11-07_1833.txt).
        The worktree .claude/worktrees/agent-aa650a10b71d57ce6 is now redundant (Wesley removes it).
FACT    2026-11-07 18:34  The October simulation results are archived (results/archive/<day>.tar.zst). Analysis and figure tools
        name many October runs (tools/thesis_runsets.py, release_bands.py, thesis_rig*_figures.py: ~300 runs); extract a run
        on demand: tar -I zstd -xf results/archive/<day>.tar.zst -C results <day>/<run dir>. Restored for the gate:
        R1140, R1145, R1206-R1208, R1211-R1213 (10-07 / 10-08), R1459, R1460 (10-10), R1626 (10-11).
DONE    2026-11-07 18:35  tools/yaw_track.py aligns the planner log on run.csv's clock through record.json's t_ready_sim (twin
        runs; rig runs keep the lift-off alignment): R4028 now reads 4.0 deg RMS, delay 0.13 s, as F077's recompute; its
        tests pass. The Gazebo replay (gz_record) was already opt-in (config default false; only --record, a record
        section or the demo configs turn it on); the lab batch's copy-back skips it since 6 Nov.
DECIDED 2026-11-09 23:02  (Wesley, via the thesis chat's Q011) Re-fly the 4.3 joins on the 0.86 kg M2A ring on the lab PC:
        the six join cells with only the ring changed (worlds without _p115; the planner's ring as every final claim on
        this ring: attach_radius 0.225, attach_z 0.02, payload_rest_z 0.02 as the world's plates, load_mass 0.86, inertia
        untyped; his geometry rig_twin_L100), x2, from 22:58. The pickup cell needed more than the ring (the M2A worlds
        carry a ball, not the plate): on Wesley's word a NEW WORLD sq_rigid_ground_rig_attach_pickup_L100_plate.sdf (the M2A
        world with the p115 plate block; tools/make_m2a_plate_world.py, --check) and his geometry rig_twin_L100_plate.
DONE    2026-11-10 00:06  Q011 answered (feed F097 + its reviewer's corrections F098; registry R4214-R4227, R4214c). The re-fly
        that counts flew from 23:14 on 9 Nov on the PLATE worlds (Wesley: every cell on a plate world, so only the ring
        changes; sq_ / three_rigid_ground_rig_attach_pickup_L100_plate.sdf from tools/make_m2a_plate_world.py), his geometry
        rig_twin_L100_plate, QUAD4's weld point 0.225 / 0.032, LAND 30 s after the weld (the thesis chat's addendum). The
        first attempt (worlds without _p115, geometry rig_twin_L100; R4202-R4213) is VOID. Figure 15: R4227.
DECIDED 2026-11-10 23:10  (Wesley, after the supervisor's message of 10 Nov: "results are king", at least a full run once and
        3-5 detach and re-attach flights; it may take 10 attempts for one success; the funnel is fine; the circling case
        matters less for the re-attach) The rig plan for the four sessions left, replacing the 6 Nov order:
        (1) detach and re-attach: nrtf in hover x8 (5, plus "add 3 x nrtf"), then nro x3 (detach on the circle, the three
        circle on, his TAKEOFF stops the ring for the re-attach, four circle on), Wed 11;
        (2) the drop into the circling ring: k1zo x3 (k1z on the 0.3 m orbit, small object only; new script), Thu 12;
        (3) only if the pickup keeps failing: the held object dropped into the circling ring, then the re-attach;
        then the FULL MISSION, defined by Wesley as on the circle: detach on the circle, the pickup and the drop with the
        three circling, the ring stopping for the re-attach, four circling on (18th-19th; fallback fm9 in hover).
        The full mission's stop starts AUTOMATICALLY when his planner starts the re-attach (Wesley's choice over a button):
        new switch in our planner, default off, tests and a simulation run. Dropped: rel and relo (the unannounced
        releases), fm9o (the circle paused from his TAKEOFF, before the pickup), k1z as a line (backup only).
        Funnel: v4 first, then the supervisor's (105 mm mouth, 11 g; guides the 64 mm disc only within ~2 cm).
        The sheets are rewritten after Wesley has read the pre-measurements (his ask, 10 Nov ~23:05).
OPEN    2026-11-10 23:10  The re-attach descent bands with the funnel (now match_xy 0.08, descent_abort_xy 0.10): an analysis of
        the rig descents is running (results/analysis/attach_band_2026-11-10/); Wesley decides the values.
OPEN    2026-11-10 23:10  The thesis's real-world freeze is set for Thu 12 11:00; the full mission (18th-19th) reaches the
        thesis only if the freeze moves to Thu 19 (the rescope's latest). Wesley's call.
CORRECTION 2026-11-10 23:27  of DECIDED 2026-11-10 23:10: "add 3 x nrtf" means 3 flights of the detach from plate 9 and the
        re-attach on PLATE 1 (nr1r's route round the outside, 5 Nov; the funnel moved to plate 1): nr1f x3. Experiment (1) is
        nrtf x5 + nr1f x3 + nro x3, not nrtf x8.
DECIDED 2026-11-10 23:27  (Wesley) The thesis's real-world freeze moves from Thu 12 to Thu 19 Nov (closes the OPEN of 23:10).
        Wednesday is not limited to the attach lines: one list of everything, in priority order, flown as far as the
        session gets (the pickup lines included). The setup pre-checks are cut to what Wesley asked (his batteries; body 24
        in the funnel read from Friday's q4f_5/q4f_6 instead of a bench check; W/X/Y kept as measured 4 Nov).
FACT    2026-11-10 23:25  Body 24 inside funnel v4 (Friday's one-quad flights RIG-1106-q4f_5, -q4f_6, from DESCEND to the
        end, 0.29 m down to seated): 238-239 Hz, longest gap 21-24 ms, none over 50 ms, no frozen stretch over 9 samples.
DECIDED 2026-11-10 23:42  (Wesley: the recommended bands become tomorrow's default) His re-attach descent bands with the
        funnel, from results/analysis/attach_band_2026-11-10/summary.md (32 rig descents of 4-5 Nov; exploratory): funnel v4
        keeps the bands as flown (match_xy 0.08, match_dwell 0.5, descent_abort_xy 0.10, descent_hold on); the supervisor's
        105 mm funnel flies 0.05 / 0.5 / 0.05 through `BAND=cone` in front of any attach script (rig_nrtf, nr1f, nro, fm9c,
        fm9hc, fm9, fm9h). Why: the start offset does not predict the arrival (r +0.02), with descent_hold on
        descent_abort_xy pauses the descent down to the mouth (the only band that aims, ~2.5 cm at best; his code raises it
        to at least match_xy), and below 4-5 cm QUAD4 would often not try. Closes the OPEN of 23:10.
FACT    2026-11-10 23:42  Built for the list (Wesley 10 Nov): attach_pause_at ('takeoff' default = as before; 'reattach': the
        pause only from his re-attach leg: TRANSIT_TO_REATTACH, APPROACH_ABOVE_TARGET, MATCH_VELOCITY, DESCEND_TO_ATTACHMENT,
        latched until the fold-in), dissipative_node.py + common.yaml + real_mode.py, tests in test_attach_pause.py (2) and
        test_real_mode_launches.py; console DETACH QUAD4 and QUAD4 magnet OFF (two presses) / ON (tools/console.py, 3 tests);
        scripts rig_nr1f, rig_k1zo, rig_fm9c, rig_fm9hc (results/rig/2026-11-11/scripts/), params
        attach_m1_1m_m2a_around_drop55_det15.yaml and attach_m1_1m_m2a_orbit_reattach_det15.yaml. gate --quick PASSED
        (2193 passed) before the console change; rig desk check 13 OK. Critic not run: a trigger for an existing pause, not
        a new control law (Wesley may waive or ask for one).
DECIDED 2026-11-10 23:50  (Wesley ~23:48) The drop has a switch with a default per case: on the circle (k1zo, fm9c, fm9hc) his
        settle drop by default (drop_on_arrival off, drop_height 0.48: over the moving drop point at 2.76 m, match its
        velocity, settle, lower, release), `DROP=arrival` for the drop on arrival; in hover (k1z, fm9, fm9h) the drop on
        arrival stays the default (0.28, as flown 5 Nov), `DROP=settle` for his. Desk-checked every combination (heights
        2.56 / 2.76 m, under max_body_z 2.8). Pushed back once: on 4 Nov in hover his settle gate passed at most once a flight
        (the object swung 23-26 cm round the drop point, RIG-1104-k1m_2..4); Wesley's call stands. Every line and option is
        simulated overnight (~/mdc_overnight/plan1110.sh, plan1110b.sh; configs/experiments/rig_*_twin.yaml).
FACT    2026-11-11 06:38  Overnight twin checks of every rig-list line and switch (R4228-R4244, registry; laptop, tonight's
        tree): WORKED fm9c (settle R4231 / arrival R4228: the circle ran through pickup and drop, the ring stopped by itself
        at his re-attach leg, first-try re-weld, the circle resumed), k1zo (R4232 / R4229), fm9hc arrival (R4230), fm9 and
        fm9 settle (R4239 / R4242), k1z and k1z settle (R4238 / R4241), fm9h (re-fly R4244). DID NOT RE-ATTACH in the twin:
        nrtf, nrtf BAND=cone, nro, nr1f (R4233, R4237, R4234, R4235; ring 1.2 m too, R4243): the tip centred on plate 9 and
        pushed by the twin's rigid rod (contact_dz -0.03) sinks into the plate, outside the twin's contact window; their
        descents, bands, nro's pause and resume and nr1f's route ran as designed. Twin flake: 2 of 4 held-object runs never
        climbed after his TAKEOFF (R4236, R4240).
DECIDED 2026-11-11 07:18  (Wesley: "test the tighter band on the new funnel, to make sure we don't get stuck") Line 1b on the
        sheet: `BAND=cone bash ~/rig_nrtf.sh nrtfc` x2-3 on the supervisor's funnel; stuck = MATCH_VELOCITY or "Contact
        descent held" for 30 s: GIVE UP, next flight `BAND=mid` (new: 0.06 / 0.5 s / 0.06 in the 7 attach scripts, desk OK),
        then the default. The twin cannot answer it: its QUAD4 holds still (R4237: MATCH 0.56 s at 5 cm, no pause), the rig's
        tip swings ~3.8 cm (RIG-1014-t6f). History: his defaults 2.5 cm / 0.08 m/s held 1 s never held on 14 Oct (t6f) ->
        5 cm / 0.15 m/s; 29 Oct at 5 cm / 1.0 s sq2_5 waited 12.3 s and a second MATCH 9.9 s with no window, and the 8 cm
        abort (no hold) cut every descent of sq5_2 -> 8 cm, 0.5 s dwell, the 1 s debounce, then the 0.10 hold (5 Nov).
FACT    2026-11-11 08:37  Plate 1 vs plate 9 measured with QUAD4's magnet seated (payload_geometry.py, T1 four markers): plate 1
        marker (+0.2013, -0.1119) m, 4.27 cm over the ring pivot; plate 9 (+0.0048, +0.2276), 4.95 cm (4 Nov's typed 0.0, 0.2293,
        4.91: plate 9 unchanged). Plate 1 sits 6.8 mm lower. rig_nr1f.sh now types W 0.0427, X -0.0037, Y 0.2303 (the
        launch turns it onto plate 1's measured point; QUAD4's start on plate 9 within ~1.1 cm, K 0.025); desk OK. Body 6
        (08:30): contact height 37.6 mm (typed 38), marker 0.5 cm off the object's centre: unchanged.
FIX     2026-11-11 09:41  magnet_gate race (RIG-1111-k1zo_3): his OFF reaching the gate before his DROP_OBJECT phase was held
        and never sent (the object stayed on the magnet). Now a held OFF goes when his phase reaches DROP_OBJECT /
        CLEAR_DROP_ZONE, unless an ON of his came after it (src/drone_magnet/drone_magnet/magnet_gate.py, test added; 21 pass).
        Also 09:25: his settle drop's release is typable (`drop_release`, partner_mission.launch.py; script switch RELEASE,
        k1zo default 0.03) after RIG-1111-k1zo's impact detached the ring.
FACT    2026-11-11 10:07  (Wesley, lab) QUAD4's magnet lead was not fully home in QUAD4: pushed in, the grip came back. The
        uncommanded releases of RIG-1111-nrtfc_3 (4 s after the proof) and -nrtfc_4 (in the pretension) are put down to
        it; check the lead before each flight.
FIX     2026-11-11 10:39  His drop check (our fork, online_join_planner.update_measured_drop): an object still on the magnet at
        the end of the 0.4 s release dwell now gets drop_puck_wait_s (2 s on the rig) to leave before the give-up
        (RIG-1111-fm9hc_8: the object rode the magnet 0.8-0.9 s after the coil OFF, then fell into the net; he had given up
        at 0.4 s). Tests updated (test_m1_contact_attach.py, 69 pass); CHANGES_FROM_TEJEN.md to follow.
DECIDED 2026-11-11 10:47  (Wesley, lab: fm9c's pickup failed in the circling carriers' wind) attach_pause_at 'pickup_reattach'
        (our planner): the circle also comes to rest for his pickup (APPROACH_ABOVE_PICKUP .. MAGNET_ATTACH_WAIT), circles
        again from LIFT_OBJECT for the drop, and stops again for his re-attach. Switch PAUSE=pickup on rig_fm9c.sh
        (configs/rig/attach_m1_1m_m2a_orbit_pickreattach_det15.yaml); test added (6 pass); desk OK; not simulated (no time).
FACT    2026-11-11 11:32  Today's code (magnet_gate held-OFF fix, his drop check wait, drop_release, attach_pause_at
        pickup_reattach, console buttons, OBJ=big / PAUSE=pickup / RELEASE switches): gate --quick PASSED (2199 passed;
        test_floor_join_delivery_rejoin's stuck-drop test moved to the 2 s wait), rig desk check 13 OK. Not simulated yet.
FINDING 2026-11-11 11:32  The pickup is the full mission's blocker, and it is alignment, not wind: every pickup descent that
        reached MAGNET_ATTACH_WAIT within ~2 cm of the object lifted within 1.5 s (k1zo, k1zo_3, fm9c_4 #3, fm9c_6: 4 of 4);
        every one 4.7-9.7 cm off failed after 13-23 s ('vertical separation'; fm9c, fm9c_3, fm9c_4: 7 of 7), with the circle
        running or at rest (fm9c_3). The off-centre arrivals came after QUAD4 flew in from the ring (fm9c), the centred ones
        from its mat (k1zo) and on later tries.
FINDING 2026-11-11 11:41  QUAD4's bounce (docs/analysis/quad4_bounce_2026-11-11.md, RIG-1111-bounce): its cable runs near slack
        because the level-ring split on plates 3/7/9/11 gives plate 9 the smallest share (1.62 of 9.74 N planned, dips to
        0.55 N; 0.3-1.1 N measured); bounce ratio vs slack share r +0.77 over 20 flights; after a re-attach the fold-in ramps
        QUAD4 up from zero. Not the mux, the tracker settings, the thrust map, the object mass or the circle.
OPEN    2026-11-11 11:41  For Wesley: raise the planner's tension_min (0.1 N) to ~1.0 N (solver rebuild, twin, rig) and/or start
        the fold-in at that floor and hold QUAD4's kT estimate through it.
DECIDED 2026-11-11 15:56  (Wesley, plan approved; ~/.claude/plans/curious-frolicking-castle.md) Thu 12 Nov, "minimum first":
        (1) nrtf in hover until 3 more clean re-attaches (5 with nrtfc_2 and nrtfc_5), (2) fm9, the full mission in HOVER,
        until one completes, (3) the circle versions (nro, then fm9c PAUSE=pickup) with the time left, (4) 1-2 late
        comparison flights with the bounce switch on. Setup: the supervisor's funnel on plate 9 (Wesley: on from about
        10:15 on 11 Nov; nrtfc_5, the catch that held, was in it), the 5 cm bands as the scripts' default, the small
        object only, with one of the supervisor's funnels fitted on it (body 6 redefined, contact height re-measured),
        the settle drop with the 3 cm release as the default in hover and on the circle, a FORCE DROP console button
        (new), a fixed camera on every flight. The bounce fix: built behind a switch, default off, flown late. Dropped
        from the plan on Wesley's word: the console layout gate (his arm-then-place-then-restart routine explains the
        launches that never armed) and the held-object takeoff loss (a weak magnet connection, fixed).
FINDING 2026-11-11 15:56  RIG-1111-fm9c_6, the fly-away (Wesley: QUAD4 "went really high, flew into the wall around 3 m high",
        DISARM): it began before LAND under his controller. The big object fell off 7.6 s into the lift (LAND -2.80 s);
        QUAD4 went from (-0.47, 1.63, 1.58) m at -3.0 s to (+0.20, 2.30, 2.88) at -1.0 s (the pickup spot was at
        y 1.77 m, past the 1.75 m limit, which by the 22 Oct decision does not cover QUAD4 under his stream). From
        -0.67 s QUAD4's pose (body 14) was frozen at (0.29, 2.34, 2.89): /drone_3/motion_capture_state kept arriving at
        94 Hz with a bit-identical pose until the DISARM at +4.2 s (the magnet marker kept changing until +2.83 s). Our
        tracker took over at LAND on that frozen pose (velocity estimate 0, a constant command, reference y held at 1.70 by
        the planner); the 0.25 s pose watchdog reads arrival times, so it never fired. Fix planned (Wesley's word, in the
        approved plan): an unchanged pose counts as no pose (the mocap node stops publishing it; critic first).
FINDING 2026-11-11 15:56  QUAD4's bounce, second reading (supersedes the cause in FINDING 11:41; the slack-cable part stands):
        body-to-magnet distance shows the cable going slack and taut 22-36 mm deep at 3-4 Hz in fm9c, fm9c_3, fm9c_4
        (slack 36-69 % of the lift and hover) and 5-7 mm deep in nrtfc_2, nrtfc_3, nrtfc_5, fm9c_6. Planner tick, tracker
        loop, mocap rate, mux delay, packs: the same in both groups. Twin R4233 / R4243 / R4239 / R4228: ring height 1.2 m
        and the circle change nothing there. With four carrying the ring hangs 3.7-5.4 deg off level in 6 of 7 flights
        (plate 9 low 14-19 mm, plate 3 high); in fm9c_3 it was level within 2 mm and QUAD4's cable was slack 69 %. Cause
        open: tonight's comparison.
FACT    2026-11-11 16:27  The small object with one of the supervisor's funnels on it, body 6 redefined by Wesley (props off,
        m1_irl_pickup measure + pickup_geometry.py, raw packets logs/motion_capture_udp/motion_capture_udp_20261111_162210.csv):
        magnet marker 11.4 mm above body 6's pivot and (-13.1, -8.9) mm from it in the object's axes with the magnet seated
        in the funnel (object yaw -4.3 deg, tilt 0.4 deg; all three bodies live at 240 Hz). The six pickup scripts type
        PC 0.011, PX -0.0131, PY -0.0089, PS 0.105 (new launch args pickup_point_x / _y -> his payload_pickup_point_x / _y
        and his manager's pickup_point_offset_x / _y); PT 0.050 is a placeholder until Wesley gives the height; OBJ=bare
        keeps 0.038 / 0.022 / 0.07.
FINDING 2026-11-11 16:27  In the same packets streaming ID 14 arrives at 480 Hz as TWO live bodies: QUAD4 on the floor at
        (-0.776, -0.007, 0.064) and another at (0.020, 1.540, 1.218). Told Wesley at once: one of them must lose the ID
        before any flight (QUAD4's pose would alternate between them). Not yet confirmed fixed.
FACT    2026-11-11 16:38  The object with its funnel, second seat (Wesley turned it a quarter turn and set the magnet back in
        the centre, 16:36): the magnet at (-4.9, -10.7, +11.0) mm in body 6's axes against (-13.1, -9.0, +11.4) at 16:25,
        8 mm apart; body 6 tracked at 240 Hz with no repeated pose while the magnet sat in it. The six pickup scripts type
        the mean: PX -0.0090, PY -0.0098, PC 0.011. PT stays 0.050 (Wesley: no height measurement; it is only his
        obstacle box and RViz marker: the release height is the magnet's, as in RIG-1111-fm9hc_8). Excerpts of the raw
        packets: results/rig/2026-11-11/desk/object_funnel_seat{1,2}_udp.csv.
DECIDED 2026-11-11 16:38  Streaming ID 14's second body (FINDING 16:27) is another user's, as are any other IDs streamed
        this afternoon (Wesley 16:33: "for someone else ... i will not use those tomorrow"). Not ours to change today;
        before the first launch on 12 Nov only our bodies are streamed. tools/mocap_stream_check.py now also fails on a
        shared ID (a body arriving at more than 1.5 x the camera rate), which it passed before.
FINDING 2026-11-11 16:44  The frozen-pose guard after its critic (card docs/experiments/2026-11-11_frozen_pose_guard.md):
        "fly it with changes". Done: the four quads' bodies only (the ring, the magnet marker and the object pass as
        before: withheld, they stall his pickup, his attachment manager and carriers_down_alone); the check before the
        parser; the velocity re-seeded on resume; the desk check on the real node (tools/frozen_pose_desk_check.py: OK).
        Replay at 0.5 s over the 43 flown flights of 4, 5, 6 and 11 Nov (172 quad recordings): no carrier ever withheld;
        QUAD4 only at |y| >= 1.72 m: fm9c (0.17 s, no effect: under the 0.25 s pose rule by 0.08 s), fm9c_4 (from LAND
        +19.9 s, our take-back; the carriers down at +13.2 s; operator DISARM at +21.7 s), fm9c_6 (from LAND -0.17 s).
        OPEN for Wesley (asked 16:43): a QUAD4 freeze while OUR tracker flies it off the ring is a fleet-wide EMERGENCY
        STOP (welded starts publish no on_load flag): accept it, or type welded_on_load (twin only, 11 Oct), or leave
        the guard off on 12 Nov. The guard is OFF in the code (MDC_MOCAP_FROZEN_S, default 0) until he answers.
FINDING 2026-11-11 16:44  fm9c_4 was a second blind flight: QUAD4's pose froze at (1.02, 1.72, 1.14) at LAND +19.4 s under
        our take-back and Wesley's DISARM came 2.2 s later. It got there by design: a take-back with the object on steps
        QUAD4 out to 2.23 m from the ring (the 1.2 m clear radius + the cable, so the object lands clear of the carriers),
        cut back to 0.10 m inside the position limit: (0.99, 1.63), then it drifted to y 1.75 (POSITION LIMIT line). The
        same step out is the POSITION LIMIT of fm9hc_2 and fm9hc_4. Not changed (landing nearer puts the object or QUAD4
        on the carriers, R1482). Sheet rule for 12 Nov: with the object on and the drop stalling, FORCE DROP, not LAND.
FINDING 2026-11-11 17:27  The fly-away of RIG-1111-fm9c_6, its start explained (the bag and his log; it was "unexplained"
        in the 15:56 finding and in the guard's critic). (1) The bigger object came off the magnet by itself 7.46 s into
        his LIFT_OBJECT while swinging 0.4 m sideways (its pose froze at z 0.52 m and it was on the floor 0.28 s later:
        a free fall; his "measured pickup proof separation; magnet OFF" came 0.18 s AFTER it left, so his OFF did not
        drop it). (2) It slid to (-0.07, +2.16) m, 0.36 m further out in y than its start (-0.38, +1.80). (3) His retry
        (LIFT_OBJECT -> APPROACH_ABOVE_PICKUP) flew to it: his reference went from (-0.35, 1.75, 1.98) to
        (0.25, 2.15, 2.54) m in 2.4 s at up to 0.8 m/s sideways, and QUAD4 overshot it to (0.29, 2.34, 2.89), 0.35 m
        high (just rid of ~0.15 kg) and 0.19 m beyond in y, where motion capture lost it. His planner has no bound on
        where a pickup may go (only visual_astar_bounds, +-5 m). So an object that falls and rolls outward takes QUAD4
        out of the tracked volume by itself, whatever the object's start. OPEN for Wesley: a typed pickup box in our
        copy of his planner (the object outside it: no retry, he lands QUAD4 in place); built behind a switch tonight.
        The guard card's sentence "his proof then ... switched the magnet off with the object in the air" is
        superseded by (1).
DECIDED 2026-11-11 20:19  (Wesley, asked with the options) The frozen-pose guard stays OFF on the rig: "We should have it
        off because I will disarm in this situation anyway it does not need to be automatic". The code stays (default 0,
        MDC_MOCAP_FROZEN_S; card 2026-11-11_frozen_pose_guard, its F3 question closed by this); the twin runs of its
        downstream (rig_fm9_q4lost_*) finish as records only. Closes the OPEN of FINDING 16:44.
DECIDED 2026-11-11 20:19  (Wesley: "Make the limit 1.9 m") The pickup box is ON at 1.9 m in the six pickup scripts
        (PBOX=0 off): an object further than 1.9 m from the cage centre in x or y while his planner would fly to it is a
        give-up and our take-back. Said once: motion capture lost QUAD4 at |y| 1.72 m and beyond on 11 Nov, so at 1.9 m
        he can still be lost over an object lying between 1.7 and 1.9 m; fm9c_6's object (y 2.16 m) would have been
        refused. Unit-tested and desk-checked; not simulated, not flown. Closes the OPEN of FINDING 17:27.
FACT    2026-11-11 20:37  The funnel on the small object is about 4 cm tall from the steel top to its rim (Wesley). So the pickup hold
        (the last 8 cm, 4 cm sideways) is already acting 4 cm above the rim, and the 105 mm mouth (5.2 cm radius) is wider
        than the hold: no change to the scripts. PT 0.050 stays.
CORRECTION 2026-11-11 21:13  (Te-Jen's reply to the 11 Nov note, read from the code) (1) Our note to Te-Jen, the 15:56 plan
        and my explanation to Wesley said his LOW band "only gates the crossing" and that inside the last 8 cm the descent
        "just keeps lowering": WRONG. At bf52a8a TieredDescent already paused the height inside LOW while the tip was over
        pickup_alignment_low_xy_m (pause_low_for_xy True); on our rig that is 0.10 (partner_mission.launch.py, after 0.045
        stalled HIGH on 14 Oct t5_5) with low_dwell_s 1.0 at the crossing. So the 4.7-9.7 cm arrivals were inside the
        pause. The LOW hold (pickup_low_hold_xy 0.04) narrows that in-LOW pause only and keeps the entry at 0.10 / 1.0 s:
        the same effect at contact as Te-Jen's single band (his overlay 0.025), without tightening the crossing that
        stalled. No code change. (2) "fresh measured vertical separation; retry" is his RECOVERY line, not the failure.
        The 11 Nov failed attempts' triggers: "contact wait made no progress for 10 s" (fm9c #1, fm9c_3 #1, fm9c_4 #1),
        "object attached but the tip 0.084-0.085 m off target" (fm9c_3 #2, fm9c_4 #2), "measured pickup proof separation"
        (fm9c_6, the big object falling in the lift); fm9c #2 and fm9c_3 #3 ended in Wesley's LAND. Counts unchanged.
        (3) He confirms his pose timeout reads arrival times, finds no bound on the pickup's place in his Python retry
        path (the pickup box fills a real gap), and has no online mass estimate (kT adapts slowly; unloading plausible
        but not the sole cause of fm9c_6's climb). (4) The moving-drop gate is target-relative: fm9c_4's transit had a
        median magnet error 28.5 cm vs 18 allowed, so a speed change alone would not pass it; he advises against
        drop_on_arrival (it skips the lowering): the sheet now says FORCE DROP, not DROP=arrival.
DECIDED 2026-11-11 21:30  (Wesley, for Te-Jen's individual paper, asked with options) Three simulation application demos,
        5 trials each (card docs/experiments/2026-11-11_tejen_app_demos.md; configs tejen_app_{hover,circle,fig8}_twin):
        QUAD1-3 on 3/11/7 lift the ring to 1.2 m, QUAD4 from the FLOOR picks up, drops into the net, attaches on 9;
        no pause of the ring's motion; LAND 10 s after the attach. Circle 0.3 m and figure-eight 1.0 m, both at 0.2 m/s
        (Wesley: "1 m, to make it nice and clean ... 0.2 m/s for all the trajectories"; 0.125 was only the rig's cage
        speed). All 15 on the lab PC tonight (R4254-R4268), videos of each case's first trial, Wesley notified when the
        first of each is back. Reported as simulation, not real-world validation.
DECIDED 2026-11-11 21:42  (Wesley, asked with options) TB16 lifted in the TWIN ONLY for Te-Jen's figure-eight demo: a new
        sim.yaml switch contact_sim_fig8 (default false; twin only; the rig refusal unchanged) lets partner_contact fly
        load_traj fig_8, typed only in configs/experiments/tejen_app_fig8_twin.yaml. Said with it: the take-back clears
        hover and the circle only, so a figure-eight run that fails before the attach may land QUAD4 badly. Test
        test_tb16_the_twin_flies_the_figure_eight_only_with_its_switch. The fig8 slots of the 21:36 batch (R4256, R4259,
        R4262, R4265, R4268) were refused at launch (void); the 5 trials re-fly after the batch.
FINDING 2026-11-11 21:53  Te-Jen's second review (source-level, 11 Nov evening), checked against the code; FIXED, unit-tested,
        gate --quick PASSED (2218) 21:51, rig desk check 13 OK 21:52, not flown: (1) FORCE DROP stepped the reference:
        a press in the lowering jumped to the release height (his example 0.25 -> 0.03 m), a press in the transit
        jumped from the transfer reference to the nominal start over the drop point (up to the 0.25 m reach). Now the
        release holds where the lowering had got to, and the lowering starts from the commanded offset; a pending
        press dies with its phases. (2) A TAKEOFF held until our release survived DISARM: now cancelled by DISARM, LAND
        and a take-back. (3) The twin's C++ re-attach branch dropped the typed drop_release and pickup_box: kept now.
        (4) pickup_point_x / _y alone were silently ignored: refused now. (5) The magnet gate's held OFF does reach the
        coil through the phase subscription (node-level test added); on the rig it has not yet been exercised (fm9hc_8:
        his OFF came after DROP_OBJECT). Corrections in CHANGES_FROM_TEJEN.md: the drop check gives up ~2.4 s after
        DROP_OBJECT (2 s after the 0.4 s dwell); the pickup box rejects the target, it is not a geofence; his off-target
        retry is pickup_lift_xy_tolerance 0.04 m held 1.5 s. For Wesley to send: ~/Downloads/tejen_fork_changes_2026-11-11b.zip
        (with tejen_mpc/main.py and throttle_bound.py, unchanged since bf52a8a, as Te-Jen asked). Te-Jen's demo runs: the
        hover and circle trials (R4254-R4267) fly the tree before these fixes, the figure-eight re-fly the tree after;
        none of the fixed paths (FORCE DROP, the held TAKEOFF, the C++ re-attach, the sideways offsets) is used in them.
DECIDED 2026-11-11 22:08  (Wesley, after R4255: on the ring circling at 0.2 m/s QUAD4's tip trailed plate 9 by a steady 5 cm
        on the 5 cm descent band, and the twin gave up at his 30 s) Te-Jen's moving cases keep 0.2 m/s with the wider
        bands 0.08/0.10 (the rig's until 11 Nov) and the give-up 120 s as the rig; the batch stopped; ONE circle and ONE
        figure-eight first, the repeats only after Wesley has seen them. Hover stays 0.05/0.05 (3 of 3 passed: R4254,
        R4257, R4260). Twin parity fix: partner_mission.launch.py now passes a typed contact_give_up_s in the twin too
        (it was rig-only, silently ignored; test test_the_twin_takes_a_typed_give_up_time).
FINDING 2026-11-11 22:28  Te-Jen's third review (package 11b): his checks confirm the FORCE DROP continuity, the DISARM cancel
        of a held TAKEOFF, the twin overrides, the sideways-offset refusal and the gate's held OFF. Two more, done (tests;
        gate --quick PASSED 2219 at 22:27, desk 13 OK 22:27): a pending FORCE DROP is cleared by DISARM itself (DISARM does
        not change his phase); set_throttle_state_max keeps the solver's own state bounds and replaces only the throttle
        entry (no change today: the state bounds are +-1 in every profile; his "oct14 +-3" are input bounds). His notes,
        no change: the flown MPC has no immediate gain or mass reset when the object falls off (the attachment-based
        schedule is ignored while thrust-ratio feedback is on), so adaptation lag is a plausible contributor to fm9c_6's
        climb, not the whole explanation; battery compensation on the radio command only is a deliberate command-domain
        choice. The tracking-validity (frozen pose) limitation stays explicit.
DECIDED 2026-11-11 22:38  (Wesley, after R4269 / R4270: with the 0.08/0.10 bands and the 120 s give-up both moving cases at
        0.2 m/s still missed the attach; QUAD4 trails the moving plate by 5-10 cm) Keep 0.2 m/s and raise his attach lead
        time: contact_lead 0.4 s (his reference leads the moving target in ATTACH_READY, the bottom hold; our launch had
        it at 0, his own default is 0.2). One circle and one figure-eight first (R4271, R4272). WAIT_WELD 280 s in both
        configs so a failed run ends sooner. Every moving-ring attach that passed in the twin so far (R4228, R4231,
        R4247) was at 0.125 m/s with the ring paused for the attach.
FINDING 2026-11-11 23:05  Te-Jen's demo at 0.2 m/s: the attach onto the moving ring failed in every trial (R4255, R4258 bands
        0.05; R4269, R4270 bands 0.08/0.10 and give-up 120 s; R4271, R4272 plus lead 0.4 s): QUAD4 trails the plate 5-10
        cm and his descent loses alignment. The pickup and the drop into the moving basket worked in every one (ball
        2.2-2.4 cm from the centre at 0.19 m/s). R4272 (figure-eight): after an aborted descent QUAD4 sat 0.84 m over the
        ring at the figure-eight's crossing (the ring slowing to rest) and its tether struck the formation (ring 24-36
        deg), later QUAD4 disarmed and the ring passed 55 deg: DROP AND LAND. A rig flight of this line would collide.
DECIDED 2026-11-11 23:06  (Wesley) All 5 of each at 0.125 m/s overnight (one variable from R4271/R4272: the speed; bands
        0.08/0.10, give-up 120 s, lead 0.4 s kept) plus the 2 hover repeats; send him the first circle and figure-eight
        results (figures and videos) when they are back. Batch ~/mdc_overnight/tejen_app4.sh, started by
        tejen_app4_after.sh when R4271's batch ends.
DECIDED 2026-11-12 00:24  (overnight, under Wesley's 11 Nov 23:50 instruction "experiment with the best available option",
        not his word on this choice) Te-Jen's three demos are flown on 0.55 m CABLES, October's m1c_orbit setup (QUAD4 from
        the floor, his own descent defaults, lead 0, the ring at 1.0 m, circle 0.5 m at 0.125 m/s, figure-eight 1.0 m at
        0.125 m/s, no pause), 5 trials each (configs tejen_app_{hover,circle,fig8}_L055_twin; R4288 = circle trial 1).
        Why: on 1 m cables no setting tried tonight attached on a moving ring without a pause (R4255, R4258, R4269-R4274,
        R4285-R4287, R4289: bands 0.05 and 0.08/0.10, give-up 30 and 120 s, lead 0 / 0.4 / 1.0 s, his defaults; at the
        bottom hold the magnet trails plate 9 by 5-6 cm at 0.125 m/s, ~10 cm at 0.2, a ~0.9 s lag of QUAD4 and its
        1 m tether); on 0.55 m R4288 attached on the moving ring on tonight's tree (as R1508, R1700 in October). The 1 m
        hover set (5 of 5: R4254, R4257, R4260, R4275, R4276) stays as the 1 m result. For Wesley in the morning: the
        cable length differs from the rig's 1 m; the alternative on 1 m is the attach with the ring paused (R4228,
        R4231, R4247), which his brief excluded.
DECIDED 2026-11-12 01:38  (overnight, under Wesley's "experiment with the best available option") Te-Jen's figure-eight case
        flies at 0.08 m/s with the rig's looser descent limits (match 0.08 / 0.5 s / 0.20 m/s, abort 0.10 m and 0.35 m/s
        held 1 s, descent hold) and the 120 s give-up, on 0.55 m cables (config tejen_app_fig8_L055_slow_twin): R4306
        attached on the moving figure-eight (8th descent, ring at 0.071 m/s). At 0.125 m/s on 0.55 m it did not attach
        with his defaults (R4290, R4293: descents aborted on the turns, 30 s give-up) nor with the looser limits and
        120 s (R4305: every descent aborted on the tip's speed relative to the plate). Hover and circle keep October's
        set (his defaults, 0.125 m/s): circle 5 of 5 (R4288, R4292, R4295, R4298, R4301), hover 4 of 4 so far. Trials 2-5
        of the figure-eight: laptop R4307, lab PC after the batch.
FACT    2026-11-12 09:01  Re-measured the object's seat before the next fm9 (Wesley: the object on the same floor; its marker read
        60 mm against 41 mm last night, tilt 3 deg): magnet seated at +10.7 mm over body 6's marker (typed 11), sideways
        (-4.5, -10.7) mm in its axes (typed the mean -9.0, -9.8; last night's second seat -4.9, -10.7). The relative
        geometry holds, so the height did not stop RIG-1112-fm9_2's pickup. Scripts (repo + ~/rig_*.sh) now type
        PX -0.0045, PY -0.0107. fm9_2: attempt 1 reached the seat height (+11.4 mm) with the coil ON but 1.8-3 cm off
        and the object did not lift ("measured pickup proof separation"); attempt 2 stopped 4 cm up (the funnel's rim
        height), "no progress for 10 s".
DECIDED 2026-11-12 09:24  (Wesley, in the lab) The rig's position limit 1.75 -> 1.8 m (real.yaml pos_limit_m; tests and the
        working rules updated; desk: the circle launch carries 1.8 on all 6 nodes). Freed or dropped drones still land
        0.10 m inside it (1.7 m).
FINDING 2026-11-12 09:24  QUAD4's magnet slides off plate 9 right after TAKEOFF in every flight today (fm9_2 16 mm at +3.0 s;
        fm9_5 32 mm at +1.0 s, back to 12 mm; fm9c 23 mm at +1.4 s, stayed 21 mm off, Wesley LANDed): outward and ~1 cm
        up (the funnel's slope). Our commands kept it ON throughout (/drone_3/magnet_coil ON at 2 Hz, the radio latched
        ON from boot, no OFF anywhere); QUAD4's link and pack normal. Wesley: it looks like the magnet turns off briefly,
        and its crimp was loose: being re-crimped now. Second suspect if it persists: the funnel holding the magnet off
        the steel.
FINDING 2026-11-12 09:46  Why the pickups "attached but did not confirm" (RIG-1112-fm9c_3 both attempts): his planner leaves
        MAGNET_ATTACH_WAIT only while his IRL manager reports "attached" (inferred: magnet within attach_radius_m 0.12 of
        the object's contact point, <0.25 m/s, 0.8 s). On the way down the swinging magnet met that 12 cm sphere for
        0.8 s: ASSUMED ATTACHED ~19 s before contact; 0.5-1.1 s later it swung out for 0.2 s: ASSUMED ATTACHMENT LOST,
        latched until a magnet OFF. Then the magnet seated (9 and 13 mm from the seat, 0 mm in height, object still),
        but the latch held, "contact wait made no progress for 10 s", magnet OFF, the object dropped. Built (typed only,
        default off, his gate threshold so Wesley's word): launch arg pickup_attach_radius -> his attach_radius_m; script
        switch ARAD (ARAD=0.04: only a seated magnet reads attached). Test passes; desk: 0.04 typed, 0.12 untyped.
BUILT   2026-11-12 10:12  The pickup press (Wesley ~10:00, after RIG-1112-fm9c_5: "once it was at the objects height if it went
        very slightly lower before confirming the attach that would be better"). In fm9c_5 (ARAD=0.04) the attach was
        confirmed with the magnet at the seat height but ~1.4 cm off-centre and it did not grip (the object never rose in
        the lift). Built (typed only, default off): launch arg pickup_press -> his pickup_candidate_press_m (0-0.05 m): at
        the early pickup contact the hold is that much below the last reference, so the cable slackens and the magnet
        settles flat. Script switch PRESS (PRESS=0.02). Tests pass; desk: fm9c with PRESS=0.02 ARAD=0.04 OK (26 nodes).
        Untested in flight and in simulation (this pickup path is rig only).
DECIDED 2026-11-12 12:12  After Te-Jen's review of the 12 Nov logs (Wesley ~12:00, all four recommended): (1) his pickup
        attach radius 4 cm is the DEFAULT in the pickup scripts (rig_fm9, rig_fm9c, rig_k1zo, rig_k1z; ARAD=0.12 = his);
        the held-object starts (fm9h, fm9hc) and the re-attach-only scripts keep his 0.12 untyped. Te-Jen: keep the
        loss latch and the proof; fm9c_5's two lifts were real non-lifts (QUAD4 up 8.9 / 8.7 cm, the object 0.3 / 1.3 mm,
        checked from the 50 Hz extract). (2) The pickup press rebuilt as a ramp (0.02 m/s, at most PRESS, never below the
        descent's own lowest contact target), still typed only; FIRST a props-off hand test by Wesley (coil ON, magnet
        lowered through the object's funnel by hand, lift: does it grip?). (3) attach_twist: every rig script typed
        zero (meant for a still plate); RIG-1112-nro flew it on the moving ring. rig_nro.sh PAUSE=off now gives his planner
        the plate's measured twist; the paused flights keep zero. Simulation: 11 of the 12 scored moving 1 m runs also flew
        zero (R4255, R4258, R4269-R4274, R4285, R4286, R4289; only R4287, his defaults, flew raw, no contact), so "1 m does
        not attach while moving" is not shown yet: R4311/R4312 (lab PC) re-fly the 1 m circle at 0.125 m/s with the raw twist.
        (4) A reply to Te-Jen emailed to Wesley to forward.
DECIDED 2026-11-12 12:45  The 13 Nov rig trials (Wesley and Te-Jen ~12:30; card docs/experiments/2026-11-12_t13_trials.md):
        hover, circle, figure-eight x 3 trials, QUAD4 starting off the ring holding the object at 9, 1 and 5 o'clock,
        the ring stopping for his re-attach on 9 (moving re-attach not reliable). Order BY EXPERIMENT (hover x3, circle x3,
        figure-eight x3); ONE COMMAND per experiment whatever the start ("it should be able to drop off and attach from
        anywhere with obstacle avoidance"). Wesley ~12:45: (a) the figure-eight on the rig behind a typed switch
        (contact_rig_fig8; it also lets traj_laps through: one 0.75 m lap is ~30 s) + a twin check, flies only if clean;
        (b) all 9 trials simulated tonight (new worlds: QUAD4 1.9 m out at 9/1/5 o'clock, tools/make_held_start_worlds.py);
        (c) a take-back on the figure-eight holds the path at rest until LAND (dissipative_node) -- WITHDRAWN ~14:00 (Wesley,
        after the critic: its evidence R4305 was wrong; R4299 clean; the take-back frame assumes full speed): the
        figure-eight's take-back is the circle's; sheet rule: anything but a weld on the figure-eight, LAND at once. OPEN for Te-Jen (message
        emailed 12:56): his planner has no obstacle avoidance; our 4 Nov line-up from beyond 1.2 m sends QUAD4 to plate
        9's line first, which from 1 or 5 o'clock passes ~9 cm (1.4 m start) to ~2 cm (2.1 m start, today's placement)
        from the carrier on 11 or 7. Built behind line_up:=gap (script switch LINEUP=gap, default off): the drop transit
        lines up on the NEAREST carrier gap (from 9/1/5 straight in, 0.64 m from every carrier). Until decided, a 1 or 5
        o'clock start needs LINEUP=gap or QUAD4 within ~1.0 m of the ring centre.
DECIDED 2026-11-12 14:00  After the critic of the 13 Nov card (GO WITH CHANGES; card ## Critic): the figure-eight hold
        withdrawn (above); placement rules on the sheet (Wesley): QUAD4 1.9 m from the ring's rest point at every start
        (2.1 m at 1/5 o'clock is past the 1.8 m limit in x), the ring's rest within |x| <= 0.3 m and y in [-0.6, +0.1] m,
        1 and 5 o'clock only with LINEUP=gap (pending Te-Jen and the twin).
CORRECTION 2026-11-12 14:05  (Te-Jen's reply, checked in src/tejen_mission/tejen_mission/ring_bridge.py:203-214, unchanged
        since 284ffb4, 29 Oct) attach_twist:=zero zeroes the attachment twist only while the ring is NOT moving ("a
        moving ring needs its velocity"). So RIG-1112-nro and every moving twin run of the Te-Jen demos (R4255-R4289)
        published the plate's measured twist despite the typed zero; DECIDED 12:12 point (3) was wrong, and R4311/R4312
        repeat R4285 (measured twist, lead 0). The PAUSE=off twist change to rig_nro.sh is reverted. The measured twist is
        his base class's one-step 50 Hz finite difference of the plate position (fake_cooperative_transport_world.py:780,
        unfiltered); Te-Jen sees the reference horizon jump between packets and will inspect it.
FACT    2026-11-12 14:05  (Te-Jen) His stack has obstacle avoidance: the C++ planner (moving carriers through their advertised
        trajectories; the carrier tethers with carrier_model:=tether). Our rig and twin scripts bypass it (cpp_authority:=false,
        attach_cpp_transit:=false). Describe every combined trial as "the direct Python transfer, obstacle avoidance
        bypassed"; the nearest-gap line-up is a prescribed routing heuristic, not a validation of his avoiding planner.
        Te-Jen favours option A (line_up:=gap) for tomorrow's three starts, the gap kept for the whole approach (built).
NOTE    2026-11-12 14:15  The placement rule tightened on the sheet (docs/rig/fri_2026-11-13.md): with QUAD4 1.9 m out at 1
        or 5 o'clock its x is the ring's rest x +- 1.65 m, so the ring's rest x must be within +-0.1 m (not 0.3) or QUAD4
        starts past the 1.8 m limit (the LAND at takeoff of RIG-1112-fm9c). Real clearances on 12 Nov (motion capture,
        QUAD4's cable as body to tip marker): route in from 9 o'clock 0.13-0.34 m to a carrier's cable, drop to attach
        0.15-0.20 m; the twin read 0.38-0.47 and 0.21-0.23 on the same legs (optimistic by ~0.1-0.25 m).
OPEN    2026-11-12 14:58  Te-Jen (~14:45, and at the machine ~14:55): the 13 Nov experiments must use his C++ obstacle-avoiding
        planner for the transit, above all carrying the object; he was not aware that every flight since RIG-1029-sq3/
        -sq4_5 (29 Oct, Wesley ~10:20: his hand-over never granted, then his backend below its search floor) flew
        cpp_authority:=false. The direct-route runs are "pickup/drop integration tests", not planner experiments. Wesley
        to confirm tomorrow's plan. Built: CPP=on in rig_fm9h/fm9hc/fm9h8.sh (no cpp_authority:=false, no line-up;
        carrier_model tether, the default; desk OK); his C++ re-attach transit (pickup_cpp_reattach) is wired only in his
        launch's twin path, so on the rig the hop to plate 9 stays Python. Twin validation running (12 runs: C++ loaded
        transit from 9/1/5 on hover/circle/figure-eight; 3 with the C++ re-attach), ownership read from his
        "authority source switched ... -> C++ at TRANSIT_TO_DROP_POINT entry" line; backend diagnostics if it stalls.
BUILT   2026-11-12 15:30  rig_q4d.sh (Wesley ~15:10: a one-quad test outside 08:00-11:00, "Quad4 pickup the object and dropoff
        into the ring ... see if we can make it more accurate"): QUAD4 alone, his M1 ground-ring stack (6 Nov's
        q4f_ground_ring.launch.py, src code, no overlay workspace), mission pickup_delivery: his pickup of body 6 (12 Nov's
        seat with the funnel, ARAD 0.04, the rig's pickup values), the loaded transit on HIS C++ planner (c1f2b, as his stack
        ships), the settle drop into ring 8 resting on the floor (gate DXY 0.05 = today's match band, release 0.03), then his
        ground-ring re-attach to plate 9 (q4f's). LAND routes back to the pad only from TRANSIT_TO_REATTACH on; earlier it
        lands straight down. No position limit in this stack. Desk OK (ids, radius, mission, C++ authority). Untested in
        flight in this form.
FACT    2026-11-12 15:49  Why q4d_3 hung 26 s over the ring (RIG-1112-q4d_3): his drop settle gate is Python's, not C++'s
        (tip within drop_settle_xy_threshold, under match_xy_velocity_threshold, for 1 s). On the rig's direct route it
        defaults to irl_commissioning's 0.18 m / 0.20 m/s and passed in 1.0-1.1 s with the carriers' wash and the tip
        4-10 cm off (fm9hc_2/_4/_5). rig_q4d.sh had typed 0.05 (wrongly called "today's") and his ground-ring file 0.08 m/s:
        the tip (2.9-15 cm, median 7.2) met it 18 % of the time. rig_q4d.sh now types the rig's 0.18 / 0.20 and the rig
        path's hand-over bounds 0.30 m / 0.12 m/s (c1f2_prepare_*); the rig scripts with CPP=on already carry the same
        (desk 15:49). The accuracy levers are the release (height DZ/RELEASE, and a release gate on the swing: not built).
OPEN    2026-11-12 16:05  (Te-Jen, for Wesley to decide when he plans 13 Nov; nothing changed) Aim for ONE trial each tomorrow:
        1 hover, 1 circle, 1 figure-eight. Hover and circle work with the direct route; tomorrow they swap it for his C++
        planner (CPP=on). The figure-eight last, as the most uncertain. Wesley writes the plan Te-Jen follows while Wesley
        is away.
FACT    2026-11-12 16:37  His C++ planner flying the loaded transit with three live carriers, twin (R4338-R4349, the final
        tree, CPP=on settings): C++ took the transit in 12 of 12, reached the drop in 0 of 12. Hover: stalled 0.66-0.89 m
        from the ring centre (at the carriers). Circle and figure-eight: some passed over the centre at 3.1-3.6 m (rig cap
        2.8) but never captured; R4339 (circle, 1 o'clock): QUAD4 tilt 40-43 deg x4, then 75 deg, envelope fault, disarmed.
        Backend: LIVE_TARGET_UPDATE_PENDING / LIVE_TARGET_RETARGETED, local plans failing the padded final check
        (LOCAL_PLAN_FAILED:PADDED_CLOSEST_PARTIAL_FINAL_CHECK_FAILED) against the carriers' attached tethers and bodies
        vs QUAD4's cable, payload, magnet. The direct route passes at 0.2-0.4 m cable-to-cable (twin), 0.13-0.34 m (rig).
        One quad on the rig today (no carriers, q4d_3/_4): his C++ loaded transit worked twice. Diagnostics for Te-Jen:
        ~/Downloads/tejen_cpp_twin_2026-11-12.zip. For Wesley's 13 Nov plan (OPEN 16:05): C++ with carriers is not shown
        to work in the twin.
OPEN    2026-11-12 16:55  (Te-Jen, for Wesley) Tomorrow is C++ or nothing: "if we don't run C++ tomorrow, we are not running at
        all ... the combined paper depends on c++ working" (the direct route = integration tests only). His values for the
        twin batch: swing bound sweep 3, 5, 7 deg; spline_time_factor 4 (his 3.0 now); workspace limits x,y +-1.8 m, z 0-2.9
        m (proposed by the lab chat). Needs two typed-only switches in partner_mission.launch.py (spline_time_factor,
        workspace) plus max_swing_angle_rad (exists). Te-Jen: WAIT FOR WESLEY before building or running anything.
FINDING 2026-11-12 17:04  Our copy of Te-Jen's code is his 21 Oct commit fd495db0 (sync 2, CHANGES_FROM_TEJEN.md); his branch
        SwungPayload2026_ThesisCExperiments is 143 commits ahead (09fef0b7, 11 Nov; fetched read-only into
        ~/tejen/drone_cage_control), ~20 in the C++ planner incl. convex swing bounds, stopped-partial soft goals, the
        stationary-timeout deadlock, finish timing, ~200 lines in ego_collision_model. The twin C++ stalls (R4338-R4349)
        are planner-side (backend healthy: goal right, inputs fresh 3/3, tracking ~1 cm, plans fail the padded check vs the
        carriers' tethers), not an adapter freeze. Also: our rig overrides swing 15 deg (his 10), object box 0.043 half
        (object 105 mm); his moving-target prediction horizon 0 s (config or our bridge); tether geometry vs mocap never
        checked; his MPC "230 ms" loop in the twin is wall-clock vs sim time (RTF ~0.15), harmless. OPEN for Wesley (emailed):
        sync 3 tonight, gate, re-fly the 12 C++ twin cases, then Te-Jen's sweep.
DECIDED 2026-11-12 17:40  (Wesley, relayed by Te-Jen ~17:15: "Wesley approves the plan") Sync 3 done in place (his 09fef0b7;
        CHANGES_FROM_TEJEN.md; backup ~/mdc_sync3_backup_*): gate 2225 passed. Twin batches on the lab PC: (1) the 12 C++
        cases + 2 direct-route checks on the merged code, unchanged settings (17:38); (2) then the collision geometry
        measured on the rig (tuned: swing 5 deg, spline_time_factor 4, carrier inflation 0.06 m, workspace +-1.8 m, z 0-2.9
        m) on hover 9/1/5 and circle 9, and the swing sweep 3 / 7 deg on hover 9 and circle 9. New typed switches:
        spline_time_factor, workspace_xy/_z_max; his backend's object box on the rig from the typed object sizes. Carrier
        deviation from his rigid prediction, 25 rig flights 4-12 Nov: 95th percentile 3-6 cm horizontal (max 6-10; sq8_2
        11/14), 2-7 cm vertical.
FINDING 2026-11-12 20:27  His workspace bounds (workspace_bounds_enabled; our typed workspace_xy/_z_max) apply to the whole
        assembly (body, cable swing envelope, object, tracking): at +-1.8 m the body is held to roughly +-1.6 m. R4366-R4373
        (twin, QUAD4 starting 1.9 m out): 0 plans accepted, the hand-over never happened (VOID). On the rig, the 13 Nov
        starts at 1 and 5 o'clock (x +-1.65 m) would be outside it too. Done: CPP=on in rig_fm9h/hc/h8.sh types the workspace
        only with WSXY (opt-in); the rig's pos_limit_m 1.8 (body) stays the hard limit. The tuned set re-flown without the
        workspace (R4374+). For Te-Jen/Wesley: start QUAD4 within ~1.4 m in x and y, or a workspace of ~2.0 m.
FINDING 2026-11-12 21:45  His C++ transit with carriers, twin, sync-3 code, tuned geometry without the workspace: CIRCLE from 9
        o'clock 2 of 2 (R4377, R4382: transit 86 / 76 s, max 3.5 / 3.9 m, QUAD4 cable 0.09-0.10 m and the object 0.07 m
        from a carrier's cable, centre lines). HOVER 0 of 8 (R4374-R4376, R4378-R4381): QUAD4 stops 0.50-0.67 m from the ring
        centre whatever the swing (3/5 deg), carrier inflation (0.04/0.06), body box (0.09/0.105) or spline time (4/6):
        not the padding; it never enters his capture region over a still ring (backend LIVE_TARGET_UPDATE_PENDING; plans
        still accepted). For Te-Jen (diagnostics package); for Wesley's 13 Nov plan.
- FINDING 2026-11-12 22:26 (lab chat; corrects "not the padding" in the 21:45 entry): the cause of his C++ hover stall
        is his collision shapes, not the padding we swept. His default `swing_envelope_shape: legacy_square` pairs every
        carrier component with every ego component as axis-aligned boxes. A carrier's 1 m tether runs at a slant (body
        r 0.73 m, z 2.12 to plate r 0.225, z 1.20), so its box is ~0.45 x 0.26 x 0.9 m. Those of carriers 7 and 11 overlap
        over the ring once the ego cable/magnet/object boxes are added, so the drop point lies INSIDE his obstacle set. A
        probe built on his own core (WorldSnapshot::timeIndexedObstacles + Separator) at R4374's stall puts the free edge
        at x 0 at y 0.675-0.70; QUAD4 stopped at 0.668, and his replans fail PREFIX_CORRIDOR_INCOMPATIBLE 74-83 % (the
        prefix on the hull's face). His convex mode `outer_octagon` keeps the real slanted prism: the route in and the drop
        point are free at swing 5 deg, with a narrow band between carriers 7 and 11 (x +-0.05 m free, +-0.10 m blocked). At
        10 deg even octagon closes it. Same with the rig's object and cable. Built: typed-only launch arg
        `swing_envelope` (CHANGES_FROM_TEJEN.md). Twin R4383-R4387 (hover 9/1/5, circle 9, fig8 9; swing 5, spline 4,
        inflation 0.06, no workspace, outer_octagon) running on the lab PC. Diagnostics for Te-Jen:
        ~/Downloads/tejen_hover_stall_2026-11-12/ (README, probe).
- DECIDED 2026-11-12 22:52 (supervisor, Teams chat with Wesley and Te-Jen 21:56-22:57, pasted by Wesley): the combined
        paper's rig experiments, replacing the 9-trial plan (3 starts x hover/circle/fig8: "the exact same case", no
        repeatability result). 1 or 3 IDENTICAL runs per case:
        (1) Te-Jen's drop-off: QUAD4 starts holding the object, his planner flies in, drops into the moving ring, flies BACK
            TO ITS START (start -> ring -> start), all while the ring moves; hover, circle, fig8; no pickup, no re-attach.
            Metrics: path, distance to the other drones, drop position from the ring centre, time.
        (2) Wesley's detach/re-attach: detach while moving (hover, circle, fig8), the ring hovers for the re-attach, same
            plate. Metrics: time, re-attach accuracy, ring tilt, QUAD4's path from 3 to 4.
        (3) 4 -> 2 -> 4 (one run each: two together or in succession, then back in succession); a limit test.
        (4) Hero (video): detach, pickup (the object raised on Thomas' stand, about ring height), drop, re-attach; if not,
            start with the object held. Pickup only in the hero.
        Extra metric next week: current draw per drone with 4 and 3 carrying (Betaflight telemetry).
        Order for Fri 13 Nov: fig8 drop-off and fig8 detach/re-attach first, then the hero; 4 -> 2 -> 4 next week.
        Lab chat's check of what is flown on the rig (registry): detach/re-attach in hover yes (RIG-1111-nrtfc_2, _5);
        the hero in hover yes once (RIG-1112-fm9_5, 3 pickup descents); detach/re-attach on the circle with the ring
        stopped NOT caught yet (RIG-1105-nr9o_2, no funnel then); no drop-off flown by his C++ planner, and none flew back
        to its start (fm9hc_2/_4/_5 used the direct Python route and re-attached).
- DECIDED 2026-11-13 00:25 (Wesley, ~00:15: "this is bad, the stand is narrow and this could cause a crash"): for the hero with
        the object on Thomas' stand, a take-back must never come down on the stand. Built (dissipative_node
        `take_back_avoid_pickup_m`, default 0 = off; the hero's params file
        `configs/rig/attach_m1_1m_m2a_drop55_det15_stand.yaml` sets 0.6; knob in common.yaml / real_mode PLANNER_OPTIONAL):
        QUAD4's position is latched as his DESCEND_TO_PICKUP starts. A take-back (our LAND, his give-up) whose landing
        point falls within 0.6 m of it first rises to that height (the magnet clear of the object), then moves sideways
        to the nearest point 0.6 m away (inside the position limit's margin, clear of the ring), then comes down. His
        own LAND still lands straight down: the sheet says our LAND only during the pickup. Unit tests in
        test_partner_contact.py; twin check rig_fm9_stand115_landpick_twin queued. Critic not run (a take-back path
        change, not a control law or gate); Wesley can ask for one.
- FINDING 2026-11-13 01:15 (lab chat; twin): (1) The take-back's stand avoidance did not engage in R4410: with his ball on, the
        take-back's ring clearance includes the rod, which rejected every point round the stand; fixed (the plain ring
        clearance; test with the ball on), re-flown in batch 4. (2) The C++ return's hand-over in the CLEAR_DROP_ZONE hold sinks
        ~0.3 m onto his 2.20 m search floor (R4406, R4407: WAITING_ABOVE_SEARCH_FLOOR); typed return_climb:=transit (climb back
        to the approach height first, as his pickup_cpp_reattach) in batch 4; on the rig it needs max_body_z 3.0 (asked).
        (3) His search box reaches 1 m above start/goal: he flew 2.9-3.6 m in the twin, which the rig's 2.8 m planned-height
        check does not see, and his workspace box stalls his hand-overs (R4401, R4408). Built typed cpp_max_z (his
        maximum_search_z_m); batch 4 flies the sheet's drop-offs with it at 2.9 m. Row 1 of the 13 Nov sheet flies
        CPP=on RET=direct (Wesley ~00:55) unless the twin shows the C++ return working.
- DONE 2026-11-13 01:15 (Wesley ~01:00, the laptop disk full): results/2026-11-01, -03, -04, -05, -06, -07 (simulation days)
        moved to the lab PC, drones:~/mdc_archive/results/<day>/, each verified by a checksum dry run before the laptop copy was
        deleted (in progress at 01:15: 2026-11-06 done). Registry run ids unchanged; the files live there now.
- DONE 2026-11-13 01:40: the move above completed for all six days (2026-11-01, -03, -04, -05, -06, -07), each checksum-verified
        on the lab PC (drones:~/mdc_archive/results/<day>/) before its laptop copy was deleted; the laptop has 30 GB free.
- FINDING 2026-11-13 02:06 (agent; real-time open-loop replays of the 12 Nov flights into his backend on this laptop, outer_octagon,
        tomorrow's settings): his planner reached the drop point in 0 of 4. Every carrier state / plate / ring pose message
        changed his collision world (~214 versions/s; plate_from_body moves with the measured samples), and his 60-90 ms final
        recheck near the tethers ended WORLD_CHANGED_DURING_FINAL_RECHECK. The twin hid this: it runs at 0.13-0.16 of real time.
        With the refreshes coalesced to 0.1 or 0.2 s (new typed backend parameter cooperative_world_refresh_period_s; his Codex
        entries 8-9: a non-default discriminator, not yet contract-preserving) he reached it in 4 of 4, but on the circle only
        after the recorded ring had slowed; heights 3.0-3.5 m; and in fm9hc_4 at 0.2 s QUAD4 held still over the circle's path
        while the carriers swept under it: the hanging line came within 0.05 m of a carrier body (his fallback brakes to a stop,
        which does not leave a moving peer's path; the twin's R4388 again). The nro replay (95 s of circling) is next.
        In the twin his search box capped at 2.9 m found no way in over the moving figure-eight or circle (R4411, R4412); the C++
        return cannot start over a moving ring (R4413), and works over a hovering ring with return_climb transit (R4414).
- DECIDED 2026-11-13 ~01:30 (Wesley, before bed): over a moving ring his planner is too high; lower it if at all possible
        (edit his planner, consult his Codex), otherwise a hard cap at 3.8 m as long as he really approaches; he never wants to
        see it fail to plan; a worst-case backup must be ready; the ring speed stays. Done so far: CPP=on in rig_fm9h/hc/h8 now
        also types cpp_max_z 3.8 (TOP=) and cpp_world_refresh 0.2 (WREF=0 turns it off); RET now works without CPP (the
        worst-case backup RET=direct bash ~/rig_fm9h8.sh: his Python transfer both ways, as 12 Nov). Desk checks OK.
- FINDING 2026-11-13 02:22 (agent; real-time open-loop replays into his backend, binary v2, tomorrow's CPP=on line: outer_octagon,
        swing 5, spline 4, tracking 0.06, 30 steps, refresh 0.2): over a STILL ring his planner reached the drop point in every
        run: hover fm9_5 3/3 (15.5-17.6 s after authority, peak 3.10-3.20 m, no stranding) and the still-ring segments of
        fm9hc_2/_4/_5 3/3 (13.2-17.2 s, peak <= 3.03 m). With his search box capped at 2.9 m it still captured (fm9_5 17.9 s,
        fm9hc_4 still 15.6 s) at 2.85-2.86 m. Temporal refinement did not lower him and made hover worse (43 fallbacks, 24 s
        beside a tether): dropped. Over a CONTINUOUSLY circling ring (nro, 90 s) he never captured, period 0 or 0.2. Basis for
        the morning proposal: his drop-off over a ring paused at his TAKEOFF (PAUSE=takeoff, with attach_pause_through_landing
        so his LAND cannot restart the ring over him; Te-Jen's Codex entry 11). Repeats of the 2.9 m cap and a CPU-load test
        follow.
- DECIDED 2026-11-13 02:29 (lab chat, within Wesley's ~01:30 instruction: as low as possible, never above 3.8 m): CPP=on now types
        cpp_max_z 2.9 by default (TOP=3.8 for his ceiling): real-time replays over a still ring with the 2.9 line reached the
        drop point 8 of 8 (4 hover, 4 still-ring segments), 12.9-21.5 s, QUAD4 never above 2.89 m (the 3.8 line: 6 of 6,
        2.80-3.20 m). His replans run ~105 ms median against his 0.1 s splice (LATE 41-64 %); 4 and 8 busy cores did not
        degrade the 3.8 line. RET=cpp types return_climb transit and max_body_z 3.0. Over a moving ring his planner is not
        proposed at all (continuous-circle replay: no capture in 90 s).
- FINDING 2026-11-13 02:39 (agent; real-time replays + the 12 Nov bags): a 3 deg swing bound let his planner in over the MOVING
        figure-eight in the twin at 2.9 m (R4420, R4421), but not in real time (nro: one brief intercept after 85 s, after 8.5 s
        parked 0.13 m from a tether; fm9hc: captures only once the ring stopped), and the real QUAD4 cable swing exceeds 3 deg
        often (his C++ q4d_7: 60 % of the transit; Python transits after a lift up to 10 deg). The swing bound stays 5 deg;
        tracking 0.04 touched a cable in the twin (R4419). CPU load: the cap-2.9 line stalled once under 8 busy cores (0.57 m
        short) and its plan gaps grew to 7-11 s; the 3.8 line captured under 4 and 8 cores (one run each). The margin is his
        replans (100-190 ms) against his 0.1 s splice lookahead; fixed_splice_lookahead_s 0.15/0.20 under load is being tested.
- DECIDED 2026-11-13 02:54 (lab chat): CPP=on's splice lookahead default 0.20 s (SPLA=), not 0.15: both captured 2 of 2 at
        8 busy cores (0-1 % LATE), and the agent's last runs showed his replan p95 rising to 161-208 ms. Evidence saved:
        results/analysis/cpp_realtime_2026-11-13/ (README). The sheet (docs/rig/fri_2026-11-13.md, draft for Wesley) has row 1
        as his C++ drop-off over the paused figure-eight (PAUSE=takeoff) and the moving drop-off on the direct route.
- FINDING 2026-11-13 03:25 (lab chat; real-time replays of 12 Nov's circling flights, results/analysis/cpp_realtime_2026-11-13/
        README "The moving ring at real time with splice 0.20"; exploratory, one replay per row): with his splice lookahead at
        0.20 s his C++ planner CAN reach a ring circling at 0.125 m/s in real time (nro: 3 of 3 at spline 4, 25.8-40.8 s,
        QUAD4 2.62-3.13 m; spline 3/2: 9-39 s at 2.58-2.66 m), where at 0.10 s it never did (LATE ~40 % -> 0-2 %). But when it
        finds no plan it stops (no escape), and a stopped QUAD4 beside a moving ring gets swept: in 2 of 3 spline-3 continuous
        circles a carrier tether crossed QUAD4's hanging cable (0.00 m, centre lines) after 10-11 s parked; fm9hc_4 parked
        21.5 s at 0.12 m. Spline 4 parked under 0.5 s in 6 of 6 but in the 20-25 s moving windows of fm9hc caught the ring
        only after it slowed. DECIDED (lab chat, for Wesley to confirm by 08:00): his C++ planner is not proposed into a
        MOVING ring tomorrow (one park = a cable-on-cable contact with Te-Jen alone); row 1 stays his C++ drop-off over the
        figure-eight paused at his TAKEOFF (twin R4423-R4427 5 of 5 incl. the circle and the C++ return; still-ring replays
        8 of 8 under load); the moving drop-off flies the direct route (12 Nov 3 of 3; twin R4428). The twin batch 9
        (rig line exactly: spline 4, splice 0.20, cap 2.9) repeats row 1 and tries the moving figure-eight/circle for the record.
- FINDING 2026-11-13 03:40 (lab chat, correcting 03:25): the twin's "paused" drop-offs R4423-R4427 and R4429-R4433 never had a
        moving ring: his twin supervisor's start gate opened in the ring's 5 s hover hold, so his TAKEOFF (and our pause at
        it) came 4.7 s before the path started. They show the drop-off, the C++ return and the three LAND aborts over a still
        ring with the pause held through his landing, not the stop of a moving ring. Built (twin only, typed): his
        supervisor's start_gate_min_traj_t (the gate also waits for the path clock); make_t13_configs types 10 s for every
        paused figure-eight/circle. Batch 9 (the rig line exactly, his TAKEOFF 10 s into the path) is the real test of row 1.
        Registry R4423c.
- FINDING 2026-11-13 04:39 (lab chat; twin batch 9, the rig line exactly: outer_octagon, swing 5, spline 4, tracking 0.06, 30
        steps, refresh 0.2, cap 2.9, splice 0.20; his TAKEOFF 10 s into the running path; tools/pause_sequence.py): row 1 in
        the rig's order passed 2 of 2 (R4435, R4436) and the circle from 9 and 5 o'clock 1 of 1 each (R4439, R4437): the ring
        moving at 0.12 m/s before his TAKEOFF, at rest 2.9-3.0 s after it; his C++ transit from 10.8-11.6 s (>= 7.8 s after
        rest; his bridge predicts the ring frozen during the 3 s ramp, Te-Jen's Codex entry 18, so this margin matters);
        transit 16.5-25.6 s, QUAD4 <= 2.90 m, >= 0.29 m in his transit, >= 0.97 m in his climb; ball in the net 4 of 4;
        the ring reference still (<= 0.001 m/s) from his transit to his LANDED_DISARMED; the path resumed only after it.
        R4438 (our LAND in his transit): QUAD4 tipped > 60 deg on its loaded liftoff (the twin's rigid rod with the ball;
        QUAD4 alone disarmed), so the abort was not reached (R4429 covers it over the still ring). R4440 (the rig line into
        the MOVING figure-eight): see its row. Row 1 as on the sheet stands, for Wesley to confirm.
- FINDING 2026-11-13 04:44 (lab chat, correcting the 04:39 line on R4438): QUAD4 did not tip at liftoff; in his loaded climb
        (his MPC, before any C++) the twin's QUAD4 went into a growing pendulum swing (ball +-0.6 m) and fell from 2.1 m,
        landing 1.6 m away, >= 0.8 m from every carrier; QUAD4 alone disarmed, the ring level. As R4339 (12 Nov). The
        rig's loaded climbs from the mat on 12 Nov were 3 of 3 clean (fm9hc_2/_4/_5). R4440 (the rig line into the MOVING
        figure-eight): no capture in 6 min and the object 0.04 m from a carrier cable: the refusal in the scripts stands.
OPEN    2026-11-13 05:16 (lab chat, from Te-Jen's Codex entries 18-19; for Wesley and Te-Jen): during our attach pause's 3 s
        slow-down (and the speed-up at the resume) /payload/trajectory_state already advertises the hold (hold_s 60) while the
        path clock still slews, so his ring_bridge predicts the ring frozen ~0.19 m of path early (ring_bridge.
        compute_payload_state). Row 1 relies on the OBSERVED separation, not a gate: from the mat his TAKEOFF climbs ~2.8 m
        at 0.3 m/s and his C++ transit started 10.8-11.6 s after his TAKEOFF, >= 7.8 s after measured rest (R4435-R4439).
        An airborne or shortened TAKEOFF, a faster climb or a longer ramp would remove it. Two possible fixes, each a contract
        change to review (a gate needs Wesley's word and a critic): (a) an explicit admission condition before his C++
        grant (measured ring at rest and the advertised future stopped, both fresh); (b) the trajectory state carrying the
        clock rate so the bridge predicts the ramp. Not built before the 13 Nov flights.
- FINDING 2026-11-13 05:32 (lab chat; Te-Jen's Codex entry 19: the abort in the rig's order was unverified after R4438): our
        LAND during row 1 in the rig's order (figure-eight running, then his TAKEOFF; the rig line) passed 4 of 4: in his C++
        transit x2 (R4441, R4442), as he cleared the drop zone (R4443), in his return (R4444): take-back at once, the ring
        reference still (0.0000 m/s) through the LAND, everyone landed, no fleet abort. R4441: after the ball reached the
        floor the twin's rigid rod held QUAD4 up and it toppled at its disarm, 0.6 m from the nearest landed carrier (twin
        artefact).
- NOTE 2026-11-13 05:34 (lab chat, Te-Jen's Codex entry 20): in the 05:32 line R4443 is a RETURN-phase abort (his phase was
        already RETURN_TO_START when our LAND arrived), so batch 10 covers 2 transit + 2 return aborts; a LAND during
        CLEAR_DROP_ZONE is covered only over a ring that never moved (R4430). The return in R4423-R4444 is his Python
        transfer (return_after_drop:=direct in each run's LAUNCH line) except R4427/R4434 (cpp). R4441 counts as a toppled
        touchdown in the twin, not an upright landing.
- FINDING 2026-11-13 06:29 (lab chat, Wesley's question: what keeps the direct route clear of the carriers): nothing on the direct
        route checks the carriers; the clearance comes from the sequence: the climb to 2.8 m at the mat, the line-up (from
        beyond 1.2 m to plate 9's radial line, then straight in through the gap between plates 7 and 11, once per phase), and
        the drop descent only within 5 cm of the drop point, velocity matched. Measured on 12 Nov's three direct drop-offs
        into the CIRCLE (mocap 20 Hz, centre lines, his TAKEOFF to the drop; results/analysis/cpp_realtime_2026-11-13/
        direct_clearance.py): QUAD4's body >= 0.73 m from every carrier body and cable; QUAD4's cable >= 0.39 m from carrier
        bodies but 0.14-0.20 m from a carrier cable; the object's edge 0.09-0.15 m from a carrier cable, at the end of the
        transit or in the descent. The FIGURE-EIGHT direct drop-off (the sheet's backup) has never flown on the rig: twin
        R4428 only (0.27 m in transit, 0.14 m in the descent). The sheet's "as flown 3/3 on 12 Nov" meant the circle.
- DECIDED 2026-11-13 06:31 (Wesley: "I dont think stopping for the dropoff is good"; the backup "Obviously figure-eight
        but both should be possible"): row 1 of the 13 Nov sheet is the drop into the MOVING figure-eight on the direct route
        (`RET=direct bash ~/rig_fm9h8.sh`, its first rig flight); the circle on the same route (12 Nov 3/3) is its backup;
        the C++ drop-off over the stopped ring moves to Other options. Wesley asked whether the direct route can avoid the
        carriers and their cables: answered in chat (not buildable and tested before 08:00).
- DECIDED 2026-11-13 07:02 (Wesley, after the swing-3 replays: "can we run it right now for the figure 8 and circle
        trajectories in the lab?"): rows 1a/1b on the 13 Nov sheet: Te-Jen's C++ planner into the MOVING circle and
        figure-eight with his settings (swing 3 deg, spline 2; outer_octagon, tracking 0.06, 30 steps, refresh 0.2, splice
        0.20, cap 2.9), `CPP=on CPPMOVING=1 SWING=3 SPLINE=2 RET=direct bash ~/rig_fm9hc.sh` / `~/rig_fm9h8.sh` (dry runs
        checked). Evidence: real-time replays 11 of 11 reached the drop point, 9 while the ring circled, 6.8-17 s, <= 2.84 m,
        >= 0.36 m from every carrier cable, never still > 0.2 s before (results/analysis/cpp_realtime_2026-11-13/full/runs/
        *_tj_*). Pushed back once: open loop (no swing, perfect tracking), 3 deg below the real swing (4-10 deg), no
        figure-eight recording to replay, the no-escape stop unchanged; the twin batch (R4445-R4450) still flying. Stop rule
        on the sheet: QUAD4 still beside the moving ring ~2 s or its cable heading into a carrier's cable: our LAND.
- FINDING 2026-11-13 07:09 (lab chat; rows 1a/1b, Te-Jen's settings swing 3 deg + spline 2 / 2.5): twin R4445-R4450 6 of 6
        (moving figure-eight and circle 4 of 4: C++ transit 11.8-14.1 s with the ring at 0.12-0.15 m/s, <= 2.89 m, >= 0.31 m
        in transit, ball in the net, back at the start; the stopped figure-eight 2 of 2); real-time replays 12 of 12 (10
        while moving), under 8 busy CPU loops 2 of 2 (8.8 and 11.5 s, >= 0.34 m, no stop). Te-Jen's Codex (entry 24): the
        replays have no swing, so they do not establish a 3 deg bound on the real cable; the operator LAND is not a
        certified escape; the rows stay EXPERIMENTAL and operator-owned. Twin LAND tests during his transit into the moving
        ring running (batch 12).
- FINDING 2026-11-13 07:24 (lab chat; twin batch 12, Te-Jen's settings swing 3 + spline 2): our LAND during his C++ transit
        into the MOVING figure-eight and circle 2 of 2 (R4451, R4452: the ring froze, everyone landed, >= 0.74 m after LAND;
        QUAD4 toppled at touchdown on the twin's rigid cable). Repeats: figure-eight clean (R4453, 0.34 m); circle R4454:
        his backend went to UNCERTIFIED_HARD_HOVER, QUAD4 stood still > 3 s and the circling ring brought a carrier cable
        0.054 m from the object, then passed under it and he dropped. With these settings the moving runs are 5 of 6 clean
        in the twin (figure-eight 3 of 3, circle 2 of 3) and 15 of 15 in the replays; the stop-in-path failure is reduced,
        not gone. The sheet's rule (QUAD4 still ~2 s beside the moving ring: our LAND) covers it; in R4454 QUAD4 was still
        ~3 s before the closest point.
- DECIDED 2026-11-13 09:05 (Te-Jen at the rig: "recommend a fix and only implement it if you think it's really a nobrainer"):
        the rig scripts' cpp_splice default 0.20 -> 0.30 s (SPLA=0.20 restores it): on the rig his replans ran median
        127-218 ms, p95 283-296 ms (RIG-1113-fm9h8_5, -fm9hc), so 0.20 discarded a third to a half of them as late and the
        incumbent then braked / hard-hovered between plans. Recommended only (not built): his hand-over settle speed bound
        0.12 -> ~0.20 m/s (3 resets in -fm9hc) and a release gate on the object's distance from the drop point on the moving
        circle (released 0.16 m off centre, the object bounced off the ring edge).
- DECIDED 2026-11-13 09:09 (Te-Jen at the rig agreed both recommendations; "don't make it release too late, it has to
        eventually release"): typed switches, defaults unchanged, the rig scripts type them: prepare_settle_speed 0.20 m/s
        with CPP=on (PREPSPD), and drop_release_xy 0.06 m with drop_release_wait 3 s on every drop-off script (RELXY=0
        off): the release waits for the magnet within 6 cm of the drop point, at most 3 s after the lowering, then releases
        anyway. Not flown or simulated yet (unit tests only, 202 passed); first flight is the next 1a/1c.
- DECIDED 2026-11-13 10:15 (Te-Jen at the rig; the supervisor: no tracking-error brakes): his execution tracking brakes OFF
        (c1f6_moving_rendezvous.yaml), the hard hover latched once per fallback episode (his backend, rebuilt), and with
        CPP=on the scripts type: his C++ goal over the drop point at 3.00 m (drop_height 0.72, max_body_z 3.0), search
        ceiling 3.3 m, hand-over settle accel 0.8 m/s^2 (plus speed 0.20, splice 0.30, search 0.10 s, state timeout 1.0,
        release gate 6 cm / 3 s from earlier). Basis: RIG-1113-fm9h8_7/_10, -fm9hc_5 (brakes from 6-8 cm tracking error,
        hard-hover ratchet down into the cable zone). Not simulated; first flight is the next 1a.
- DECIDED 2026-11-13 10:40 (Te-Jen at the rig: "freeze it there at 3.0"): his C++ drop-off settings FROZEN for the rest of
        the session and the paper runs: CPP=on CPPMOVING=1 SWING=3 SPLINE=2 RET=direct with the scripts' defaults: goal over
        the drop point 2.99 m (drop_height 0.71, max_body_z 3.0), search ceiling 3.3 m, outer_octagon, carrier padding
        0.06, 30 rendezvous-time steps, world refresh 0.2 s, splice 0.30 s, search limit 0.10 s, cooperative state timeout
        1.0 s, tracking brakes OFF, hard hover latched per episode, hand-over settle 0.20 m/s / 0.8 m/s^2, release gate 6 cm
        / 3 s. Successes on this line: RIG-1113-fm9h8_13 (figure-eight); at 2.75 m fm9h8_14 passed (figure-eight) and
        fm9hc_9 failed (circle: no route, hard hover in the path, a cable crossing).
- FINDING 2026-11-13 ~12:00 (lab chat; the hero RIG-1113-fm9c_2, motion capture; results/analysis/hero_2026-11-13/): the
        re-attach missed because the magnet never reached plate 9: our comotion contact fired with the tip 0.8 cm above the
        plate (the window allows up to +4 cm, `comotion_normal_max` 0.04 typed by the scripts), his LIFT_PROOF began 0.1 s
        after his descent profile ended (his reference 3.3 cm below the plate, QUAD4 ~4 cm behind it), the tip rose with
        the reference (taut cable) and slid 7 cm sideways ("lost"). The 11 Nov catches read -2.7 to +0.4 mm at contact.
        The pickup: the tip reached the object in all three tries; 1-3 mm above the calibrated contact on the two that
        failed, 0 mm on the one that lifted; the reference went 2.3 / 3.6 / 3.6 cm below the surface, so more overtravel
        does not separate them (the cable goes slack at the bottom).
- OPEN    2026-11-13 ~12:00 (for Wesley and Te-Jen): the re-attach fix for 18 Nov, (a) our contact window
        `comotion_normal_max` 0.04 -> 0.01 m (a gate: Wesley's word), (b) a ~1 s hold at the bottom of his attach descent
        before LIFT_PROOF (his planner: Te-Jen's OK). Evidence pending: the replay of every real contact event
        (results/analysis/reattach_contact_2026-11-13/).
- DECIDED 2026-11-13 17:00 (Wesley, the supervisor's 4 -> 2 -> 4 of 12 Nov): on the real quads BOTH returning quads re-attach
        with Te-Jen's attach (his stack, one instance per returning quad), "ideally both detach and attach are commands
        simultaneously". Design study running (results/analysis/424_design_2026-11-13/: our stack, his stack as two
        instances, rig layout / hardware / twin plan); build plan and the decisions it needs (layout, a second funnel and
        magnet marker, Te-Jen's OK to run two instances) to Wesley before 18:00.
- FINDING 2026-11-13 17:19 (lab chat; CORRECTS the FINDING ~12:00 above; results/analysis/pickup_seat_2026-11-13/, all 27 rig
        lifts 14 Oct-13 Nov): the cable is NOT slack at the pickup seat. QUAD4's body stayed 5-45 mm ABOVE his reference in
        every lift (fm9c_2: 24 / 37 / 36 mm) and the body-to-tip length stayed at its free hang (-0.4..+2.8 mm), so his
        overtravel never reached the magnet: it hung on a taut cable at its swing tilt. The magnet's tilt on the object
        separates held from failed lifts (26 of 27): held lay flat (0.5-2.5 deg), every failure was tilted 5.3-10.8 deg
        (touching with an edge; the 1-3 mm seat gap is a side effect). So Te-Jen's reading ("not going low enough") holds:
        the body does not follow the reference down (the same ~4 cm lag as at the re-attach). Fix candidates for him: a
        deeper effective descent (overtravel that the body actually follows, or his z tracking offset removed) to give real
        slack, or a check before the lift that the face is within ~3 deg of the object's up axis (a gate: his and Wesley's word).
- FINDING 2026-11-13 17:23 (lab chat; evidence for the OPEN re-attach fix above; results/analysis/reattach_contact_2026-11-13/):
        all 25 real contacts (4-13 Nov) replayed through our ComotionProof (logged contacts reproduced within 13 ms):
        comotion_normal_max 0.01 blocks or delays all 7 high contacts (+8.5 mm or more, every one failed) and blocks none
        of the 9 proven catches (two delayed 13 / 94 ms); 0.02 still fires on fm9c_2 and fm9_5 try 1. fm9c_2's contact read
        +13.6 mm (the tip dipped to +1.8 mm on a ring bob and rebounded). At the bottom of his attach descent QUAD4's body
        sat 33-47 mm ABOVE his reference and never closed it (the same offset as at the pickup), so his ~3.5 cm press ends
        at plate level. Highest real catch +6.1 mm (nr1_2, 4 Nov): 0.01 keeps ~4 mm margin.
- DECIDED 2026-11-13 17:37 (Wesley, on the replay of all 25 real contacts above): our contact window 4 cm -> 1 cm above the
        plate on the rig: every re-attach script types comotion_normal_max ${CNMAX:-0.01} (CNMAX=0.04 restores it;
        results/rig/2026-11-13/scripts/*.sh and the ~/ copies, dry run checked). Code default unchanged. The bottom hold
        before his LIFT_PROOF stays Te-Jen's call (emailed him the finding, 13 Nov ~17:35).
- DECIDED 2026-11-13 17:37 (Wesley): the FULL 4 -> 2 -> 4 for Wed 18 Nov (over the staged option; pushed back once: 60-80 h,
        safety paths, the minimum items first). Square on plates 0/3/6/9: QUAD1 + QUAD2 the pair on 0/6, QUAD3 (drone_2) on
        3 and QUAD4 (drone_3) on 9 detach on one command and both re-attach with Te-Jen's stack (a second instance for
        QUAD3). Built behind a two-partner switch (default off: single-QUAD4 flights unchanged), twin + desk check before
        it flies. Design: results/analysis/424_design_2026-11-13/ (ours.md, tejen_stack.md, rig_and_twin.md).
- DECIDED 2026-11-13 18:40 (Wesley, on the critic of docs/experiments/2026-11-13_424.md, GO WITH CHANGES): (1) the second
        fold-in happens AT ONCE (no queue; the critic's finding 1-2), its blend starting from the current references;
        (2) the first rig 4 -> 2 -> 4 flies both of his TAKEOFFs TOGETHER (over the critic's and the design study's
        "succession first", and the supervisor's "then back in succession": pushed back once in the question; his call);
        (3) the ring-tilt drop stays 55 deg in configs/rig/attach_424_pair.yaml (over the locked chain-pair 45 and the
        critic's recommendation; his word on the gate thresholds min_survivors 2, drop_gap_deg 200, int_k_xy 0, 55 deg).
        Lab PC: twin T1 (pair hold) and T3 (set detach) started 18:40 (lab_batch, R4461/R4462 reserved).
- DECIDED 2026-11-13 19:05 (Wesley, on builder P's finding): after both returners re-attach the planner's nominal split was
        pair 4.87 N each, returners 0.1 N each (two slack riders; the least-squares over six rows with the measured catch
        points); fix: in two-partner mode a fold-in shares the load with the lift and level-moment rows only (level=True),
        keeping the measured catch points (~2.4 N each). Being built; twin re-fly of T4/T5 before the rig. Twin so far:
        R4461 (sequential 4 -> 2, today's code) ring tilt peak 43.6 deg; R4462 (set detach) 0.24 deg, the ring dipped
        10.5 cm for ~2 s at the release (sheet stop rule excludes the first ~3 s); the pair alone <= 0.24 deg, throttle
        0.72-0.73.

DECIDED 2026-11-13 19:05 (Wesley, thesis chat): the Results restructure. Results and Discussion 16-18 pages with at
least 12 on the real world. Simulation in the main text keeps only robustness to the attachment-point error, scalability
(incl. the relay) and the comparison with Quan et al.; fixed formation, detaching and attaching are shown with the real
world; the simulated comparison with the no-resize method leaves the main text (its table goes to an appendix, the
thesis chat's recommendation, unless Wesley says delete). The real world is told event by event up to the full mission;
the drop shows disturbance rejection and recovery (the drop-off credited to the collaborator's controller, his C++
planner included), the reattachment shows resizing (the attachment credited to his single-quadrotor controller). Setups
1 and 2 are told in the Methodology as lessons (why the next setup was tried); Setup 3 is the main setup. Setup 1's
material compressed (Fig 26, a two-row detection table; the rest to the appendix). Plan:
~/.claude/plans/majestic-crafting-prism.md; lab request Q013.

DECIDED 2026-11-13 19:50 (Wesley, thesis chat): the simulated resizing comparison (resizing at detection against the
no-resize baseline: tab:res_x1, fig:res_x1, X1) is DELETED from the thesis, not moved to an appendix ("it wasn't very
strong and doesn't read that well"). The thesis chat had recommended the appendix (the only comparison against not
resizing); Wesley decided. Resizing is shown by the real-world detachments and reattachments.
- NOTE    2026-11-13 19:51 (reviewer of F113-F115): the pickup FINDING above is refined for Setup 3: the magnet's tilt sorts 23
        of 24 Setup 3 lifts (26 of 27 counted the 14 Oct lifts); QUAD4's body sat 0.5-4.1 cm (median 3.2) above its
        reference at the seat; the cable stayed taut in 23 of 24 lifts (k1ob_2 7.7 mm slack). A correlate, not a tested cause.

DECIDED 2026-11-14 (Wesley, thesis chat): in the thesis, cable tensions are always the MEASURED tension (the trackers'
eq:meas_tension, Tpull; "the commanded is not reliable"), never the planner's commanded node-0 tension. The thesis chat
said once that F122's reviewer advises against Tpull as a cable tension (sums 7.8-8.0 N where 9.7 N is needed on some
flights, per-quad thrust-map bias, frozen while the collaborator's stream flies QUAD4); Wesley decided. In nrtfc_5 the
four measured tensions sum to 9.5 N at the blend end (9.7 N needed). Also: no dates in the results ("on 4 November");
the funnel is described as tried-without, not reliable, so used (no before/after comparison); the collaborator's
controller is left unexplained (no C++/Python transfer detail, no pickup statistics): the thesis reports the ring's
response and the resize.

DECIDED 2026-11-14 (Wesley, thesis chat, after the supervisors' papers' usage): "experiments" / "experimentally" replace
"in the real world" in the thesis; "real-world experiments" where it reads better, and 4.2 is titled "Real-World
Experiments". "Payload" replaces "ring" in general statements and measures (payload tilt, payload height, the payload's
centre; introduced once in 3.1 as the ring-shaped payload); "ring" stays only where its shape matters (plates around the
ring, the net across the ring). Figure labels follow ("Payload tilt", "Payload").

DECIDED 2026-11-14 (Wesley, thesis chat): "payload" also replaces "load" where it means the carried object (the
Introduction and the literature review); "load--cable" stays as the formulation's name, and "load" stays where it means a
force or a share of the weight ("redistribute the load", "load shares", "under load"). Figure 1: (a) the new agent
reattaching (fm9hc_4 story frame 09), (b) the four steady after the reattachment (fm9_5 story frame 09), the new agent
labelled in both (second from the left in (b), Wesley), no object label.

DECIDED 2026-11-14 (Wesley, thesis chat agreed): Results and Discussion puts the real-world experiments first (4.1
Real-World Experiments, event by event to the full mission) and simulation second (4.2: what the experiments could not
test: the attachment-point error, scalability with the relay, the comparison with Quan et al.). Appendices reordered to
stay cited in order: C Further Experimental Results, D Approaches Not Kept, E Further Simulation Results.
- DECIDED 2026-11-14 13:40 (Wesley, next week's plan; CLAUDE.md updated): sessions 08:00-11:00 Mon 16, Thu 19, Fri 20 Nov
        (Wesley), Tue 17 and Wed 18 (Te-Jen alone; with Wesley Fri). Mon: detach and re-attach x3 each in hover,
        figure-eight, circle (nrtf / nr8 / nro: detach while flying, his TAKEOFF 5-10 s later stops the ring, re-attach,
        fly on), then one 4 -> 2 -> 4 (setup in the battery breaks); the hero later in the week. Tue: Te-Jen's drop-off x3
        each in hover, circle, figure-eight, then hero attempts. Wed/Thu/Fri: what is short, the best hero. The detach
        keeps its 3 s unload path pause (Wesley over "unload while moving"). Thesis: real-world flights up to Thu 19 Nov;
        Friday's for the combined paper; hand-in Fri 20 Nov 17:00.
- FINDING 2026-11-14 14:32 (lab chat; the critic's finding 11, runtime desk check of rig_424.sh with real_io:=false, fake
        mocap for the square, ROS domain 77; scratchpad desk424/report.txt): PASS on wiring.
        - 25 nodes, both of Te-Jen's instances up, nothing crashed in ~8 min.
        - All 92 of QUAD4's per-partner topics have a QUAD3 twin by the contract's rule, with identical publisher and
          subscriber sets (0 differences); no cross-feeding.
        - An injected /drone_2 weld flips only mux_2 (isolated muxes); on the live graph neither mux reacts at boot, by
          design (both welded, ours).
        - CPU 8.5 of 12 on the ground without radios, mocap, RViz or the bag: the card's risk 7 stands. Sheet: close RViz
          for row 10, watch the carriers' radio rate.
        - ~/rig_424.sh copied.
- DECIDED 2026-11-14 14:51 (Wesley): Te-Jen is fine with a second instance of his stack on QUAD3 for 4 -> 2 -> 4 (Monday's row
        10). Paper/thesis credit wording, no specifics: "a controller from a collaborating student, for a single
        quadrotor with a swung payload, makes the attachment" (replaces the "C++ approach, Python return" detail as the
        credit line; the drop-off and attachment are credited to that controller, the ring's response and resizing to the
        proposed planner). His descent choice and the C++ line: waiting on his reply.
