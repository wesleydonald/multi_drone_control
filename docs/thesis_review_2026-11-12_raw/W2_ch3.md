# W2 Chapter 3 (12 Nov) - as received (condensed); equations checked against code: sign of integral, eq 6, eq 13, RTI + 60 ms budget, 20-node horizons, 1.6/2.2 N shares, support margins all match.
F1 must: drop rule 3.4.2 (reconfiguration:55) "fewer than three ... or 200 deg or more" contradicts code default (fewer than three or gap >= 180; dissipative_node.py:1926-1945, bringup/config/common.yaml:117 min_survivors 3) - 200/2 quads/45 deg only in chain_pair configs (configs/rig/chain_pair_tip_cap20.yaml:7-9) [VERIFIED by me too]. Rewrite given.
F2 must: test_cases:16 "0.2 m/s used for all trajectories" vs 3.6.3 0.4 m/s; "0.9 m line" (sim_studies:34) vs Table 8 d = 0.75 m.
F3 must: system_overview:196 "Recording a video on the computer while running the mocap software doubled the delay ... So the lab computer did not run anything" - wrong computer/untrue/"So"; it was a Motive recording on the mocap PC.
F4 must: "tick" 17 times in Ch3 (controller:9,11,35,40; reconfiguration:46; metrics:32 "Late ticks"; sim_studies:23,25,27) -> control step; Table 10 "Late ticks" vs results "Late control steps".
F5 must: system_overview:126 "baseline runs", "even split", "evenly spaced triangle", "leaves".
F6 must: reconfiguration:9,114,116 "the new agent's catch", "confirm the catch", "does every attach", "so the switch cannot cut".
F7 must: controller:79 "RMS error of 0.004, which is about 0.15 N" - 0.004 x 19.4 N = 0.08 N; 0.15 N belongs to the 0.008 max.
F8 should: gap study + baseline comparison (4.1.2) have no method in Ch3; Table 9 does not list them.
F9 should: Table 10 metrics vs results (maximum tilt, second swing, settled tilt, height loss, recovery missing; time in band never reported; "(below)" wrong).
F10 should: own tip-steering approach (sim reattachments) not explained in Ch3 [it is in 4.1.3 by Wesley's move].
F11 should: 3.6.1 offsets 16 cm rim / 20 cm inward have no plate on the real ring; who flies the new agent not stated.
F12 should: 3.1.2 uses thrust map, measured tension, disturbance estimate, slope velocity before they are explained (content forward refs); velocity estimate repeated in 3.3.3.
F13 should: system_overview:57 "three nodes" vs Appendix A eight; fleet manager does trajectory/reconfiguration? (planner does); "take-off".
F14 should: controller:79 "It was found that" x2; only one quad flew 196 g (others 0 and 98 g) per registry 601-602.
F15 should: kT 35.2 at 0.55 kg kept on 0.583 kg Setups 2/3 (real.yaml:10, :47) - say why or per setup.
F16 should: reconfiguration:53 reason for prebuilt solvers; "warm-starts twice" = two solves on a hover reference from measured state (dissipative_node:1836-1840); model.tex:68 "without rebuilding its solver" vs switching solvers.
F17 should: model:70 "Each quadrotor is therefore its own tracking MPC"; cascade introduced 3 times; "a throttle and body-rate commands".
F18 should: model:74 "a few modifications" never named.
F19-F21 should: state list parse; q used for 3 things; G_k gate wording, K "measured stiffness" parse.
F22 should: system_overview:160-162 inflated Gazebo sentence; cause reversed (0.22 real time); emulator source uncited.
F23 should: six comma splices; "loss".
F24 should: Simplex terms w/o pointer; position check undescribed; LAND only defined in Appendix A; take-back unexplained.
F25 should: "An announced detachment informs the planner"; "one-step" undefined.
F26 should: attachment split over 3.4.1/3.4.3/3.4.4/Table 7; 0.1 N said 3 times; blend duration not given; "lever about the apex".
F27 should: Table 7 0.15 m vs 3.4.1 and Appendix B 0.06 only; unclear cells.
F28 should: "errand", "admitted", "since" cause.
F29 should: tilt vector definition parse; t_e only for detachment; "without the clock"; lag method.
F30 should: "centralised control ... they"; "a bench"; 3.4 vs 3.5 factor; ratio from 3 trackers applied to 10.
F31 should: Table 8 sizes are real-world; sim defaults 0.9 m figure-eight, 1.0 m line (common.yaml:31-32); circle 0.3 m in results vs r 0.5 in Table 8; "drone cage".
F32-F40 could: one-flight swing claim; notation reuse (m, f_i, g_max, r, k_i, u, e_R, F); Fig 6 caption purple/apostrophe; "a ExpressLRS"; "three or less", "infinite solutions", plates never collinear, alpha anticlockwise; 5 s = 86 % not converged; smootherstep wording, T undefined, Fig 13(b) amplitude ~0.31a; stacked headings 3.5/3.5.1 and Table 9 before 3.6; "gains" agreement, cable config of the Quan detachment, gains location.
