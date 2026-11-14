# 4 -> 2 -> 4 on the rig: what OUR stack must change (design study, 13 Nov 2026 ~17:15 AEDT)

Read-only study of the working tree on week8 (bf52a8a + the uncommitted diff). Nothing in `src/`, `configs/` or
`tools/` was changed. Companion files in this folder: `tejen_stack.md` (his stack as two instances) and
`rig_and_twin.md` (layout, hardware, twin plan). Decision behind it: decisions.md DECIDED 2026-11-13 17:00.

## 1. Verdict

Our stack is built around ONE partner. The pieces that are per drone already (fleet manager, magnet gate, the magnet
manager's topics, the planner's reserved-slot arrays, on_load, the OCP resize to any prebuilt size) carry over. The
pieces that decide what happens to the partner (the planner's partner_contact state, the shared magnet topics, the
launch graph's newcomer chain, the console and RViz buttons) are hard-wired to drone 3 / plate 9 / body 24 / one tip.

Four of the single-partner assumptions are safety faults with two partners, not just missing features:

1. `/magnet/object_attached` is one shared Bool. One proof folds BOTH pending partners into the OCP and flips BOTH
   muxes to our tracker, wherever the second one is (`dissipative_node.py:2401-2420`, `elrs_mux.py:156`).
2. `_taken_back` is one bool. On LAND, on a drop (`drop_takes_back`) or on a give-up, only the FIRST pending partner
   is taken back; the second stays on his stream (`dissipative_node.py:2723, 2731`, callers 2015-2017, 2712-2714,
   2640-2643).
3. The ring-magnet span ON/OFF goes to one `/magnet/command`, and the span runs only for reserved index 0
   (`dissipative_node.py:680, 2659-2690`): the second partner never gets its span, or both coils get the first's.
4. The take-back release guard reads ONE tip (`/magnet_tip_pose`) against ONE weld point (plate 9)
   (`dissipative_node.py:846/888, 873-880, 2904-2980`): a take-back of the plate-3 partner checks QUAD4's tip.

Detaching two at one command does not exist: `/fleet/detach` is one Int32, and with the rig's soft detach
(`detach_unload_s` 3.0) a second DETACH is refused while the first unloads (`dissipative_node.py:1736-1742`).
No rig flight has ever reached the pair through the chain (section 5). The twin has (X2), with our own re-attach.

Effort for our stack: about 46-60 h with the twin rehearsal, about 40 h for a minimum (queued detach and
fold-ins, console only) (section 9). Recommended layout: the pair QUAD1/QUAD2 (drone_0/1) on plates
0/6, returners QUAD3 (drone_2) on plate 3 and QUAD4 (drone_3) on plate 9 (section 2).

## 2. Layout (confirm or change)

- Our planner numbers reserved (attach) drones after the carriers: drone id = `n_carry0 + j`
  (`dissipative_node.py:796, 1154-1158`), and the rig maps drone i to `/dev/QUAD<i+1>` (`real_mode.py:359-365`).
  So with two carriers (n = 2) the returners are drone_2 = QUAD3 and drone_3 = QUAD4 unless the radios are remapped
  (`drone<i>_serial:=`, possible but one more thing to get wrong on the day).
- Pair on 0/6 (typed `attach_azimuths_deg:=0,180`), QUAD3 starts welded on plate 3, QUAD4 on plate 9 (its funnel,
  body 24 and calibration stay). The square 0/3/6/9 is the 29 Oct `sq` layout and the twin's X2 square.
- What plate 3 needs that plate 9 has: a funnel, QUAD3's magnet marker (Motive body 23 on 28 Oct) redefined at the
  tip as body 24 was on 4 Nov, and its record (W = weld_point_z, X/Y, L = rod, K = contact radius, taut length).
  `graph_attach.reattach_plate_point()` (`graph_attach.py:320`) can turn plate 9's measured X/Y onto plate 3, but W
  with a funnel must be measured.
- Motive: 8, 11, 12, 13, 14, 23, 24 = 7 bodies (no object, so no body 6): inside the 8-body limit.
- Physics of the layout (risk R1, section 10): plates 3 and 9 sit 90 deg off the pair's line, where the pair has
  no stiffness. Any other returner plates are off the line too (1/7 halve the lever, but crowd the carriers' cables).

## 3. Prioritised change list

P0 = needed for any two-partner flight (twin or rig) and for safety. P1 = needed for the rig flight as asked
(simultaneous commands, operator tools). P2 = display, analysis, guards that are off on the rig.
Size: S < 2 h, M 2-5 h, L > 5 h (code only, tests in section 9). Line numbers are today's working tree.

### 3a. Planner (`src/dissipative_planner/dissipative_planner/dissipative_node.py`)

| # | P | Where | What it assumes today | What must change | Size | Risk |
|---|---|---|---|---|---|---|
| 1 | P0 | 1122-1124, 2401-2420 `_magnet_attached_cb` | one `/magnet/object_attached` for all reserved drones: a True folds in every pending one | per-partner attached topic (`/drone_<d>/object_attached`), each folds in its own j | S | H if missed: one proof folds both |
| 2 | P0 | 869, 2723, 2731 `_take_back`; guards 1283, 2447, 2500, 2664, 2047 | one `_taken_back` bool per flight | per-j set; every guard reads its own j; `DROP_TAKE_BACK_EXTRA_S` if any | M | H: second partner never taken back on LAND/drop |
| 3 | P0 | 914-929 | `/drone_{n_carry0}/take_back` publisher, `/drone_3/mux_state`, `/join_planner/phase`, `/join_planner/give_up`, `/tejen/object_attached`, `/magnet/contact`: one partner | per-j publishers/subscriptions from a partner list parameter (topics per his instance) | M | M |
| 4 | P0 | 2604-2626 `_his_phase_cb`, `_mux_state_cb`, `_ball_cb`; readers 2645-2657 `_contact_warm_ref`, 2495-2509 `_contact_release_tick`, 1276-1296 `_attach_pause_wanted`, 284-292 `_free_drone_ff` | `_his_phase`, `_his_took_off`, `_mux_state`, `_ball_on` are single values | dicts keyed by j | M | M (warm reference of the wrong quad) |
| 5 | P0 | 2659-2690 `_contact_span_tick`, 868 | the span runs for `attach_pending[0]` / `_pos(n_carry0)` only; `_span_t`, `_span_word`, `_his_descended` single | loop over j with per-j span state | M | H: QUAD3's coil never gets ON |
| 6 | P0 | 680 `_magnet_cmd_pub`; users 1770 and 1863 (reserved leaver OFF), 2035 (drop), 2416 (refused weld), 2573 (tilt fault), 2690 (span), 2736 and 3033 (take-back), 3167, 3204, 3273, 3299 (refusals, aborts, OFF resend) | one `/magnet/command` reaches the one manager | one command topic per partner; drop and LAND send OFF to all | M | H: an ON/OFF for one reaches both coils |
| 7 | P0 | 716, 2452 `_release_pub` | `/partner/release` String `{"running": true}` with no drone id | per-partner release topic (his takeoff gate, our mux and gate per partner) | S | H: first release starts both of his instances |
| 8 | P0 | 558-561, 873-880, 2692-2706, 3430 | one `attach_cable_len` / body-to-tip, one weld point (attach_x/y_offset, weld_point_z) | per-partner rod and weld point lists; `_weld_rho_of(j)` and `_reserved_plate_dir(j)` per j | M | H: a QUAD3 take-back over the centre heads to plate 9; wrong rod in the fold-in |
| 9 | P0 | 846, 888 `_tip_now_cb`; 2904-2980 `_guard_*` (`take_back_release_guard` is on on the rig, graph_attach 651) | one tip on `/magnet_tip_pose` | per-partner tip topic and `_tip_now[j]` | S | H: guard reads the other quad's tip, holds or LANDs wrongly |
| 10 | P0 | 2798 `_take_back_plan` | the carriers are `drone_pos[:n_carry0]` | the bodies now in `slot2drone` (a welded partner is a carrier) plus the other free partner | S | M |
| 11 | P0 | 2640-2643 `_give_up_cb` | his give-up takes back every pending partner (with #2, really only the first) | one give-up topic per instance, mapped to its j | S | M |
| 12 | P1 | 1709-1770 `_fleet_detach_cb` (refusal 1736-1742), 730 `_unload`, 1793-1816 `_detach_plan`, 2056-2111 `_start_unload`/`_unload_tick`, 1827 `_resize_and_release`; `reference_builder.py:34 unload_tensions(..., k, ...)` | one leaver per command; the soft unload serialises detaches | a set detach: one message names both (new `/fleet/detach_set`, or `/fleet/detach` as a String list), `unload_tensions` over a set K, the gates on every leaver, ONE resize to the survivors, every leaver released and stepped out, one hold. Minimum fallback: queue the second DETACH until the first is released (S) | M | M (release path; critic) |
| 13 | P1 | 3502 `t_before = list(self.refs._t_nom)`, 3560 `s_from = ... self._s_nom[:-1]`, 3583 `start_blend` | a fold-in starts from the nominal split; a second fold-in inside the first's 10 s blend (`attach_traj_hold_s` 10) throws the first's blend away, so the first returner's tension steps from its blended value to its full share | start the new blend from `refs._refs_at(0)` (as `retarget` does, `reference_builder.py:158-162`), or queue the second fold-in until `blend_active()` is false | S (code), M (risk) | M: untested anywhere (section 7) |
| 14 | P2 | 838-847, 899-913 | `welded_on_load`, `partner_tilt_cut`, `partner_stuck_guard` wired to `d3 = n_carry0` and one tip | refuse them with more than one partner (all off on the rig: common.yaml 289-300), or per j later | S | L |
| 15 | P2 | 1318-1333 `partner_rod_add_m` | QROD on `d = n_carry0` only (off by default) | per partner or refuse | S | L |

Already per partner (no change): reserved mocap and reference publishers (1111-1116), on_load and arming per
reserved drone (850-859), `_newcomer_out` / `_abort_newcomer` / LAND loop (3193+, 4533-4550), `_release_state` per
drone (2438-2493), `_comove` per drone, start-weld fold-in into the creep (3439-3445), `prebuild_sizes(2, 2, 2)` =
{2, 3, 4} (`mpc_planner/params.py:51-57`; the rig's 0.86 kg copy `~/mdc_rig_generated_m2a_0p86` holds
load_cable_2/3/4).

### 3b. Launch graph (`src/bringup/bringup/graph_attach.py` unless named)

| # | P | Where | What it assumes today | What must change | Size | Risk |
|---|---|---|---|---|---|---|
| 16 | P0 | 47-48 `ATTACH_DRONE_ID = 3`, `ATTACH_DRONE_NAME`; 59 `COIL_TOPIC`; 680-686 refusal `n != 3`; 706-719 `n_radio = n + 1`, latches, ONE `magnet_topic_overrides`; 723-1015 the newcomer chain built once for d = 3 | one newcomer, always drone 3 after three carriers (R2828 was VOID on exactly this) | a partner list (ids n..n+k-1, plate, tip body, record) and the chain (tracker, mux, manager, gate, his include) in a loop; `magnet_topic_overrides` for every partner | L | M |
| 17 | P0 | 554 fleet manager `num_drones n + 1` | one newcomer | `n + k` | S | L |
| 18 | P0 | 566-660 planner parameters | `reserved_attach` typed separately; one `attach_cable_len`; one weld point in the partner_contact block (650-652) | `reserved_attach = k` from the partner list; per-partner rods, weld points, topics | M | M |
| 19 | P0 | 774-794 `elrs_mux_{d}` | `release_topic '/partner/release'`, attach topic shared | per-partner release and attached topics | S | H (with #25) |
| 20 | P0 | 962-990 `magnet_attachment_manager` (one node, fixed name) | tip `/magnet_tip_pose`, attached `/magnet/object_attached`, command `/magnet/command`; not passed so left at the manager's defaults: `body_pose_topic` `/drone_3/motion_capture_state`, `/magnet/contact`, `/magnet/attach_proof`, `/magnet/attachment_state`, `/join_planner/proof_lift_complete`, `/join_planner/phase` (`magnet_attachment_manager.py:77-83, 201-215`) | one manager per partner, every topic passed, its plate's object offsets and record | M | H: the proof of one partner would read the other's lift and phase |
| 21 | P0 | 992-1006 `magnet_gate_{d}` | `/tejen/magnet/command`, `/join_planner/phase`, `/tejen/object_attached`, `/partner/release`, one `magnet_operator_topic` | per-partner topics (the node itself is fully parameterised, `magnet_gate.py:239-251`) | S | M |
| 22 | P0 | 178-208 `_partner_nodes`; 66-69 `PARTNER_OWNED`; 200 `takeoff_gate_topic '/partner/release'` | one include of his launch, contact target from the one weld point | one include per partner with its namespace, drone, plate, carriers, release gate, contact target. Needs his launch parameterised first (`tejen_stack.md`: `partner_mission.launch.py:63-64` NS 'tejen' / DRONE 3, root topics, backend plate topics `range(3)` at 386) | M (ours) | M |
| 23 | P0 | 123-176 `_combined_rig_quad4_refusals`, 211-263 `_contact_manager_params` (one proof_taut_len), 334-371 `_reattach_plate` | one QUAD4 record and one plate | the checks per partner record; records typed through ONE file (e.g. `configs/rig/partners_424.yaml`: drone, plate, tip body, W, X, Y, L, K, taut) rather than 2 x 10 launch args | M | M (a typo puts a contact target on the wrong plate) |
| 24 | P1 | 56 `BALL_ROUTE`, 718 | his ball body 6 routed under partner_contact | no object in this test: type `mocap_extra_bodies` without 6 (keeps Motive at 7 bodies) | S | L |
| 25 | P0 | `drone_magnet/elrs_mux.py:156` | subscribes the shared `/magnet/object_attached` | `attached_topic` parameter | S | H: the second partner's mux hands it to our tracker mid-air (the combined-stack critic 4 failure) |
| 26 | P0 | `drone_communication/motion_capture_publisher_node.py:33, 367-371, 395, 574-577` | one `magnet_tip_body_id` published on `/magnet_tip_pose` | a list `tip_bodies 'id:/topic,...'`, each PoseStamped (or the manager learns to read MotionCaptureState and the tip goes through `extra_bodies`) | S | L |
| 27 | P0 | `drone_magnet/magnet_attachment_manager.py:285-295` | tip only as PoseStamped/PoseArray | nothing if #26 publishes PoseStamped; else a `mocap_state` tip type | S | L |
| 28 | P0 | `bringup/real_mode.py:354-356` `rig_io` | `mocap_vel_drones` = every radio but the LAST: only QUAD4 keeps `ema` for his MPC | every partner keeps `ema` (QUAD3 then carries on ema in the square, as QUAD4 does today) | S | L-M (his MPC on QUAD3 tuned on ema) |
| 29 | P1 | `bringup/launch/real_io_launch.py:176-184` fleet_viz | `tether_drone 3`, one tip, coil, attached topic | lists (RViz display only) | S-M | L |
| 30 | - | `fleet_manager_node.py:138-156, 385-436` | per drone already (mux state, on_load, partner disarm, ARM gate) | only `num_drones` (#17) | - | - |

Twin side of the same graph (needed for any twin rehearsal): `graph_attach.py` 727-772 (bridges, mocap emulator,
bf_comm for d) and 1017-1098 (magnet_tip_publisher, `/payload/attach|detach` bridges, the one detachable-joint
state) are one partner and one gz DetachableJoint. A twin two-partner world (pair 0/6, standalone partners welded on
3 and 9, a joint each) does not exist: `tools/make_attach_layout_worlds_L100.py` 'pair' has one partner. L
(details in `rig_and_twin.md`).

### 3c. Operator tools, scripts, configs

| # | P | Where | What it assumes today | What must change | Size | Risk |
|---|---|---|---|---|---|---|
| 31 | P1 | `tools/console.py:49-58` (one his command, give-up, land, force-drop topic; `HIS_NODES`), 93 `partner=3`, 405-407 one "DETACH QUAD4" per flight, 460-470 one his phase/armed/release/proof/attached, 521-533 buttons | one partner row | a row per partner (his ARM, his TAKEOFF, GIVE UP, his LAND, his DISARM, magnet OFF/ON, readiness) plus "both" buttons: DETACH BOTH and his TAKEOFF BOTH (the simultaneous commands) | M | M (a button on the wrong quad) |
| 32 | P1 | `src/drone_visualisation/src/arm_panel.cpp:29-30` (`/tejen/tejen_mpc`, `/online_join_planner`), 146-158 and 679-693 (DETACH spin box default 3, one per flight), 193-204 and 294-300 (one his row on `/tejen/*`), 251 ATTACH on `/magnet/command` | one partner | per-partner his row and a DETACH-set button, or hide his row and fly the partners from the console (Wesley prefers RViz buttons: his call). C++ rebuild | M | M |
| 33 | P1 | new params file (from `configs/rig/attach_m1_1m_m2a_drop55_det15.yaml` + `configs/rig/chain_pair_attach_1m.yaml`) | attach mode defaults `min_survivors` 3 (common.yaml 121: the second DETACH was refused in R2827), no gap rule | `min_survivors 2`, `drop_gap_deg 200`, `int_k_xy 0`, the drop tilt (45 chain / 55 rig attach), `drop_takes_back`, the M2A ring values. Gate thresholds: Wesley's word | S | M |
| 34 | P1 | `src/bringup/config/real.yaml:104-115` modes.attach | `num_drones 3`, carriers `90,330,210`, plate 9's point | typed in the script: `num_drones:=2 attach_azimuths_deg:=0,180` and the partner file | S | L |
| 35 | P1 | new script `rig_424.sh` (from `results/rig/2026-11-13/scripts/rig_nrtf.sh`) | one `partner_args`, one record | the partner file, both of his argument sets, hover | S | L |
| 36 | P2 | `tools/rig_flight.sh:89-103` | records QUAD4's and `/tejen/*` topics | per partner and per instance | S | L |
| 37 | P2 | `tools/mocap_stream_check.py` (sheet line), `tools/rig_cable_check.py`, `tools/dropoff_summary.py`, `tools/pause_sequence.py` | QUAD4 / body 24 | `--expect 8,11,12,13,14,23,24`; the analysis tools per partner later | S | L |

## 4. Detach: can one command release two plates?

No, today. What exists:
- `/fleet/detach` is `std_msgs/Int32`, one drone (`dissipative_node.py:1063`, console `detach(i)` 521, RViz panel
  DETACH one per flight).
- In OCP mode with `detach_unload_s` > 0 (the rig: 3.0) a second DETACH during the first's unload is refused
  (`... refused: drone X is unloading`, 1741). After the first release it would be accepted (the planner, not the
  tools, allows a second).
- The rig's chain to a pair was always one announced soft DETACH (plate 9) and then an UNANNOUNCED release of the T's
  rider, caught by the magnet marker or the stretch check: RIG-1022-t3p_2 (the OFF never reached the radio),
  RIG-1022-t3p_3 (the OFF went to /drone_0, one of the opposite pair: 270 deg gap, DROP AND LAND as designed),
  RIG-1029-ch1/ch1b (plate 0 released under the load, 1.15 kg ring), RIG-1029-ch1b_2 (the T on 0/3/6 at the 0.9
  throttle cap, ring sank 16 cm; the trigger most likely QUAD4's released magnet, per its correction row). No rig
  flight reached the pair this way.
- The twin did: R2821/R2822, R2829, X2 R3586-R3591 and R3986-R3991, always one leaver per event.

Recommendation: build the set detach (#12), not a queue. Taking 3 and 9 off the square together is symmetric: the
two leavers' moments cancel at every step of the unload, and the ring goes square -> pair without the T. The
sequential path passes through the T, whose soft square-to-T step peaked 23-31 deg in the twin (X2 A1: 29.67 / 28.39
R3586-series, 31.03 R3986) and whose opposite pair ran into the 0.9 throttle cap on the 1.15 kg ring (RIG-1029-sq1,
ch1b_2). The symmetric-unload prediction is mine, not flown: the twin must show it first.

## 5. Fleet resize n = 2 -> 3 -> 4 with two attaches pending

- Two pending reserved drones are supported by the arrays (`attach_pending[j]`, `attach_pos[j]`, references per j),
  and each fold-in is an in-place OCP resize (`_attach_ocp`, 3447-3621) at the measured catch point.
- What is missing: per-partner weld signals (#1), per-partner partner_contact state (#2-#11), and overlapping fold-ins
  (#13). With `attach_traj_hold_s` 10 the first fold-in blend lasts 10 s; a second proof inside it restarts the blend
  from the nominal split. The X2 twin never overlapped two fold-ins (its ATTACHes were a lap apart, and
  `_rejoin_busy()` serialises rejoins), so this is untested in both worlds.
- Sequential re-attach goes pair -> T (0/6/9) -> square. In the T the first returner's planned share is the rider's
  (about 0.1 N, X2 card): a nearly slack cable, the condition of QUAD4's 3-4 Hz slack-taut bounce of 11 Nov
  (`docs/analysis/quad4_bounce_2026-11-11.md`). Simultaneous fold-ins go nearer pair -> square.
- Suggested order of work: first the per-partner refactor with the re-attach SERIALISED (his TAKEOFFs pressed together,
  but the planner holds the second fold-in until the first blend is over, #13's queue form); overlapping blends after
  the twin has scored them.

## 6. Drop, survivors, faults, LAND and take-back with two partners

| Rule | Today | With two partners |
|---|---|---|
| `drop_on_loss`, `min_survivors` | attach mode default 3; the rig attach files do not set the chain rules | type `min_survivors 2`, `drop_gap_deg 200` (#33). In the pair any loss leaves one: drop and land, as wanted. `_drop_due` (1937-1957) already handles the T and pair |
| tilt drop | trackers' `drop_tilt_deg` 55 (rig attach files) / 45 (chain files) | Wesley's choice; see R1 for why the pair's tilt matters |
| `newcomer_fault_land` | per reserved drone (`/drone_d/on_load`, fleet manager per drone) | works for both once each partner has its own on_load (it does) |
| LAND before both re-attach | `_land_take_back_tick` loops j but `_take_back` stops after the first (#2) | fix #2; both taken back on their own bearings (opposite sides) |
| drop while both are out (`drop_takes_back`) | 2015-2017 loops j, same stop after the first | fix #2 |
| his give-up | one topic takes back every pending partner | per instance (#11) |
| take-back route | carriers = the first n drones; plate direction from the one weld point | #8, #10 |
| `carriers_down_alone`, `land_descend_to_stall` | per drone | no change; the LAND's wait gets `DROP_TAKE_BACK_EXTRA_S` once if any partner was taken back |
| `pos_limit_m` 1.8 | fleet-wide | release step-out 0.5 m out + 0.3 m up from a body ~0.73 m out puts each partner ~1.2 m out on its own side: inside |

## 7. The twin: what X2 already supports, what carries over

X2 (card `docs/experiments/2026-10-30_x2_full_chain_fig8.md`, R3586-R3591, R3986-R3991) flew square -> T -> pair -> T
-> square with OUR re-attach (`rejoin_departed`: departed carriers stage in the ring frame, our approach flies them
back, the gz tether joint re-welds, the OCP grows with the plate point and rod they left with).

Carries over to two partners flown by his stack:
- the OCP resize in place for any slot order and the prebuilt sizes 2-4 (`resize_fleet`, `prebuild_sizes`);
- `_attach_ocp`'s fold-in (apex direction, balanced split, blend, hold), and its `rejoin=` form that folds in at the
  plate point the drone left from (an option for partners returning to their own plate; the rig's rule today is the
  measured catch point, so keep that);
- the T and pair rules (`drop_gap_deg`, `min_survivors` 2, `_drop_due` excluding a drone still blending) and their
  params (`configs/twin/x2_chain_tip80.yaml` with the twin's 80 deg limit; the rig's are 45/55);
- more than one drone off the ring at once (`_rejoin` dict, `_departed_hold`, `_stage_departed`): the pair stage of X2
  had two staged at once;
- the soft single-leaver detach on a path and the marker check (`tip_detect_m`).

Does not carry over:
- the weld and approach: gz joint `/drone_i/attach`, `REJOIN_WELD_*` distance gate, `_rejoin_approach_tick`,
  `tether_tips`; on the rig the weld is the magnet manager's co-motion proof on a tip body, flown by his planner;
- identity: X2's returners are departed CARRIERS (`d < n_carry0`); partner_contact works on RESERVED drones
  (`d >= n_carry0`). Mixing the two (QUAD4 reserved, QUAD3 a departed carrier) would need a third path; making both
  reserved (section 2) keeps one path;
- concurrency: rejoins were serialised (`_rejoin_busy`, one `_rejoin_blend`), a lap apart. Two pending at once yes,
  two fold-ins at once never.
- Twin with his stack on the pair: R2829 (27 Oct) welded QUAD4 onto the pair 0/6 first try (his floor_join,
  pair -> T); R2828 (a pair start) was VOID on `ATTACH_DRONE_ID = 3`. The tilt across R2829's contact was not scored
  (its run is in `results/archive`): worth reading before the rig (R1).

## 8. Thrust margin: the pair carrying the 0.86 kg ring

Map (CLAUDE.md, `tracker_node.py:672-687`): throttle = 0.185 + 0.507 * (thrust / g, kg) - 0.022 * (V - 23.5).
Quad 0.583 kg with pack, cable and magnet (`real.yaml` drone_mass), planned cable angle 60 deg
(`common.yaml` cable_elev_deg), level hover, no dynamics:

| Formation | Thrust per quad (N) | Cable tension (N) | Throttle 23.5 V | 22.6 V | 22.0 V | 21.5 V |
|---|---|---|---|---|---|---|
| square (4), 0.86 kg | 7.92 | 2.44 | 0.594 | 0.614 | 0.627 | 0.638 |
| triangle (3), 0.86 kg | 8.68 | 3.25 | 0.634 | 0.654 | 0.667 | 0.678 |
| **pair (2), 0.86 kg** | **10.23** | **4.87** | **0.714** | **0.734** | **0.747** | **0.758** |
| pair (2), 1.15 kg (reference) | 11.82 | 6.51 | 0.796 | 0.816 | 0.829 | 0.840 |

Maximum thrust at the caps: 0.8 -> 11.9 N (23.5 V) / 11.3 N (22.0 V); 0.9 -> 13.8 / 13.2; 0.95 -> 14.8 / 14.2.
The pair's hover is 69-72 % of the 0.95-cap thrust; at the real.yaml default cap 0.8 it is 86-91 %.

The model matches the rig: RIG-1029-sq1 (the T's opposite pair on the 1.15 kg ring) flew 0.82-0.90 against the model's
0.80-0.84, the square on it (h4) 0.57-0.62 mean against ~0.62.

Pair flights in the registry (all on 0.55 m cables; no rig pair has flown 1 m cables or the M2A ring since 4 Nov):
- 22 Oct (QUAD3/QUAD4, ema then slope): h2 throttle p50 0.74-0.75, p95 0.80, on the 0.80 cap 7-8 % of the lift,
  QUAD3's magnet let go at 6.4 N peak, DROP AND LAND; h2t9p, h2m, h2rts a 1.3-1.6 Hz see-saw that grew with ema;
  h2slope and f2slope (slope filter, now the default) calm, f2slope the first full rig figure-eight with two quads,
  the ring hanging ~7 deg about the pair's line and 14 deg at the bends.
- 28 Oct (QUAD2/QUAD3, slope, cap 0.9, the M2A ring): hS1, hS2, f2a, f2c, ls2a held; f2b sank 166 mm at a bend,
  ls2c 537 mm at the line end. From the tracker logs (`thr_out`, this study): hS2 p50 0.72-0.75, p95 0.77-0.79;
  f2c p50 0.72-0.74; f2b drone 1 p95 0.90 and ls2c p95 0.88-0.90, i.e. ON the 0.9 cap during both sinks.
  The registry's "throttle peak 0.57-0.69" for these rows is the throttle above the offset (`thr_out - thr_off`),
  not the output: worth a correction row (lab chat).
- 29 Oct (1.15 kg ring, 1 m): no pair reached (ch1, ch1b, ch1b_2, sq1).

Reading: on the 0.86 kg ring with 1 m cables the pair should hover at ~0.72-0.78 with fresh-to-mid packs, under the
0.95 cap the scripts type (`throttle_max:=0.95`), with ~0.2 throttle (~40 % thrust) left. Paths are where the 0.55 m
pair hit the cap. So: hover only, `throttle_max:=0.95` typed, packs at or above 22.6 V at rest. Each pair magnet
holds ~4.9 N static, twice the square's 2.4 N; rig magnets have let go at 2.8-6.4 N (RIG-1029-h4, RIG-1022-h2) against
~10 N rated.

## 9. Effort (our stack only; his launch and the rig calibration are in the other two files)

| Work | Items | Hours |
|---|---|---|
| Planner per-partner refactor | 1-11, 14-15 | 10-12 |
| Set detach (or the queue: 1 h) | 12 | 3-4 |
| Fold-in from the current blend (or queue) | 13 | 1-2 |
| Launch graph: partner list and file, chain in a loop, refusals | 16-24 | 8-10 |
| Nodes: mux topic, tips, rig_io, fleet_viz | 25-29 | 2-3 |
| Console rows and "both" buttons | 31 | 3-4 |
| RViz panel (optional if the console flies the partners) | 32 | 3-4 |
| Params file, script, rig_flight.sh, sheet lines | 33-37 | 2-3 |
| Unit tests (planner per-j, set detach, blend, launch refusals; `test_real_mode_launches.py` has 121 tests to extend) | - | 5-6 |
| Twin: two-partner sim branch of the graph + world | 3b note | 6-8 |
| Verification: pytest, `gate.sh --quick`, critic (new release path and safety gates) | - | 3-4 |
| **Total** | | **~46-60 (minimum without the RViz panel and with both queues: ~40)** |

Twin runs on top (lab PC): 4 -> 2 set detach, 2 -> 4 serialised, 2 -> 4 overlapping, LAND with both out, a drop with
both out; one each (exploratory), ~30 min each.

## 10. Biggest risks

- **R1, the pair cannot hold plates 3 and 9 still.** The pair's cables meet the ring on its own line (0-6), so they
  give no moment about it; only the centre of mass hanging ~1.75 cm below the cable ends resists (M2A `attach_z`
  0.0175, if the Motive pivot is the centre of mass): about 0.86 x 9.81 x 0.0175 = 0.15 N m/rad. His descent targets
  the tip 3 cm below the contact point (`contact_dz:=-0.03`); with nothing holding plate 9 up, the plate follows: about
  asin(0.03 / 0.225) = 8 deg of roll, and plate 3 (the other returner's target) rises 3 cm. His lift proof then rolls
  it the other way. With both pressing together the moments cancel only while they are equal. In the triangle today
  plate 9 sits between two carriers, which is why the contact works there. Evidence so far: the rig pair already
  hung ~7 deg about its line and swung to 14-16 deg (RIG-1022-f2slope, 28 Oct rows). The twin's R2829 welded onto the
  pair, but its catch is a joint made at contact. Before the rig: score R2829's tilt across its contact; on the rig,
  a pair hover first, then one dry descent (coil held OFF, `dry_hold`) onto plate 9 of the hovering pair to measure it.
- **R2, single-partner state left anywhere** (sections 1 and 3): one shared topic or flag hands the second quad to
  the wrong controller or leaves it on his stream through a LAND or a drop. Every item in 3a/3b P0 needs a unit test
  that drives two partners through LAND, drop, give-up and a proof of the other.
- **R3, overlapping fold-ins** (#13): never flown in either world.
- **R4, the pair on 1 m cables and the M2A ring has never flown on the rig;** 0.55 m pairs reached the 0.9 cap on
  paths; magnets carry twice the square's load.
- **R5, his stack twice:** QUAD3 has never been flown by his MPC (his thrust ratio 22 is QUAD4's); the two instances
  do not see each other (his carrier model is static, typed carriers), so the routes must stay on their own sides;
  two of his MPCs, planners and backends plus our four trackers and planner on one laptop (CPU; see
  `tejen_stack.md`).
- **R6, calibration and hardware on plate 3:** a second funnel, body 23 redefined at QUAD3's tip, its W/X/Y/L/K
  record; a wrong record puts the contact target on the wrong point (refusals per partner, #23).

## 11. Decisions this needs from Wesley

1. Layout and quads: pair QUAD1/QUAD2 on 0/6, QUAD3 on 3, QUAD4 on 9 (or remap radios to keep another quad as a
   carrier).
2. Detach: build the set detach (recommended: symmetric unload, no T) or queue two DETACHes through the T.
3. Re-attach: serialise the fold-ins first (recommended), overlapping blends only after the twin scores them.
4. The chain rules in attach mode (gate thresholds): `min_survivors 2`, `drop_gap_deg 200`, drop tilt 45 or 55.
5. Partners flown from the console only, or the RViz panel extended too (+3-4 h, C++).
6. Hover only for the whole test (recommended: the pair's thrust and the contact physics, R1, R4).
