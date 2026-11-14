# C: decisions and rules against the actual defaults (read-only audit, Thu 5 Nov 2026 ~22:50)

Tree read: branch week8 at bf52a8a plus the working copy (only docs/ and results/registry.csv dirty). Nothing was edited,
run or committed. Sources read: CLAUDE.md, docs/decisions.md (all 827 lines), src/bringup/config/{common,sim,real,sim_legacy}.yaml,
src/bringup/bringup/{profiles,real_mode,graph_attach,graph_detach}.py, the launch files, node `declare_parameter` defaults
(planner, dissipative node, tracker, fleet manager, magnet manager, mocap nodes), src/mpc_planner/mpc_planner/params.py,
tools/make_carry_worlds.py, configs/rig/*.yaml, the Wed 11 / Thu 12 sheets and scripts, the memory folder, the five
bookkeeping files and the registry.

Marking: CONFIRMED = both sides read at the quoted lines. UNSURE = the code side is clear but whether it is a drift
depends on a decision that is not written down (or on intent).

A note on node defaults: since the 3 Oct launch consolidation (decisions.md:83) the node `declare_parameter` defaults are
mostly the pre-decision values (e.g. dissipative_node.py:1063 `drop_on_loss` False, :966 `departed_land` False,
planner_node.py:811 `int_mode` 'reference', magnet_attachment_manager.py:210 `proof_rules` 'contact_lift'), and the decided
defaults live in the profiles. Every launch goes through bringup/profiles.py, so these are by design and not listed as
drifts. The two decisions that said "launch + node" (decisions.md:46, :49) do match the node defaults
(dissipative_node.py:616-632, planner_node.py:363).

---

## Most important first

### For the rig days (Wed 11, Thu 12 Nov)

1. **A match_dwell 1.0 script is still in Wednesday's folder** (CONFIRMED). decisions.md:826 (5 Nov ~21:48): "No match_dwell 1.0
   try ... Thursday's nrt line and results/rig/2026-11-12/scripts/rig_nrt.sh removed". But
   `results/rig/2026-11-11/scripts/rig_nrt.sh:32` still types `match_dwell:=1.0`. The Wed sheet copies every script to ~
   (`docs/rig/wed_2026-11-11.md:56`, `cp results/rig/2026-11-11/scripts/rig_*.sh ~/`). It still offers "nrt / nrtf" in its
   order table and fallback (`wed_2026-11-11.md:46`, `:49-50`). The "17 OK" desk check counts this script too
   (13 Wed + 6 Thu scripts, less the two T1 scripts).
2. **Thursday's sheet still shows the old k1z numbers in two places** (CONFIRMED). decisions.md:812 (3) and the sheet's own
   call 1 (`thu_2026-11-12.md:9`) say drop_height 0.28 and max_body_z 2.8, with QUAD4 planned at 2.56 / 2.56 / 2.76 m.
   `rig_k1z.sh` types exactly that. But the bold table row at `thu_2026-11-12.md:17` still labels **"1.2 (k1z)"** with
   0.48 / 3.0 / 2.96 m. And the desk-check paragraph (`:141-144`) reads back "k1z ... 2.76 / 2.76 / 2.96 m under max_body_z 3.0".
   An operator checking the launch's printed heights against the sheet will see them disagree.
3. **The console's `--slot-gate` dropped off the sheets with no decision** (CONFIRMED). decisions.md:223 (16 Oct, item 4): "Console
   --slot-gate ON at the rig (typed on the sheet; code default off)". The Wed and Thu sheets (and the 4/5 Nov ones) give
   `python3 tools/console.py --small-angle 90` with no `--slot-gate` (`wed_2026-11-11.md:67`, `thu_2026-11-12.md:61`). The code
   default is still off (`tools/console.py:697`). The sheet's manual "slot error" check (`wed:71-72`) may be meant to replace
   it, but no decision line says so.
4. **The release line flies the magnet-marker detector, with no recorded decision for real flights** (UNSURE).
   `rig_rel.sh` reads `configs/rig/m1_release_tip_m2a_det15.yaml`, which sets `tip_detect_m: 0.04`. decisions.md:302-309
   (21 Oct) put the check on the rig tree "as a switch (tip_detect_m, default off)". The card's rule
   (docs/experiments/2026-10-21_tip_detect.md:231, :329) is "Part C defaults to OFF; Wesley decides". decisions.md:752 (2)
   types it only for X3b's simulated 8-9 carrier chains. The rig use first appears in the lab chat's 30 Oct sheet note
   (docs/archive/session_state_to_2026-10-30.md:186-191), not in decisions.md. It is a detection gate, so it needs Wesley's word
   under CLAUDE.md's "What needs Wesley's word".
5. **The contact (catch) radius differs on every side** (CONFIRMED; it touches the thesis too, see below).
   The rig claim line `rig_nrs.sh:11,19` types `weld_radius` 0.025 (K), as decided at decisions.md:776.
6. **The rig attach mode forces the M2A ring's rest height** (CONFIRMED, low impact). `real_mode.py:42` RIG_DEFAULTS sets
   `payload_rest_z` 0.0275 (the M2A ring, 22 Oct), and `apply_rig_values` (called for rig attach at `graph_attach.py:386`)
   uses it unless the value is typed or in the params_file. real.yaml:25 says 0.013 (the 1.15 kg ring, decided 28 Oct,
   decisions.md:610-614), so a rig attach or m2 launch on the 1.15 kg ring silently gets 0.0275. It does not matter for
   Wed/Thu: every M2A params file types 0.0275 itself, and the value only shifts the trackers' "ring resting" test, which is
   z <= rest + 0.05 (tracker_node.py:929). The ring rest measured 4 Nov was 0.0237 (SESSION_STATE.md:138): also immaterial.

### For the thesis

1. **The catch radius and co-motion constants in the thesis are not the ones flown** (CONFIRMED). decisions.md:731-732 (30 Oct): "the
   thesis states the catch radius as 45 mm (the next attach flights type weld_radius 0.045)". Thesis
   `~/thesis_full_draft/Source/src/reconfiguration.tex:145` says "Catch radius 45 mm, face -15 to +12 mm, under 0.03 m/s for
   0.3 s"; `:146` says "tip within 6 mm ... taut within 3 mm ... 45 mm separation". What was flown:
   - The simulated 4.3 join claims type `weld_radius: 0.01` (e.g. `configs/experiments/fri30_sq6b.yaml:63`,
     `sq_floorjoin_hover_L100_p115_noxy_low.yaml:55`). The twin's co-motion contact radius is `weld_radius`
     (`graph_attach.py:236-238`), and the twin proof runs without plate motion and with a 15 mm normal (`:244-247`).
   - The real-world claim line flies 0.025, with the loose co-motion set of decisions.md:767 (drift 0.02, |normal| 0.04, taut
     margin 0.05, 0.06 m/s, 0.5 s dwell, 6 s inconclusive, 0.06 loss and separation; `rig_nrs.sh:19-23`).
   - On 4 Nov the rig flew 0.06 (decisions.md:767), then the radius went back to 0.025 (decisions.md:776).

   No decision line reconciles the 30 Oct "45 mm" with what was flown. The handin_plan still lists "the confirmation tolerance
   (settled after the first real join)" as open (`handin_plan.md:89`).
2. **The real-world 1 m rows will be on the 0.86 kg ring, not "Setup 2"** (feed gap; CONFIRMED that no feed entry exists).
   decisions.md:812 (1) (5 Nov ~14:40) says the 1 m baselines and the release pair fly the 0.86 kg M2A ring, "the text says
   which ring". The thesis defines Setup 2 as 1.15 kg / 1 m (`system_overview.tex:4, :98`; decisions.md:758). No feed entry
   (F048-F052) carries the 14:40 decision. F050 covers only the LAND defaults for the next lab days.
3. **The height integral's bound has no decision line** (CONFIRMED record gap). decisions.md:72 (30 Sep) says the bound is 0.15 m,
   but `common.yaml:26` sets `z_i_max: 0.3`, citing "Wesley 4 Oct" only in a comment. No decisions.md line records 0.3. The
   thesis (`sim_robustness.tex:29`) already speaks of "the earlier bound of 0.15 m".
4. **The 14 Oct fold-in decision never happened** (CONFIRMED). decisions.md:217 (14 Oct) says "the planner folded into mpc_planner,
   mode dissipative -> detach, the old network controller to the `legacy` branch". The code still has the
   `dissipative_planner` package and `MODES` 'dissipative' and 'network' (`profiles.py:23-25`). The plan's WP7 is "to do"
   (`docs/archive/plan_after_rig_2026-10-14.md:19`). From 22 Oct the twin network mode was revived for the comparison with
   Quan et al. No decision line withdraws or defers WP7. The memory note no-dissipative-wording still says "once the fold-in
   lands".

---

## 1. Locked decisions and valued rules against the real defaults

Consistent (checked, no drift):
- Rig thrust map 0.185 / 0.022 / 35.2, kt_trim off: real.yaml:46-52 and sim.yaml:44-50 (CLAUDE.md "Thrust").
- pos_limit_m 1.75 on the rig, off in the twin: real.yaml:56, common.yaml:112.
- Rig paths 0.75: real.yaml:41-42.
- drop_on_loss on in every mode: common.yaml:128. The attach graph no longer forces it off; it adds only
  `departed_land: False` (graph_attach.py:652-657).
- drop_tilt_deg default 30 (tracker_node.py:1001); min_survivors 3 (common.yaml:117).
- drop_gap_deg 200 with min_survivors 2 and tilt 45: in all 7 configs/rig/chain_pair*.yaml (chain_pair_off types gap 0).
- drop_takes_back typed through every M2A attach params file (e.g. attach_m1_1m_m2a_drop55_det15.yaml:7).
- newcomer_lifecycle, newcomer_fault_land and departed_ground_idle true: common.yaml:214, :217, :127. The legacy profile
  keeps them off (sim_legacy.yaml:41, :45-46).
- Rig LAND defaults: real.yaml:30-39. The launch refuses carriers_down_alone without land_descend_to_stall
  (real_mode.py:173-177). The band is 0.03-0.20 (real_mode.py:267). The twin sets only the re-arm (sim.yaml:28).
- Band-stop on the rig: real.yaml:78-80 sets cable_notch_hz 5.3, Q 1 and cable_lp_hz 0. Both are off in common.yaml:95-98.
- Velocity filter slope by default: mocap node motion_capture_publisher_node.py:464; twin payload_mocap_emulator.py:59. On
  the rig in attach mode QUAD4 keeps ema (real_mode.py:354-356). rig_flight.sh:79-85 unsets the env switches, so the code
  default (slope) flies.
- Twin links: make_carry_worlds.py:53-58 gives ROD 0.002, MAGNET 0.038, AIRFRAME 0.510. sim.yaml:42 rod_level_mass 0.040
  and sim.yaml:14 drone_mass 0.55 agree.
- The plan is massless: sim.yaml:41 `plan_rod_level: false`. "Typed, the rod sets int_k_xy 0 unless chosen; real flights
  refuse it": real_mode.py:193-205.
- weld_vel_filter_s 0.1 in the twin: sim.yaml:56.
- int_mode auto, int_k_xy 5.26, int_k_z 7.13: common.yaml:67-69.
- Attach at 0.2 m/s: approach.py:15 APPROACH_VEL 0.20.
- Plates clockwise; carriers typed 90,330,210: real.yaml:97, sim.yaml:86.
- contact_still on, proof_rules comotion: common.yaml:279, :245.
- Transit low (attach): the rig ceiling is in partner_mission.launch.py:160 and :192-216. Untyped floor_join defaults to the
  'over' route (:174-178), but every claim config and rig script types the low heights.
- cable_accel_cap 14.5 (common.yaml:94); cable_elev_deg and handover_elev_deg 60 (common.yaml:15, :23); detach_step_out_m 0.5
  (:74); detach_detect_m 0.06 (:124); x0_relax_frac 0 (:101); land_unwind true (twin) and false (rig) (common.yaml:34,
  real.yaml:29); land_ff_ramp and ground_idle on (common.yaml:35, :103); fig8_size 0.9 (common.yaml:31);
  measure_rod_len true (:11); magnet channel 6 (:303); sim clock_hz 500 (sim_io_launch.py:248); sim rate_ki 5
  (payload_betaflight_comm.py:122-123); rate_source imu (common.yaml:298).
- Deleted switches are really gone (no hits in src/ or tools/): land_height_gate (decisions.md:244), land_slow_vel / land_slow_z
  (:814), detect_skew_s (:807), rate_kff / thr_tau_s / the body-rate filter / the tip-hinge world (:473-475). foldin_cap
  'weld' is refused in two places (real_mode.py:208-210, dissipative_node.py:621-623).
- Tejen side: drop_puck_wait_s 2.0 (partner_mission.launch.py:1160); mpc_profile oct10 (:1246); his rig object_mass_kg 0
  (:1098); the tracking-check half 0.20 (c1f6_moving_rendezvous.yaml:151-153); GUARD_SLACK_M 0.12 and GUARD_WAIT_S 3.0
  (dissipative_node.py:186, :172); DEPARTED_CLEAR_R 1.2 (:64).
- Every Wed/Thu script carries a 0.86 kg params file and the M2A solver folder (all 17 launch scripts checked), so the 4 Nov
  mistake of flying the 1.15 kg config on the M2A ring cannot recur from these scripts.

Drifts:

| # | Decision / rule (file:line) | Actual default (file:line) | Status |
|---|---|---|---|
| 1.1 | Ring CoM rest is the rig default 0.0275 (decisions.md:394, 22 Oct); new payload rest 0.013 is the real-world default (decisions.md:610-614; real.yaml:25) | real_mode.py:42 RIG_DEFAULTS payload_rest_z 0.0275 overrides real.yaml in rig attach and m2 unless typed (real_mode.py:99-109, graph_attach.py:386, graph_detach.py:60). The rig carry and detach modes read real.yaml (real_dissipative does not call it) | CONFIRMED, low impact |
| 1.2 | Height integral bound 0.15 m (decisions.md:72) | common.yaml:26 z_i_max 0.3 ("Wesley 4 Oct", comment only); params.py:124 node default 0.3 | CONFIRMED (no decision line for 0.3) |
| 1.3 | Console --slot-gate on at the rig (decisions.md:223 (4)) | sheets omit it (wed:67, thu:61); code default off (console.py:697) | CONFIRMED |
| 1.4 | No match_dwell 1.0 try; rig_nrt.sh removed (decisions.md:826) | results/rig/2026-11-11/scripts/rig_nrt.sh:32 match_dwell:=1.0 is still there; wed sheet :46, :49-50 still name nrt | CONFIRMED |
| 1.5 | k1z at drop_height 0.28, max_body_z 2.8 (decisions.md:812 (3)) | rig_k1z.sh agrees; thu sheet :17 (bold k1z row 0.48 / 3.0) and :141-144 (read-back 2.96 under 3.0) do not | CONFIRMED (sheet text) |
| 1.6 | S's speed-ups become the twin default after a smoke run and identity tests (decisions.md:420-421 (20)) | common.yaml:52 plan_fast_refs '' (off); sim.yaml does not flip it. The smoke run was done 23 Oct (scalability card :493-500); the card's section 8 checks were "not checked here"; "flips only between claim tables" | UNSURE (a decided flip left pending; the frozen claim tree flies off except where typed) |
| 1.7 | M1 weld point on plate 9 at the magnets' radius (decisions.md:115; real.yaml:94-99) | real.yaml:98-99 attach_x/y_offset 0 / 0.225 with real.yaml:16 attach_radius 0.204 (the 1.15 kg ring) | UNSURE, latent: every attach script types X/Y |
| 1.8 | CLAUDE.md:88-89 "In attach mode the newcomer keeps ema" | true on the rig by code (real_mode.py:354-356). In the twin only when the config lists MDC_MOCAP_VEL_DRONES (payload_mocap_emulator.py:82-86); a bare twin attach launch filters QUAD4 with slope too. The claim configs list the carriers (e.g. sq_floorjoin_..._noxy_low.yaml:8) | CONFIRMED, low impact |
| 1.9 | Rig rel line with tip_detect_m 0.04 (configs/rig/m1_release_tip_m2a_det15.yaml) | decision only "default off" (decisions.md:307-309); no line approving rig use | UNSURE |
| 1.10 | Ring-tilt drop at 55 deg "for the square's detach and re-attach (sq9)" (decisions.md:799) | also typed on the triangle lines nrtf and nro (attach_m1_1m_m2a_drop55_det15.yaml, attach_m1_1m_m2a_orbit_det15.yaml); flown that way on 5 Nov (nr9m, nr9o) with Wesley in the lab | UNSURE, minor |
| 1.11 | params.py "needs his word"; Sim plant = rig twin (decisions.md:70, :102) | params.py:12-26, :46 module defaults are still the legacy x3 plant (CABLE_LEN 0.5, ATTACH_RADIUS 0.25, DRONE_MASS 0.64, PIVOT 0). They matter only for a node started without the launch. The 22 Oct additions (decisions.md:401-402: load_ixx/izz, prebuild_sizes, fig8_size) are present (params.py:51-57, :83, :159) | note, not a drift |

## 2. Contradictions, OPEN lines decided later, stale CLAUDE.md rules

### 2a. DECIDED lines that disagree, with nothing resolving them
- **Catch radius** (CONFIRMED): 45 mm for the thesis and 0.045 for flights (decisions.md:731-732) against the 0.06 typed on
  4 Nov (:767) and "stays 0.025" (:776). The thesis says 45 mm (reconfiguration.tex:145); the twin claims fly 0.01 and the
  rig claim 0.025. See "For the thesis" 1.
- **WP7 fold-in and rename** (CONFIRMED): decisions.md:217 against the 22-23 Oct network-mode work (decisions.md:383-426,
  :552-565) and the code (profiles.py:23-25). Never withdrawn.
- **small_angle** (UNSURE, minor): decisions.md:670-674 ("small_angle 180 on the FCs ... Console: --small-angle 180") against
  the note at :675 (QUAD1 at 90, "his value"). The handin_plan (handin_plan.md:98) and both sheets say QUAD1-4 at 90. No
  DECIDED line records 90 for all four.
- **Critic scope** (UNSURE): CLAUDE.md:101-104 (a critic for a new control law or gate change) against SESSION_STATE.md:145
  "Critic waived for all (3 Nov)". decisions.md:765 waives it "for all" in the context of the re-attach on any plate.
  Critics still ran on 5 Nov (decisions.md:805, :808, :810, :813). Which reading holds is unclear.
- **Setup naming** (UNSURE): decisions.md:758 (Setup 2 = 1.15 kg / 1 m) against :764 (fixed-team set on 0.86 kg / 1 m) and
  :812 (1) (real 1 m rows on the 0.86 kg ring). There is no third setup name, and no line says how the thesis labels
  0.86 kg / 1 m.

### 2b. OPEN or PROPOSED lines decided later but never closed (CONFIRMED unless marked)
| Line | OPEN item | Decided or answered by |
|---|---|---|
| decisions.md:339 | slope filter as the rig default (22 Oct) | :604-616 (28 Oct: slope, both sides) |
| :652 | type Tejen's descent debounce on the rig | typed on every attach script since (rig_nrs.sh: `descent_debounce:=1.0 descent_abort_xy:=0.10`) |
| :655 | 2-quad baselines B2 on 0.86 kg, or skip | :700-704 (30 Oct 13:10: use the 28 Oct pair flights) |
| :663 | the attach gates G, K and give-up | G loosened :767; K :776; tip_skip_repeats, comotion_speed_window_s 0.5 and contact_give_up_s 120 typed on every attach script |
| :668 | h3tw, plan_w_pos_scale 2.0 | never flown (no h3tw row in the registry); corner cutting closed :261-262; not on any later sheet (UNSURE: moot rather than decided) |
| :682 | 4-quad T1 streams 9 bodies | the Wed 4-quad T1 streams 7 (rig_t1_q4.sh: 8,11-14,21,24) |
| :684 | does the heave go with the delay gone? | :798 (5 Nov: the cause was a Motive recording; fm9y_2 clean once stopped) |
| :714 | attach_z of the 1.15 kg ring | closed by :719-723 ("Closes the OPEN of ~14:00"), but the OPEN line itself carries no marker (minor) |
| :771 | four proposals from 4 Nov | (1) :776; (2) and (3) BUILT :772; (4) corrected :773 |
| :794, :797 | PROPOSED ring-low gate numbers | :825 (1) (stays OFF on the rig); the twin default is undecided |
| :7, :9, :30 | September layout, attach_traj_hold_mode settle, SIL vs Gazebo | moot: rig layouts decided (:54, :115, :812); our own approach is twin-only (:145, :162); attach_traj_hold_mode stays 'timed' (common.yaml:193) |
| :187 | P6 offline analyses (7 items) | partly answered :195; the T6 and band items moot since :756 (no frozen predictions) |
| :216 | rig ARM gate on missing pack voltage | still genuinely open (the console greys ARM; the manager arms without it): fine |

### 2c. Stale CLAUDE.md rules (CONFIRMED)
- CLAUDE.md:157-158: "Old worlds ... fly only with legacy:=true; only the network mode still needs it". But
  `profiles.py:27-29` has `LEGACY_ONLY = ()` "none since the twin network mode", and sim.yaml:68-81 flies network on the
  twin. The header comment of sim_legacy.yaml:4 is stale the same way.
- CLAUDE.md:94: the band-stop's "first real four-carrier test is Wed 4 Nov's h4m1". h4m1 never flew (no RIG-1104-h4m1 row)
  and is dropped from the Wed 11 sheet (`wed_2026-11-11.md:23-25`). Four-carrier lifts with the band-stop did fly on
  5 Nov on the M2A ring (sq8, sq8_2, the fm9 lines).
- CLAUDE.md:164-165: the rig launch refuses QUAD4 above 2.6 m "unless transit:=over is typed". Since 4 Nov a typed
  `max_body_z` (2.6-3.0) raises the ceiling (partner_mission.launch.py:211-216, :1217; decisions.md:774-775), and Thursday's
  k1z types 2.8. The 5 Nov exception for the simulated 4.3 pickup cell (:821) is already in CLAUDE.md:165-167.
- CLAUDE.md:173: "`week5` ... is the testing tree from Wed 14, rig and twin". The tree is week8 (bf52a8a, the freeze of
  decisions.md:825 (4)).
- UNSURE: CLAUDE.md does not mention the per-line rig settings flown since 5 Nov: the loss check at 15 cm on every line
  (`_det15`, decisions.md:800, :807) and the 55 deg drop on the attach lines (:799). They are typed per line, not defaults,
  so this may be intended.

## 3. Dead switches (built default-off, never adopted). Nothing deleted.

### 3a. No decision adopted or kept them (candidates under "record it, delete the code")
| Switch | Where the code lives | Decision trail |
|---|---|---|
| partner_tilt_cut, tilt_slow_deg | dissipative_node.py:874; tracker_node.py:805 | decisions.md:157 (D6: off 21-22 Oct, "on from 28-29 Oct if they pass and Tejen agrees"), :179 ("stays off ... until 28-29 Oct"); never revisited; 19 old configs |
| fc_disarm_land | fleet_manager_node.py:191 | decisions.md:157 D6, as above; never adopted |
| warm_fault_not_live | tracker_node.py:766 | decisions.md:157, :169 ("stays off until after 21-22 Oct"); never adopted |
| partner_stuck_guard | dissipative_node.py:875 | no decisions.md line at all (critic items only) |
| welded_on_load | dissipative_node.py:819 | off; the 5 Nov fix used /drone_3/mux_state instead (decisions.md:805); no adoption |
| dry_hold | graph_attach.py:203, :988 | critic F6 15 Oct; the dry hold d1 was skipped (decisions.md:219 bracket); used by 0 configs |
| approach_tip_feedback, approach_tip_log, attach_tip_dz | dissipative_node.py:609-610 | decisions.md:131, :134 ("ON from the first R8b"); R8b left the rig plan (:145, :162); the 60 U claim configs pin it false |
| cmd_next_tick (env TRACKER_CMD_NEXT_TICK) | tracker_node.py:556-559; graph_attach.py:869 | decisions.md:209 (card A, off; twin arms); no later decision; rig_flight.sh:79-85 strips it |
| Ring velocity slope (MDC_MOCAP_VEL_RING / SIM_MOCAP_VEL_RING) | motion_capture_publisher_node.py:479; payload_mocap_emulator.py:62 | decisions.md:354 ("twin first, then the rig"); never decided; the claim configs pin ema |
| cable_pull_up_max | tracker_node.py:240-241, :1765 | built decisions.md:789 (2); its runs 3-6 cancelled (:793); SESSION_STATE.md:120 "its question ... is open" |
| ring_low_land_m / _s | planner_node.py:375 | :794, :797 PROPOSED; :825 (1) "stays OFF on the rig"; no twin decision |
| dist_est (+ dist_tau, dist_rod_mass, dist_rod_com) | planner_node.py:667 | decisions.md:93, :95 ("losing methods stay behind off-by-default knobs until he decides"); undecided since 4 Oct |
| offset_free (+ of_tau, of_window) | planner_node.py:756 | decisions.md:95, :99; undecided since 4-5 Oct |
| pin_cable_rates | planner_node.py:472 | no decisions.md line |
| int_settle_s | planner_node.py:816 | no decisions.md line |
| x0_rate_source_drones | graph_attach.py:265-270 | decisions.md:62 (carrier-kick oracle test, 28 Sep); never adopted |
| w_thr_ref, vel_swing_k | tracker_node.py:690, :625 | decisions.md:96 (w_thr_ref 10 "did not move it"); vel_swing_k has no line |
| mpc_profile oct14 (Te-Jen's fork) | partner_mission.launch.py:1246 | decisions.md:211, :214 (flown once, "back to oct10"); never adopted |
| mpc_command_execution partial_step (fork) | partner_mission.launch.py:1248 | decisions.md:648-651 ("does not fly on the rig for now"); handin_plan.md:193 re-test pending |
| mpc_battery_comp (fork) | partner_mission.launch.py:1218 | built 4 Nov (SESSION_STATE.md:114); R3679 PASS; no decisions.md line |
| net_traj_lean | dissipative_node.py:519 | no decisions.md line |
| net_twin_geometry | dissipative_node.py:524 | sim.yaml:80 "approved on, off until the card's critic"; never flipped (UNSURE: may be pending, not dead) |

### 3b. Kept by an explicit decision (record only; the rule's exception)
- ref_time_shift, plan_ref_lag_s, plan_w_pos/s/r_scale, plan_rate_ff (planner_node.py:474-478, tracker_node.py:548):
  "its switches stay exploratory" (decisions.md:261-262). plan_sqp_iters is used by S and X3b.
- plan_rod_level with rod_level_mass/com and rod_mass/rod_com (planner_node.py:459-463): reversed at :790 and :792; "the
  pinned configs and the coupling stay" (:790). 5 claim configs pin it false.
- foldin_cap 'level' (dissipative_node.py:620): kept OFF as an exploratory finding (:817).
- cable_lp_hz / cable_lp_order (tracker_node.py:242): "the low-pass off; kept in the code" (:632-634).
- x0_relax_symmetric (tracker_node.py:541): "stays a diagnostic flag" (:23).
- kt_trim and thrust 'auto': legacy profile only (decisions.md:102; sim_legacy.yaml:31-36).
- Pending, not dead: rod_len_reread (Thursday, if R1 holds; :751), partner_release_around (flown on the rig 5 Nov as nr1r;
  not on the Wed/Thu sheets), plan_fast_refs (see 1.6), and the per-channel command delay (kept as a testbed, :475).
- Twin-only experimental arms that thesis rows use: resize_delay_s, net_law quan, net_damp_relative, net_hold_carry,
  rejoin_departed, rejoin_task, relay_task, traj_laps.
- Leftover worktrees: .claude/worktrees/agent-a4a6813c25d6d67f3 (5a7ccd0) and agent-a6ad05d91f3a03b90 (284ffb4). The first
  may hold the unmerged zero-minimum-pull code that decisions.md:749 keeps there. The second is unexplained (UNSURE).
  Branch cleanup is Wesley's script, by memory rule.

## 4. Stale memory (~/.claude/projects/-home-wesley-multi-drone-control/memory/)
| Note | Stale fact | Now |
|---|---|---|
| hardware-attach-launch.md, and its MEMORY.md line | "the rig attach/M1 launch is three_attach_launch.py real:=true" | that file no longer exists; rig attach is `real_control_launch.py mode:=attach` (decisions.md:83; src/bringup/launch/) |
| project_overview.md | "Active work is the controller_mpc_multi package ... FREQUENCY_HZ in main.py/controller_mpc.py"; "PhD project" | no such package (src/ has mpc_planner, tracker, dissipative_planner, ...); a master's thesis on the carried ring |
| goals-doc.md, and its index line | docs/GOALS.md "(gitignored, like all of docs/)", "re-read it every loop iteration" | moved to docs/archive/GOALS.md; docs/ tracked since 12 Oct; the loop is over |
| thesis-plan-and-gate.md | thesis plan `docs/thesis_plan_2026-10-11.md`; "No geofence, by decision" | plan moved to docs/archive/; current plan docs/plan_rescope_2026-10-30.md; the rig has the 1.75 m position limit (decisions.md:445) |
| no-commits-without-asking.md, and its index line | "exception: the GOALS loop commits on branch integration; rig work on real-world-testing"; description "Claude files are gitignored" | branches long gone (week8); the root CLAUDE.md has been tracked since 9 Oct (said in the body, not the description) |
| fixed-thrust-ratio.md, and its index line | sim "thrust_ratio:=auto" secant gain, "Sim launches use 32.9 flight / 30.0 takeoff", "Sim attach runs capsize until that is done" | the Gazebo throttle was linearised 24 Sep (decisions.md:20); the twin flies the rig's affine map 35.2 / 0.185 (sim.yaml:44-50, :59); 'auto' only in sim_legacy.yaml:31 |
| payload-disc-spec.md | "keep attach_radius 0.25"; "params.py, launch defaults ... all 0.86 kg"; "real.yaml attach_z 0.025 likely high" | M2A 0.225 (ring_m2a.yaml:7, sim.yaml:15), the new ring 0.204 (real.yaml:16); real.yaml load_mass 1.15; attach_z 0.028 (real.yaml:17). It also misses that every flight since 4 Nov, and Wed/Thu, fly the M2A ring (decisions.md:812) |
| quad4-never-over-carriers.md, and its index line ("rig refuses > 2.6 m") | 2.6 m hard ceiling | typed max_body_z up to 3.0 (partner_mission.launch.py:211-216), 2.8 for the pickup (decisions.md:774); the twin's 4.3 pickup cell flies over (:821) |
| rig-thrust-map-and-geometry.md | "QUAD4 never flies over them anyway (2.6 m launch cap)"; "Drone as flown 0.55 kg (0.475 + rod + magnet)" | as above; twin airframe 0.510 + 0.040 (make_carry_worlds.py:53-58); rig drone_mass 0.583 at 1 m (real.yaml:10) |
| dissipative-only-flight.md | `dissipative_only_launch.py` in controller_quad_load/launch; link [[adaptive-thrust-ratio]] | gone; the network flies through `sim_control_launch.py mode:=network` (sim.yaml:68-81); no such memory note |
| dissipative-detach-controller.md, and its index line | "own package controller_dissipative ... branch dissipative-detach-controller ... origin/main 15 behind" | renamed dissipative_planner (decisions.md:81); old network kept on the `legacy` branch (:217) |
| drone-attach-integration.md, and its index line | "now branch term3 ... all UNCOMMITTED as of 2026-09-09" | history; the work is on week8 |
| m2-handover-state.md, its index line | "open: I-term default, universal-joint tethers" | decided 26-27 Sep (decisions.md:41, :45, :47; the body's update says so, the index does not) |
| gazebo-experiment-runner.md | "rviz_quad_load_launch.py owns the /clock bridge ... three_attach_launch.py"; test path src/controller_load_mpc/... | sim_io_launch.py / sim_control_launch.py (decisions.md:83) |
| analysis-plotting-pipeline.md | palette "imported from controller_quad_load/rviz_config.py"; "docs/figures/ is gitignored until promoted to results_archive/" | bringup/rviz_config.py; results_archive removed and docs/ tracked since 12 Oct (CLAUDE.md "How to work") |
| lab-workstation-remote.md | "tools/remote/ does not exist yet" | tools/remote/ (lab_batch.py, Dockerfile) is the lab path |
| thesis-page-limit-not-strict-yet.md | "the draft will change a lot before 27 Nov" | due 20 Nov (decisions.md:158) |
| cable-load-controller.md index line; package-rename-2026-10-03.md | "see CURRENT_STATE.md" / "Full table in CURRENT_STATE.md §12" | retired 11 Oct; the map is in docs/reference.md §12 |
| lift-system-anchor-pose-fix.md | "controller_mpc_multi/launch/mpc_two_no_cables_launch.py" | gone |
| m1-week-plan-2026-10.md (index: "no freeze") | "no rig code freeze", "until the 7 Nov results freeze", tip feedback from R8b | code frozen 5 Nov (bf52a8a, decisions.md:825); real-world freeze Thu 12 / 19 Nov; R8b dropped |
| branch-cleanup-rule.md | "back on week5" | week8 |
| current-state-doc.md | "general/CLAUDE.md ... untracked local files" | general/ retired 12 Oct |
| no-dissipative-wording.md | "mpc_planner once the fold-in lands" | the fold-in (WP7) never happened (see 2a), so the note waits on an unfulfilled decision |

## 5. Two-chat bookkeeping
| File:line | Issue | Evidence | Status |
|---|---|---|---|
| handin_plan.md:9-11, :35-37 | Direction still reads "First, before the code freeze: fix the square-to-T settle ... X2-full and X8 wait for it", "Push week8 once the T fix is merged" | the T is handled by the int_k_xy 0 fallback (decisions.md:809 (1); sim_runs_todo.md:14), the rod fix was reversed (:790-792), and the freeze is committed (:825) | CONFIRMED, done but not marked |
| handin_plan.md:25 | "QUAD4's fold-in gets the rejoin's weld-height cap" open | decided 5 Nov 15:57 (decisions.md:817: 'level' kept off, 'weld' deleted) | CONFIRMED, done but not marked |
| handin_plan.md:101-111 (S1, Wed 4 Nov) | all unticked, and unlike S2 (:120-123) no "flown instead" line | W/L re-measured (4 Nov 08:44 and 17:52, decisions.md:779); the first real join flew (RIG-1104-sq6m, feed F039; sq6b dropped, RIG-1104-sq6b); h3t, h4m1, bench latency and release x2 were not flown (no rows); ":110 Catch radius 45 mm" not what flew | CONFIRMED |
| handin_plan.md:152 | [x] code freeze "9f20f8b + uncommitted" | the tree is now committed as bf52a8a (SESSION_STATE.md:3) | minor |
| sim_runs_todo.md:12 | rule "plus one 1.15 kg cell per table" | decisions.md:809 (3): no more 1.15 kg cells | CONFIRMED stale |
| sim_runs_todo.md:49-50 | X2's 1.15 kg cell and its hover / spinny-line cells "Wesley's call" | dropped (decisions.md:809, "X2's unwritten hover and spinny-line cells dropped"; no more 1.15 kg) | CONFIRMED, should be [-] |
| sim_runs_todo.md:69-70, :102-103 | "the per-case wrapper ... is gone: it must be rewritten"; "tools/x8_metrics.py was never written" | both exist (tools/x3_metrics.py, tools/x8_metrics.py; decisions.md:811) | CONFIRMED stale |
| sim_runs_todo.md:88-89 | [ ] ground-computer 8- and 9-quad it1 runs | tried 5 Nov: R3777/R3778 VOID, the laptop cannot run them (registry; SESSION_STATE.md:19-21) | CONFIRMED, should be [-] or re-planned |
| sim_runs_todo.md:125-126 | gap table "flies only on Wesley's word" | given: decisions.md:809 (2) "chunk 9 on" | CONFIRMED stale |
| sim_runs_todo.md:128-130 | 4.3 joins "Wesley's call", old `_noxy` names | decided :809 (1); renamed `_noxy_low` / `_noxy_over` (:818-821; claim_batch_2026-11-07.md:45) | CONFIRMED stale |
| sim_runs_todo.md:138-140 | switch-of-solvers count "Wesley's call" | taken on the recommendation (:809, last sentence) | CONFIRMED stale |
| SESSION_STATE.md:125-142 | old 4 Nov sections: "Running: Nothing. Laptop quiet", the "To do" list, and "Waiting for Wesley" (rod flip, weld filter, clamp, X2 bar...) | contradict the RUNNING section at :6-24; the waiting items were decided 4 Nov 22:20 (decisions.md:784-789) | CONFIRMED |
| results_feed.md (after F052) | no entry for decisions.md:812 (the S3/S4 1 m rows on the 0.86 kg ring; the claim on the square) | F050 covers only the LAND defaults for the next lab days; the thesis keeps Setup 2 = 1.15 kg / 1 m | CONFIRMED gap (UNSURE whether the thesis chat already knows) |
| results_feed.md | X4 "as flown" (decisions.md:827) has no entry yet | SESSION_STATE.md:42 says it is posted once R3774-R3776 are in | pending, fine |
| lab_requests.md | Q001-Q008 all ANSWERED and closed with feed entries | - | OK |
| registry | every run id the todo ticks cites exists (X1, X4, X2-full, X3, X3b, X8, 4.1, 4.2 checked); R3774-R3776 still running | - | OK |

## Counts
- 1. Decision against default: 11 rows (6 CONFIRMED, 4 UNSURE, 1 note). Most rules checked consistent (list above).
- 2. Contradictions and stale lines: 5 DECIDED-vs-DECIDED (2 CONFIRMED, 3 UNSURE); 12 OPEN or PROPOSED lines decided or moot
  but not closed (plus 1 legitimately open); 4 stale CLAUDE.md rules (+1 UNSURE).
- 3. Dead switches: 22 with no adopting or keeping decision (3a); 6 groups kept by explicit decision; 4 pending; 2 leftover
  worktrees.
- 4. Stale memory: 24 notes (or their MEMORY.md index lines) with a wrong fact; about 12 matter (the first 12 rows of
  the table), the rest are old names or paths in history notes.
- 5. Bookkeeping: 13 issues (handin_plan 4, sim_runs_todo 7, SESSION_STATE 1, feed 1); lab_requests and the registry clean.
