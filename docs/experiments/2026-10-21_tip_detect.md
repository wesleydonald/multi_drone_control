# Card: catch a release from the magnet marker (21 Oct; chain step 1b)

Wesley (21 Oct): a release "should detect it based on either the swung electromagnet being in the wrong place, or
the quad being too far away". The planner had only the second check (the cable stretched 6 cm past its length, 2
planner ticks). That check cannot see a quad that carries nothing, such as the T's quad opposite the gap: its release
moves nothing. Every carrier has a magnet marker on the rig (Motive bodies 21, 22, 23, 24 for QUAD1-4). **Bars
FROZEN 21 Oct 20:00**, before any run with the check.

## 1. Question
Does a magnet-marker check catch an unannounced release within about 0.1 s, including the unloaded quad's, without
firing in a normal flight?

## 2. Hypothesis
(Sections 2-8 are rev 1, kept as written; rev 3 at the end supersedes them.)

- **Detection.** A released magnet swings off its plate at once: on 0.55 m cables it hangs ~0.28 m out and ~7 cm
  lower within ~0.4 s, so it passes 4 cm beyond its attached offset in well under 0.1 s. Three samples in a row at
  60 Hz is 50 ms.
  - In the T, `t3_pair_release_tip` is detected within 0.5 s and the fleet resizes to the pair. Without the marker it
    is likely never detected (`t3_pair_release`, R2250+).
  - In the square, the marker catches the release ~0.15 s earlier than the stretch did (R2249: +0.21 s).
- **No false detection.** The attached marker sits at a fixed offset from its plate point, learnt while in band, and
  mocap noise is mm. So a whole floor-start flight never fires.

## 3. The one variable
`tip_detect_m` 0.04 (off by default):
- the planner subscribes `/drone_<d>/magnet_tip_state` for each carrier;
- it measures the distance from the marker to the plate point (the slot's attach point with the ring's pose
  applied), against the marker's attached offset, learnt in band (EMA 0.02 per sample);
- 3 samples in a row past it by 4 cm are a release. The action is the same as the stretch check's (drop or resize,
  per the drop rule).
- **Guards:** a stale ring pose (> 0.1 s); every fresh marker past at once (the ring's pose jumped); LAND (unless
  the drop rule needs it); the creep and lift phases and an unload, as the stretch check.

The stretch check stays on beside it.
- **Twin:** the chain's worlds publish each rod's pose (`--tether-pose-hz` 60), and the mocap emulator reports the
  rod's payload end on `/drone_<i>/magnet_tip_state` (`sim_io` `tether_tips:=true tether_len:=0.55`).
- **Rig:** `real_io_launch.py mocap_extra_bodies:=22:/drone_0/magnet_tip_state,23:/drone_1/magnet_tip_state,24:/drone_2/magnet_tip_state`,
  in drone order. No rig code changes.

Configs: `t3_pair_release_tip`, `t3_hover_tip`, `c4_release9_tip`. Each is its parent config plus `tip_detect_m` and
the emulated markers.

## 4. Baseline
- `t3_pair_release` (R2250/R2251, tonight): the stretch check only.
- `c4_release9_base` (R2249): stretch detection at +0.21 s; ring peak 57.1 deg at +0.6 s.

## 5. Pass / fail numbers (frozen 20:00)

| Run | Confirms | Falsifies |
|---|---|---|
| Unit tests | noise, a blip, a slow drift, a ring-pose jump, a stale ring pose, off, detached and LAND never fire; a 5.5 cm swing fires on the 3rd sample | any of these fails |
| `t3_hover_tip` x1 | no UNANNOUNCED DETACH from TAKEOFF to LANDED | any detection |
| `t3_pair_release_tip` x2 | "UNANNOUNCED DETACH ... magnet" within 0.5 s of the RELEASE, the resize to n=2, no drop or abort, tilt peak < 15 deg over the whole flight, the pair's roll < 5 deg p-p and lean < 5 deg over its last 10 s | no detection, a drop or abort, or a bar missed in either run |
| `c4_release9_tip` x1 | informational: the detection time against R2249's +0.21 s, and the roll peak | - |

**Rig go/no-go for Part C:**
- the unit tests and `t3_hover_tip` pass;
- `t3_pair_release_tip` passes;
- tomorrow's bench reads every marker in Motive (bodies 22, 23, 24) and `/drone_<i>/magnet_tip_state` streams while
  props are off;
- plus the drop-rule card's own conditions.

## 6. Repeats
`t3_pair_release_tip` x2; the others x1.

## 7. Cost
4 laptop runs (~5 min each), recorded.

## 8. New code?
Yes:
- `dissipative_node`: `_tip_check`, `_tip_cb`, and `_on_unannounced` (shared with the stretch check, behaviour
  unchanged);
- the `tip_detect_m` knob (`common.yaml`, `real_mode.PLANNER_OPTIONAL`);
- `generate_rigid_world --tether-pose-hz` (the chain's worlds regenerated);
- `payload_mocap_emulator` `tether_pose_topic` / `tether_len` and `rod_end`;
- `sim_io_launch` `tether_tips` / `tether_len`;
- tests: `test_tip_detect.py`, `test_magnet_tip_emulation.py`.

If it fails, the knob stays off and the code is removed.

## Critic (21 Oct ~20:15): REWRITE. Summary of the must-fix items; the full report is in the session

1. **No rig evidence yet.** Bodies 21-23 have never been seen streaming, and `/drone_<i>/magnet_tip_state` is not
   bagged (`tools/rig_flight.sh:81-90`).
   - Record the markers from Part A on, with the detector OFF.
   - Replay the check over those bags and gate on zero would-fires through lifts, paths and LANDs.
   - Bench: move each magnet by hand, cover each marker for 2 s, record each offset over 3 re-seats.
2. **Invalid samples.**
   - A NaN or a (0,0,0) untracked body must be rejected.
   - Reject any sample whose distance to its own drone is outside the taut length +-3 cm (the guard's band,
     `GUARD_TIP_BODY_M`).
   - Catch rotation glitches as well as translations.
3. **Use the ring-frame vector, not the scalar distance**, as `_release_seen_tick` does. On the rig the marker sits
   ~3 cm above the plate point, so a tangential swing needs ~8 cm before the norm grows 4 cm (~0.17 s).
4. **Learn the offset once**, in a still window after the lift.
   - Bound it to +-1 cm.
   - Write N and alpha in seconds; the rig streams ~100-120 Hz against the twin's 60.
   - Refuse to arm a drone whose offset is over ~6 cm.
   - The unbounded EMA follows a slow peel-off, so that is never a release.
5. **Wire the rig path:** `tip_detect_m` in the params file, `mocap_extra_bodies` on T1, and each drone's offset
   logged after the lift.
6. **Rig prediction.**
   - The plate-3 quad is planned at 0.1 N against a ~0.7 N tether, so OFF may leave its magnet resting on the plate.
     Only the twin's rod, which has no collision, falls away.
   - So "no detection, marker still on the plate" is expected, not a failure. "Marker off its plate in the bag, no
     detection" falsifies.
   - Sheet: with no detection, `/fleet/detach 0` (the announced path, which leaves only an opposite pair) before LAND.

Should-fix 7-15:
- a 0.2 s bar;
- twin marker noise and dropouts;
- the baseline with `tether_tips` on and the detector off;
- contiguous samples;
- the lift guard (the card text is wrong: the lift is not guarded);
- tests for LAND with the ring up, a NaN, and plate 0 in the T;
- planner load (`tools/load_check.py`);
- the regenerated world SDFs need Wesley's word, and the thresholds need re-sizing for 1 m;
- the one-tree rule for the params file.

**Decision for tonight:** the detector stays on branch `tip-detect`, not merged, and is not flown. Tomorrow's rig
only records the markers (detector off) for the evidence in item 1.

## Rev 2 (21 Oct ~23:15), after the critic and Wesley

**Wesley (23:00) on the critic's main point:** a magnet switched off on the rig "will instantly be noticeable and
absolutely detach because of the angle the quad would be at". So on the rig the released magnet is expected to leave
its plate at once, as in the twin.

| Must-fix | Rev 2 |
|---|---|
| 1. Rig evidence | The sheet routes markers 22-24 on every flight with the check OFF, and `rig_flight.sh` bags `/drone_<i>/magnet_tip_state` (main tree cf556ff). Bench: each magnet moved by hand, each marker covered, 3 re-seats. Part C uses the check only if this card passes tonight; the B flights' bags are the first false-fire evidence |
| 2. Invalid samples | Non-finite samples are dropped. A sample whose body-to-marker length is off the learnt one by more than 3 cm (`TIP_BAND_M`) is not this magnet: an untracked body at 0,0,0, or a swap. A ring-pose jump moves every armed marker at once and is not a release (tested by moving the ring's pose) |
| 3. Scalar distance | Now the ring-frame vector from the plate point (`R^T (tip - ring) - rho`), against the learnt vector |
| 4. The learnt offset | Learnt once: the median over `TIP_LEARN_S` 1.0 s of still ring (< 0.05 m/s) after the lift, then fixed. A drone is not armed, with a loud WARN, if its offset is over 6 cm from its plate or its body-to-marker length is over 6 cm from the rod. Counts are in seconds: `TIP_DETECT_S` 0.03 s contiguous, a gap over `TIP_GAP_S` 0.05 s restarts it |
| 5. Rig wiring | `configs/rig/chain_pair_tip.yaml` (`tip_detect_m` 0.04). T1 routes the markers. Each drone logs one "armed: offset ..." line after the lift |
| 6. Prediction and the operator's branch | Prediction (Wesley): the marker leaves at once, so detection within ~0.1 s. Rig falsifier: a would-fire in A/B (from their bags) or no detection within 0.2 s of the OFF. The sheet keeps the `/fleet/detach 0` branch, Wesley's call |
| 11. LAND and the lift | The check is OFF in LAND (the stretch check still runs) and cannot arm before the lift is done |
| 8. Twin realism | 0.5 mm marker noise in the emulator |
| 9. Control | `t3_pair_release_tipoff`: the markers emulated, the check off |

**Bars (rev 2, frozen 23:15):**
- `t3_hover_tip` x1: every drone armed, no detection.
- `t3_pair_release_tip` x2: "UNANNOUNCED DETACH ... magnet" within 0.2 s of the RELEASE, and the resize to n=2.
- `t3_pair_release_tipoff` x1: no magnet detection, by construction.
- `c4_release9_tip` x1: informational, the detection time against R2249's +0.21 s.
- Unit tests: 21 pass (`test_tip_detect.py` rev 2, `test_drop_and_land.py`); the tracker log test passes.

**Also on this branch:** the tracker logs the raw measured cable accel (`acm_x/y/z`) beside the applied one, so peak
pulls above the 6.6 N cap show (Wesley 21 Oct).

## Critic round 2 (21 Oct ~23:00): NO-GO for the check ON on Thu 22; rewrite (rev 3) before it flies

Must-fix for rev 3:
- **M1. A ring-pose tilt glitch fires plate 0 or 6, then DROP AND LAND from 1 m.** In the T, a ~10 deg Motive
  mis-solve about the 3-9 axis for >= 30 ms moves markers 0/6 ~4.8 cm in the ring frame and marker 3 ~1 cm, so the
  "every marker at once" guard misses it. Fix: require the marker to move in the world frame (a pose glitch moves the
  plate point, not the magnet) and/or a ring-quaternion step guard with a hold-off (as the trackers' drop_glitch_step_deg
  15). Add a test on plate 0 that does not stub `_on_unannounced`.
- **M2. Persistence too short.** Time on header stamps, require a minimum sample count, and use 0.06-0.1 s (the rig's
  QUAD4 guard uses 0.1 s).
- **M3. A lone armed marker has no glitch guard.** Act only with at least one other armed, fresh marker; otherwise log
  WOULD-FIRE.
- **M4. Frozen markers.** Reject bit-identical repeats. The bench's covered-marker result is a gate.
- **M5. The rig replay.** Now `tools/tip_replay.py` (7102723). Run it over Thu 22's A/B/C bags (markers recorded with
  the check OFF) and today's QUAD1 marker-21 data.
- **M6. Restore the whole-flight bars:** no drop or abort, no detection on drones 0/2, and the pair's roll and tilt no
  worse than R2250/R2251.

Should-fix:
- S1: the still window, from a window median or position spread, logging "still learning" after ~5 s;
- S2: clear the tip state on attach and detach;
- S3: a load check;
- S4: the twin tip configs fly the rig's params file;
- S5: keep the rod-pose publishers in separate `*_tip` worlds, or get Wesley's word;
- S6: a tighter ring-pose staleness;
- S7: stale comments;
- S8: score in sim time.

The tracker's raw-pull logging (acm_x/y/z): **GO, low risk.** It is ported alone to day-1021.

**Decision (Wesley's condition "if it passes its critic and twin checks"):** Part C flies Thu 22 with the check OFF
(chain_pair_noxy.yaml, the stretch check), and LAND if nothing detects within 2 s. The markers are bagged, so Part C's
bag gives the check its first rig evidence: the detection time it would have had, and false fires replayed
offline.

## Rev 3 (21 Oct ~23:40), after critic round 2 and Wesley

**Wesley (~23:30), against round 2's no-go:** "how often do we get a mocap glitch in our logs big enough to cause
this?" He wants the guards added, the two methods compared tonight, and both available ("both methods should work, it
would just be nice to have both available").

**Our rig logs** (`tools/mocap_glitch_scan.py results/rig/*/*/`: every bag with the ring airborne above 0.3 m):
| Signal | Bags, airborne time | In level flight (ring tilt < 20 deg) | Tilted (after a failure) |
|---|---|---|---|
| Ring pose (Motive, ~88 Hz) | 46 bags, 41.1 min (30.4 min level) | 2 departures > 3 deg from the 0.2 s median, both in the 14 Oct t5_2 crash (a 13.4 deg one-frame step); otherwise frame steps up to 4.2 deg, and over 2 deg only in flights where a magnet let go (c2q_noxy_3/4, c2q_p11p5_2, a2_2, r0013, r200006) or t6f2_4 (2.1); no frozen pose | 7 departures > 3 deg and every frozen run (up to 515 s), the ring at 55-120 deg after a magnet let go or a crash (14 Oct lt_7, t5_2; 7 Oct r0013, r200006; 15 Oct a2_2; 21 Oct c2q_noxy_4, c2q_p11p5_2) |
| Magnet marker (QUAD4's or QUAD1's, /magnet_tip_pose) | 17 streams, 10.2 min | jumps over 0.3 m to a stray marker in 8 streams (0.4-2.8 m), some held > 0.1 s; departures > 2 cm lasting 3+ samples in 4 (at most 2.7 cm) | - |

So outside a crash, a ring-attitude glitch in level flight has not happened in our data; marker swaps have. The body-length band
rejects swaps that land off the quad's tether length.

| Round 2 item | Rev 3 |
|---|---|
| M1 ring-pose glitch | (a) A ring sample that turns over 4 deg or moves over 3 cm from the last, or repeats it bit for bit 3 times, holds the check off for 0.2 s (`_tip_ring_sample`, on the planner's ring callback). (b) A sample counts as past only if the marker's distance from the ring's centre has also moved 1.5 cm: a release swings it out (twin rod: 3.5 cm at 4 cm off the plate; rig cable at 60 deg: ~2.6 cm), and a glitch in the ring's attitude cannot change that distance. Tests: a 12 deg mis-solve about the 3-9 axis held 0.15 s and 1 s (step), and creeping in at 1 deg a sample and held 1 s; markers 0 and 6 read 4.8 cm off. Each fires nothing; with (a) and (b) disabled they fire |
| M2 persistence | 0.1 s on the samples' header stamps (sim time in the twin) and at least 6 samples |
| M3 lone marker | With no other armed marker fresh it logs one WOULD-FIRE per run and does not act. The replay counts these as would-fires |
| M4 frozen marker | A sample repeated bit for bit is no evidence |
| M5 rig replay | `tools/tip_replay.py` runs rev 3, the ring samples through the glitch guard; tomorrow's A/B bags |
| M6 bars | Restored below, whole flight |
| S1 learning | No ring-speed test: a sliding 1 s window after the lift arms once its samples sit within 1 cm (90th percentile) of their median; "still learning" is logged after 5 s |
| S2 attach/detach | The drone's learnt marker is forgotten when it leaves or attaches (`_tip_forget`) |
| S4 params | The twin configs fly the rig's own files: `chain_pair_tip.yaml` (on), `chain_pair_noxy.yaml` (off) |
| S5 worlds | Wesley: the rod-pose publishers go in the existing worlds |
| New: slack cable | The T's unloaded quad may hang slack on the rig. A body-to-marker length up to 15 cm under the rod arms, and the band is one-sided above (the rod + 3 cm): once the magnet lets go and the cable goes taut, its samples stay in |

Twin fault for M1: `MOCAP_TILT -1 deg [s [ramp]]` rotates the ring's published attitude about its body y (the T's 3-9
axis) in the mocap emulator.

**Bars (rev 3, frozen 21 Oct 23:45, before any rev 3 run).** One tree for every run: day-1021 with this branch merged,
the 8 N pull cap and the rig's Part C files.

| Run | Confirms | Falsifies |
|---|---|---|
| Unit tests | `test_tip_detect.py` 16 pass, the planner suite passes | any failure |
| `t3_hover_tip` x1 | every drone armed; no UNANNOUNCED DETACH, WOULD-FIRE, drop or abort | any of them |
| `t3_hover_tip_glitch` x1 | the three 12 deg glitches: no UNANNOUNCED DETACH, WOULD-FIRE, drop or abort; a "held" line at each step | any detection, drop or abort |
| `t3_pair_release_tip` x2 | "UNANNOUNCED DETACH ... magnet" for drone 2 (/drone_1, plate 3) within 0.3 s of the RELEASE in both; the resize to n=2; no drop or abort; no detection of /drone_0 or /drone_2; the pair's roll p-p over its last 10 s and the whole-flight tilt peak no worse than the off arm's worse run | no magnet detection within 0.3 s, a drop or abort, a detection of another drone, or worse rocking or tilt than the off arm |
| `t3_pair_release_tipoff` x2 | the off arm: detection time by the stretch check (R2250/R2251: +0.82 s with chain_pair.yaml) | - |
| `c4_release9_tip` / `_tipoff` x1 each | informational: detection time, roll peak after the release, dropped at 45 deg or not | - |

**Rig use (Wesley's rule):** Part C flies `chain_pair_tip.yaml` only if these bars pass, critic round 3 leaves no
must-fix, and tomorrow's A/B flights replay clean (`WOULD-FIRES: 0`, every drone armed). Otherwise
`chain_pair_noxy.yaml`.

## Critic round 3 (22 Oct ~00:05): GO for tonight's twin comparison; Part C ON only after N1-N4

Round 2 items:
- **Closed:** M2, M3, M6, S1, S2, S5.
- **Partly closed:**
  - M1: a pure attitude glitch is harmless. A held mislabel that also shifts the pivot 1.5 cm along 0-6 for 0.3 s or more could still fire 0 or 6. The twin fault is a pure rotation, so `t3_hover_tip_glitch` passes by construction. No unstubbed plate-0 test yet.
  - M4: the covered-marker bench is not in the rig rule.
  - M5: not yet run on rig marker streams.
  - S7: these sections were stale; the 1.5 deg comment is now 2.1.
- **Closed once merged:** S4.
- **Open:** S3, S8. S6 is open but harmless.

New must-fix for Part C ON:
- **N1.** The scan that overruled round 2 cannot see a held marker swap. A departure held 11 samples becomes its own median, and swaps of 3-30 cm were not counted. Tonight: list every frame step over 3 cm in the 17 streams, with its hold time and |marker - body| against the hanging length. Any in-band hold of 0.1 s or more is a NO-GO until the radial test is shown to reject it.
- **N2.** The A/B replay gate is a weak yes/no. Print per drone the longest in-band run past both tests, the samples the band rejected, and the learnt offset. New bars: no such run of 50 ms or more, and each magnet's learnt offset repeats within 1 cm across flights.
- **N3.** On the rig the coils open 0.16-0.20 s after OFF (card 2026-10-08_drop_trigger_replay). Predict detection about 0.3 s or more after the OFF, not 0.1 s.
- **N4.** The sheet's ON branch needs the full procedure:
  - the replay between Part B and Part C;
  - R3 on chain_pair_tip.yaml;
  - T4 reads back tip_detect_m;
  - T2's three "armed" lines each within 1 cm of that magnet's A/B offset. A stray marker learnt in place of the magnet would otherwise fire when the real one comes back.

Should-fix:
- The slack band only for the rider, with the band printed.
- A sideways slide on plate 3 needs ~8.4 cm before the radial test passes.
- The replay arms on the measured ring height, the planner on the lift reference.
- The leaver's index differs between the twin (drone 1) and the rig (drone 0).

**Done tonight (before the comparison runs):**
- **Rider band:** only the rider (planned under 1 N, `TIP_RIDER_N`) gets the slack band up to the rod. A loaded carrier keeps +-3 cm and does not arm 6 cm or more short (new test).
- **"armed" line:** gives the offset in its plate's own frame (out, along, up), comparable across plates, and the band.
- **`tools/tip_replay.py`:** prints per drone the longest in-band run past both tests (ms, samples) and the samples off the band. Its exit status fails on a run of 50 ms or more as well as on a would-fire. `--rider` gives the T's rider its band.
- **Tests:** 17 in `test_tip_detect.py`, 235 in the planner suite.

**Bars added (22 Oct 00:15, before the comparison; the frozen bars above are unchanged):**
- **N1 (tonight, rig data):** count the in-band holds of 0.1 s or more among the frame steps over 3 cm in the 17 marker streams. Must be 0, or each one shown rejected by the radial test.
- **N2 (the morning's A/B replay):**
  - no in-band run past both tests of 50 ms or more on any drone of any A/B flight;
  - each magnet's learnt offset (out, along, up) repeats within 1 cm across its flights.
- **Load:** the planner's tick overruns in the tip arm of the comparison no worse than in the tipoff arm. Overruns are planner log rows more than 0.15 s apart in sim time, and solve_ms over 100.
- **S8:** detection time in sim time, from the RELEASE event to the planner's first row at the smaller n (10 Hz).
- **N3, the rig prediction:** Part C's detection about 0.3 s or more after the OFF. A marker more than 4 cm off its plate for more than 0.1 s in the bag with no detection falsifies.
- **N4:** the sheet's ON procedure as above.

**N1 result (22 Oct ~01:00, `tools/marker_swap_scan.py` over the 17 airborne marker streams, 10.2 min): 0 held
in-band swaps.** The stream is cut at every frame step over 3 cm, which gives 48 pieces.
- **Swaps:** every swap (jumps of 0.2-2.8 m) lands outside the body-length band.
- **In band and 0.1 s or more after a step (8 pieces):**
  - the marker's own track resuming after an out-of-band swap or a gap (6 pieces, 8-104 s long);
  - lt_7's 0.36 s piece: real motion during a stream stall. The 14 Oct crash flight's stream repeated samples, then
    caught up 5.7 cm while QUAD4 flew at ~1 m/s (printed sample by sample);
  - a3's piece: its "hanging length" is 2.76 m, so the marker was not on QUAD4.
- **The rest in band:** t5_4's 1-2 sample pieces (3-4 cm steps, within 0.6 cm of the bridging track) are a 3.5 m/s
  swing.
- **Caveat:** the two rules that set these apart were added after looking at the data, so they are post hoc:
  - a piece held 2 s or more is track;
  - a step right after a repeated sample is a stall catching up.

  The raw listing is in the tool's output.

**22 Oct 01:35, before any counted comparison run:**
- The twin's first runs exposed two harness faults in the marker emulation (fixed in b42227c and f5f9086):
  - the rod pose came relative to its own model, so R2287's markers never armed;
  - a crash in the tether callback.
- R2290 then armed every marker at 0.0 cm from its plate. But the frozen-ring rule (3 bit-for-bit repeats) held the
  check off 34 times in 45 s: the twin's ring stream repeats a sample or two. The rule is now time-based: the same
  ring pose for 50 ms or more (`TIP_RING_FROZEN_S`). The rig repeats 150 ms or more when it loses the ring.
- The comparison restarts from its first run on this tree. R2287-R2291 are void for the comparison.

## Results: the comparison (22 Oct 01:33-02:13, tree 367c98f, laptop)

| Run | Arm | Detection after RELEASE (stamped lines x RTF / planner's first n-1 row) | Outcome | Tilt peak, whole flight | Pair roll p-p, lean (last 10 s) |
|---|---|---|---|---|---|
| R2292 | T hover, on | none (3 armed at 0.0 cm; 6 frozen-ring holds) | landed | 7.4 | - |
| R2293 | T hover + three 12 deg ring glitches, on | none (held at each step) | landed | 14.3 (measured, glitches included) | - |
| R2294 / R2295 | T to pair, on | magnet: +0.27 / +0.25 s (rows +0.31 / +0.20) | resized to the pair, no drop | 8.8 / 9.6 | 3.6, 1.1 / 3.7, 1.1 |
| R2296 / R2297 | T to pair, off | none: the stretch check missed both | the T model flown to LAND, no drop | 7.4 / 11.1 | 3.0, 1.0 / 3.3, 1.1 |
| R2298 | square to T, on | magnet (rows +0.31) | dropped at 45.5 deg (peak 56.1) | 56.1 | - |
| R2299 | square to T, off | stretch (rows +0.21) | dropped at 46.1 deg (peak 81.3) | 81.3 | - |

Against the frozen bars:
- **Passed:** the unit tests; `t3_hover_tip`; `t3_hover_tip_glitch`.
- **`t3_pair_release_tip`, detection:** the magnet within 0.3 s, the resize, no drop or abort, no other drone's
  detection, the tilt peak under the off arm's 11.1: passed in both runs.
- **`t3_pair_release_tip`, rocking: MISSED in both.** The pair rocked 3.6 / 3.7 deg p-p against the off arm's worse
  run, 3.3. The off arm never detected the release and flew the T model on, so the arms differed in more than the
  check.
- **Load (round 3), passed:** over the flight, LIFTED to LANDED, no planner row came more than 0.15 s after the last.
  Solves over 100 ms before the release: 47 / 43 of 220 (on) against 50 / 46 (off). The whole-log count first read
  55 gaps in R2294: all of them after LANDED, while the runner read parameters back.
- **S8 (timing):** as in the table.

Verdict: the check does what it was built for. It catches the zero-share release that the stretch check missed
twice, and it does not fire on hover, glitches or load. It missed one frozen bar by 0.3-0.4 deg of rocking against
an arm that never resized. Under this card's rule Part C defaults to OFF; Wesley decides (the sheet recommends ON,
if the A/B replay is clean).
- **Square to T:** neither arm saves the ring. The release transient is the problem, not the detection (chain step
  2b).
- **Twin:** markers here are perfect, as round 3 noted. The rig's own false-fire evidence comes from the A/B replay.
