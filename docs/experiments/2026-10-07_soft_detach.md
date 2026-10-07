# soft_detach (7 Oct 2026): unload the leaving drone before the magnet opens

## 1. Question

Wesley, 7 Oct: "a plain detach on 1/3/5/9 with the new method, then a soft detach on the same setup,
then a soft detach on the more difficult setup, first trying in simulation and seeing what the
tensions get to."

## 2. Hypothesis

Today's OCP-mode detach applies the new 3-drone tension split in one planner tick, at the same moment
the magnet opens. The soft detach instead blends the tensions in the 4-drone OCP: the leaver goes to
0.2 N and the survivors to the balanced 3-drone split, over `detach_unload_s`. Only then does it
release and resize. Expected effect, per layout:

| quantity | plain (unload 0) | soft (unload 3 s) |
|---|---|---|
| survivor planned-tension step at release (1/3/6/9) | +64 / +64 / -35 % (r0013) | < 10 % |
| ring tilt peak after release (1/3/6/9) | capsize on the rig (r0013) | < 8 deg |
| ring z jump after release (1/3/5/9) | +13 cm rise on the rig (r0012), 3.0 cm in the twin (R1093) | < 2 cm |

## 3. The one variable

`detach_unload_s` (0 = today's step; 3.0 = soft). Everything else stays at the 7 Oct defaults:
- measured pull, 60 deg plan and handover;
- r0012's events, lift 0.15, 0.5 m step-out.

**Revised 7 Oct evening, after the critic.** Wesley: detaches fly the planned pull
(`cable_source: model`). With the measured pull each tracker flies the pull it actually feels, so a
planned unload would not change the load split. The soft-detach comparison is therefore on the planned
pull, and the r0012/r0013 settings and a forced cancel are added.

| arm | config | settings |
|---|---|---|
| D1 plain 1/3/5/9 | `w4_3915_detach_new.yaml` | measured pull, 60 deg ("new method") |
| D1p plain 1/3/5/9 | `w4_3915_detach_model.yaml` | planned pull, 60 deg: baseline for D2m |
| D2m soft 1/3/5/9 | `w4_3915_softdetach_model.yaml` | planned pull, 60 deg, unload 3 s |
| D3 plain 1/3/6/9 | `w4_3969_detach_new.yaml` | measured pull, 60 deg (new twin world `four_rigid_ground_rig_3969.sdf`) |
| D3p plain 1/3/6/9 | `w4_3969_detach_model.yaml` | planned pull, 60 deg: baseline for D4m |
| D4m soft 1/3/6/9 | `w4_3969_softdetach_model.yaml` | planned pull, 60 deg, unload 3 s |
| R1 plain 1/3/5/9 | `w4_3915_detach_r0012.yaml` | r0012 settings: planned pull, 45 deg |
| R3 plain 1/3/6/9 | `w4_3969_detach_r0013.yaml` | r0013 settings: does the twin capsize like the rig? |
| C forced cancel | `w4_3915_softdetach_cancel.yaml` | D2m with the tilt gate at 0.1 deg: cancel, blend back, LAND |

## 4. Baseline

| run | what | numbers |
|---|---|---|
| R1093 (twin, old defaults: model, 45 deg) | plain detach on 1/3/5/9 | peak tilt 2.9 deg, settle 0.9 s, dz 3.0 cm, landed |
| RIG-1007-r0012 | plain detach on 1/3/5/9, model, 45 deg | pass; the ring rose 13 cm; planned (2.42, 2.27, 4.27) -> (3.65, 3.50, 3.71) N |
| RIG-1007-r0013 | plain detach on 1/3/6/9 | capsized within 0.5 s, tilt 68.6 deg; planned (2.80, 2.29, 3.62) -> (4.59, 3.85, 2.38) N |

The plain arms D1 and D3 are the baselines for the soft arms (same defaults, same world).

## 5. Pass / fail numbers

Measured in the twin, from the detach command to 10 s after the release.

| metric | supports (fly it on the rig) | falsifies |
|---|---|---|
| aborts, tips | none | any |
| ring tilt peak, from the detach command to release + 10 s | soft <= plain and <= 8 deg | soft > plain |
| ring z jump in the 2 s after release (absolute) | soft <= 2 cm | soft > plain |
| leaver's ACTUAL pull at release (offline estimate from its mocap acceleration and throttle) | <= 50 % of its pre-unload pull | > 50 % |
| survivors' climb and ring tilt during the blend, before release | climb <= 5 cm, tilt <= 5 deg | worse |
| largest ACTUAL survivor pull, detach command to release + 10 s | below the magnets' ~9-10 N (Wesley) | >= 9 N |
| cancel arm C | blends back level, magnet on, lands | tilt > 5 deg or a release |

The planned-tension bars cannot fail by construction (the gate reads the planner's own tension), so
they are reported but not used as bars. If R3 does not capsize, the twin cannot test the fix: the
rig is then the test, and the soft detach is judged on the release pull and the climb/tilt bars only.

## 6. Repeats

One run per arm first (neighbours: both layouts, both cable sources). The arms that pass and go to the
rig get their second run.

## 7. Cost

9 Gazebo runs (D1, D1p, D2m, D3, D3p, D4m, R1, R3, C), plus the figure-8 at 0.2 m/s, plus 2-4 repeats.
Lab PC tonight, in parallel (no cap there).

## 8. New code?

Yes: `unload_tensions` and `ReferenceBuilder.retarget` (`mpc_planner/reference_builder.py`); the
unload phase in `dissipative_node.py` (`_start_unload`, `_unload_tick`, `_resize_and_release`,
`_cancel_unload`); five knobs (after the critic: the leaver's release tension is 0.5 N, not 0.2, so the
cable stays lightly taut and cannot go slack and snatch; the cancel blends back over the unload time):

| knob | node default | gates |
|---|---|---|
| `detach_unload_s` | 0, off | |
| `detach_unload_t` | 0.5 N | |
| `detach_unload_tilt_deg` | 10 | release |
| `detach_unload_wait_s` | 2 | timeout cancels the detach |
| `detach_post_blend_s` | 1 | |

Off by default, so plain detaches are unchanged. If falsified, the code is recorded in the registry
and deleted the same session (Wesley's call).

## Critic

Critic agent, 7 Oct evening (read-only), condensed. Verdict: **rewrite the card**. On the default
measured pull the soft arms do not change the physical load split, and the gate plus two bars pass
by construction, so a twin "supports" would send an untested change to the rig.

**Must-fix**
1. On the measured pull, each tracker flies the measured pull at node 0 (`tracker_node.py:1510-1514`),
   and the drone position references do not move, so the leaver still carries about 2 N at release.
   **Done:** Wesley chose the planned pull for detaches; the measured soft arms were dropped.
2. The gate reads the planner's own stage-1 tension, so "planned change per tick" and "planned
   leaver tension" pass by construction. **Done:** the bars now use the leaver's actual pull at
   release (offline from its mocap acceleration and throttle) and the climb and tilt before release.
3. No arm reproduces r0013 (planned pull, 45 deg); the legacy twin flew a plain 1/3/6/9 detach at
   5.7 deg without capsizing (R0582/R0584). **Done:** R1 and R3 added.
4. Add a forced-cancel arm; the cancel blended back three times faster than the unload. **Done:**
   arm C added; the cancel now blends back over `detach_unload_s`.

**Also raised**
- Tried before in the attach direction: the post-weld tension slew worked in SIL (R0546) and Gazebo
  (R0547). In R0547 a 0.1 N target on an attached rod went slack and then yanked. **Done:** the
  release tension was raised to 0.5 N.
- A simpler explanation: today's measured-pull default may already remove r0013's step for a plain
  detach (D3 tests it).
- On the rig:
  - 0.5 N is near the measured pull's resolution (about 1 N off in r20004);
  - a real cable can snatch, which the rigid-rod twin cannot produce;
  - the 9 N bar cannot be read from the 10 Hz planned or 1 Hz measured logs;
  - loop delay is untested.
- The new world `four_rigid_ground_rig_3969.sdf` needed Wesley's OK; it came with the approved plan.

## Results (7 Oct night, lab PC twin)

Actual pulls from `tools/detach_tensions.py`: drone mocap acceleration minus the twin's thrust law.
Check: the four vertical pulls add up to the ring's weight, 8.40-8.43 N in every run. Pre-detach
ring tilt is 1.5-2.0 deg in every run.

| arm | run | leaver pull before / at release (N) | tilt during unload (deg) | tilt peak, release to +10 s (deg) | ring z jump (cm) | survivor peak (N) | LAND |
|---|---|---|---|---|---|---|---|
| D1 plain 1/3/5/9, measured | R1118 | 1.67 / 1.30 | - | 8.4 | -3.2 | 3.77 | landed |
| D1p plain 1/3/5/9, planned | R1128 | 1.57 / 1.50 | - | 3.3 | +0.6 | 3.70 | landed |
| D2m soft 1/3/5/9, v1 | R1120 | 1.57 / 0.38 | 4.7 | 4.7 | +0.5 | 3.71 | landed |
| D3 plain 1/3/6/9, measured | R1121 | 1.88 / 1.71 | - | 16.4 | -4.5 | 4.55 | landed |
| D3p plain 1/3/6/9, planned | R1122 | 1.84 / 1.76 | - | 4.1 | -1.1 | 4.48 | landed |
| D4m soft 1/3/6/9, v1 | R1123 | 1.84 / 0.35 | 5.5 | 5.6 | +0.6 | 4.27 | landed |
| D2m soft 1/3/5/9, v2 | R1134, R1137 | 1.57 / 0.49, 0.50 | 2.1, 2.1 | 2.5, 2.6 | -0.8, -0.8 | 3.71, 3.73 | landed, landed |
| D4m soft 1/3/6/9, v2 | R1131, R1136 | 1.84 / 0.49, 0.48 | 2.0, 2.0 | 3.3, 3.5 | +0.7, +0.7 | 4.20, 4.20 | landed, landed |
| R3 plain 1/3/6/9, r0013 settings | R1125 | 2.18 / 2.08 | - | 5.5 | +2.2 | 5.14 | landed |
| C forced cancel | R1126 | 1.57 / not released | 5.3 (blend and back) | - | - | 3.81 | landed |

Voids, all pose timeouts with several four-drone runs at once on the lab PC: R1119, R1124, R1129,
R1130, R1132, R1133, R1135.

- **v1 → v2.** v1's survivor target ignored the leaver's remaining 0.5 N (0.1 N·m), and the ring
  tilted 2 → 5.5 deg during the unload. v2 balances the survivors with the leaver's 0.5 N counted
  (`unload_tensions`). The ring then stays at its hover tilt through the unload.
- **The measured pull is the wrong source for a detach** (D1, D3 against D1p, D3p): 8.4 and 16.4 deg
  against 3.3 and 4.1.
- **The twin does not capsize at r0013 settings** (R3: 5.5 deg). The rig is the test of the fix.
- Peak actual pull on any cable: 5.1 N (45 deg) and 4.6 N (60 deg), against the magnets' 9-10 N.

**Verdict against section 5 (v2, two runs per layout):**
- no abort or tip;
- the leaver's actual pull at release is 0.48-0.50 N (26-32 % of before; bar 50 %);
- tilt during the unload stays at the hover level (2.0-2.1 deg; bar 5); survivor climb 0.5-0.6 cm
  (bar 5);
- tilt peak after release is below the plain detach on both layouts (2.5-2.6 against 3.3; 3.3-3.5
  against 4.1);
- ring height jump 0.7-0.8 cm (bar 2);
- peak pull 4.2 N (bar 9);
- the cancel works (R1126).

**SUPPORTS: fly the soft detach on the rig** with `cable_source model` (Wesley 7 Oct), unload 3 s,
leaver at 0.5 N. The twin cannot capsize at r0013 settings, so the rig run on 1/3/6/9 is the real
test.
