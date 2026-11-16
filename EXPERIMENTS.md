# The paper's experiments

Each experiment, the command that ran it and the code it ran on. "week8 X + changes" means the commit X of the
development repository with uncommitted changes; for simulation runs those changes are saved in each run's folder
(`git_diff.patch`, its sha256 in the run's manifest). Real-world flights from 16 Nov run on a tagged commit.

## Real world
| Experiment | Command | Code | Flights |
|---|---|---|---|
| Detach and re-attach, hover | `experiments/rig/rig_nrtf.sh` | tag `rig-2026-11-16` | RIG-1116-nrtf_4, _5, _7 (nrtf_2, _3 voided: caught on the plate's edge) |
| Detach and re-attach, figure-eight | `experiments/rig/rig_nr8.sh` | tag `rig-2026-11-16` | RIG-1116-nr8, nr8_3, nr8_4 |
| Detach and re-attach, circle | `experiments/rig/rig_nro.sh` | tag `rig-2026-11-16` | RIG-1116-nro_2, _3, _5 |
| The full mission, circle | `experiments/rig/rig_fm9c.sh` | tag `rig-2026-11-16` | RIG-1116-fm9c_4 (fm9c_2 voided: the re-attachment let go) |
| The full mission, hover | `experiments/rig/rig_fm9.sh` | week8 bf52a8a + changes (not recorded) | RIG-1112-fm9_5 |
| Drop-off, hover / circle / figure-eight (Te-Jen) | `CPP=on SWING=3 SPLINE=2 RET=cpp EXIT=moving experiments/rig/rig_fm9h.sh`; circle and figure-eight add `CPPMOVING=1` (`rig_fm9hc.sh`, `rig_fm9h8.sh`) | to be flown 17-18 Nov | - |
| 4 -> 2 -> 4, hover | `experiments/rig/rig_424.sh` | to be flown 19 Nov | - |

## Simulation
| Experiment | Configs | Code | Runs |
|---|---|---|---|
| Attachment-point error, along the rim or radially in | `configs/experiments/x11_u_meas_*.yaml` (the fold-in at the measured point), `x11_u_slot_*.yaml` (the slot assumed) | week8 bf52a8a + changes (diff 11643a387a4c) | R4034-R4093 |
| Attachment-point error, both together | `configs/experiments/q034_u2d_*.yaml` | week8 4d613bac (the configs added 16 Nov) | R4541-R4580 |
| Eight carriers: four releases and four returns | `configs/experiments_oldlinks/x3_s8_chain4back_it5_tip.yaml` | week8 bf52a8a + changes (diff be9ea5d8a1c2) | R3972, R3973 |
| The endurance relay | `configs/experiments_oldlinks/x4_relay_30min_v3_norod.yaml`, `x4_relay_30min_m115_v3_norod.yaml`, `x4_fixed_B_v3_norod.yaml` | week8 bf52a8a + changes (diff f09c9430426b) | R3910-R3914 |

The scalability runs fly the earlier quadrotor link masses (worlds `*_oldlinks.sdf`).
