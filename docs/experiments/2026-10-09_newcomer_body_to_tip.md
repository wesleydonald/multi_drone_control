# Newcomer approach and weld capture use body-to-tip (rod + pivot drop)

1. **Question.** Does the newcomer's approach put its magnet tip at the plate target, and does the weld capture the
   plate, when the rod hangs from a pivot below the body (rig and twin: 4 cm)?
2. **Hypothesis.** attach_cable_len served both as the body-to-tip length (approach, weld capture) and the rod
   pivot-to-tip (OCP rod, detection). On the rig and twin the pivot sits 0.04 m below the body, so the approach
   parked the tip 4 cm low (on the plate in a dry approach) and the weld capture put the tip 4 cm high. Using
   attach_cable_len - pivot_offset_z for the two body-referenced uses puts the tip at the target; the OCP keeps
   the rod length.
3. **The one variable.** dissipative_node.py: _attach_body_to_tip = attach_cable_len - pivot_offset[2], used by
   ApproachProfile and _capture_weld_rho only. Legacy (pivot 0): unchanged.
4. **Baseline.** No twin attach run exists yet (the twin attach world is new, 8 Oct); QUAD4's rod measured
   0.541-0.546 m pivot-to-plate in the 8 Oct rig logs (body-to-tip ~0.58).
5. **Pass / fail.** Twin T-R8a (dry approach, weld 0) on the free-start world: supports if the tip holds within
   0.03 m of the target for 5 s with no ring contact; falsified if the tip ends more than 0.03 m off in z. A
   neighbour at attach_cable_len +/- 0.04 m shows the sensitivity.
6. **Repeats.** 2 (T-R8a), neighbours 1 each.
7. **Cost.** Rides on the T-R8a gate runs (+2 neighbours).
8. **New code?** Yes, two lines in the attach reference (critic below). Removal: revert the commit.

## Critic
(pending)
