# Small return package

Please use existing saved artifacts; do not wait for new instrumentation/flights.
Mark absent fields as unavailable rather than reconstructing them as measured.
Keep full identified run bundles when practical; no generated solvers/build trees
or unrelated bags are needed. Existing relevant bags are useful if CSVs omit poses.

## Priority runs

1. **sq5_2:** all six descent reapproaches, plus preceding MATCH and terminal hold.
2. **k1_4:** the 26 s pickup capture wait, including pre-wait settling.
3. **sq2_5:** slower MATCH window and any C++ handoff/install issue.

Provide exact directory names and the operator observations associated with each.

## Configuration and provenance

- Flown Git SHA, branch and dirty source/config diff, including renamed packages.
- Exact launch command/overlays; saved resolved planner, MPC, bridge and backend
  parameters if available. Do not supply only the intended launch defaults.
- Actual mpc_profile and command-execution metadata/startup line, cable length,
  mass/model selection, thrust calibration and swing-mode selection.
- All capture/HIGH/LOW/MATCH/descent-reapproach/contact/proof thresholds and dwells,
  alignment trim settings and whether a measured-tip mode was active.
- Search floor, approach/terminal height, carrier geometry, swing allowance,
  tracking allowances, tracking enforcement flags, any backdate/skew override.
- Body/marker/plate IDs, anchor and contact transforms, measured ring geometry.

## Measurements and events

- Planner CSV/status, phase transitions and retry reasons; measured versus
  reference body and magnet position/velocity; raw ring/plate motion if recorded.
- Tip/target clearance, XY error and relative-speed channels actually used by
  gates; identify filtered, zeroed, commanded and measured quantities separately.
- Raw body, magnet, pickup and ring pose streams with timestamps if existing.
- MPC state/reference/commands, swing angles/rates, solver status, timing and kT.
- Backend/replans CSVs for any C++ window: acceptance, installation/rejection,
  coverage/start-skew/tracking blocker, world versions and fallback events.
- Times of TAKEOFF, START/release, LAND, DISARM and visual attachment observations.

Keep source/receipt, ROS and monotonic clocks identified. Do not align streams
by assuming their numeric time origins are equal.

## Specific questions

- Which controller profile actually flew each run: oct10 or oct14?
- Is the 8 cm descent test using measured magnet-to-plate error?
- Is the stated 0.02–0.14 m/s speed body speed or magnet-to-plate speed?
- Is the roughly 4 cm tip displacement a persistent mean offset, oscillation
  about a mean, or both? Which coordinate frame defines it?
- Was the 1 m cable visibly bent/slack, and was its geometry measured in that
  same configuration? Do not assume its vertical drop equals cable length.
- Was raw ring velocity retained when attach_twist was set to zero?
- Were the reported legs actually C++-owned or Python-owned? Keep these distinct.
