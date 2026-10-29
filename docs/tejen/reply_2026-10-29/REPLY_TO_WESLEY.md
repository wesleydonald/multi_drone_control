# Reply to Wesley

Thanks for the detailed report. Transit/MATCH progressing is useful evidence.
The final-step failures look like several separate issues, not one reason to
widen every threshold. These are provisional interpretations of your report;
I have not yet inspected sq2_5, sq5_2 or k1_4.

## 1. Confirm the actual controller profile first

Your change notes say the elapsed-command-interval correction is applied only
under `mpc_profile:=oct14`; the default remains `oct10`. Which profile actually
flew each run? Please send saved controller parameters/startup output.

If oct10 retains the full 100 ms command endpoint at each roughly 33 ms tick,
that known timing discrepancy is still present. Do not assume it explains all
the reported swing. Confirm the actual send path before another gain change.
The profile also changes weights/bounds, so switching profiles is not an isolated
timing-only experiment. Do not bulk-copy a newer controller to resolve this.

## 2. Descent reapproach: a narrow debounce is worth evaluating

Our current generic attachment descent still returns to approach on a single
position/speed threshold violation. A fresh-evidence sustained-loss dwell around
0.3 s is a reasonable proposal for transient excursions, rather than repeated
immediate retreats. It is not implemented or approved by this message.

First identify whether the failing metric is measured magnet-to-plate position
and velocity, or a body/nominal-tip tracking metric. The reported QUAD4 speed
does not establish a low magnet-to-plate contact speed. Inspect actual descent
speed/clearance before deciding what happens during any proposed debounce.
Preserve distinct final contact/proof evidence; do not let a completed reference
or a widened approach band declare attachment.

## 3. Pickup capture and MATCH

The proposed 6 cm / 0.15 m/s values can be evaluated as an above-object capture
trial, not certified final-contact limits. Your rig already has 10/12 cm LOW/HIGH
bands; those are substantially wider than our source defaults. Please send the
resolved whole alignment profile so capture, LOW and proof are reviewed together.
Slot dimensions alone do not establish contact capture: magnet footprint and
registration offsets matter too. Keep proximity plus low relative speed for
actual contact/proof; a body-speed reading is not a substitute.

Zero attachment twist can suppress noisy derivatives in an actually stationary
target test, but ignores real bounce and changes what MATCH measures. Please
retain raw measured ring/plate motion alongside any zero/filtered command path.
Do not extend that approximation to a genuinely translating ring by default.

## 4. A bent cable is not necessarily pendulum swing

Tejen flags an additional confound: a stiff bend can give the magnet a persistent
lateral offset and shorten its vertical drop even without oscillation. An angle
derived from the body/anchor-to-marker line then includes that offset; it does
not identify the cable's shape or prove periodic swing. The MPC's simple
pendulum model assumes a straight cable and does not model a stiff bent shape.

For scale, 40 mm lateral offset over 1 m vertical separation is about 2.3 degrees;
95 mm is about 5.4 degrees. These are geometric equivalents, not measured
oscillation amplitudes. Conversely, 1 degree over 1 m permits only about 17 mm
of lateral displacement. Separate the mean offset from oscillations about it,
using a consistent anchor transform and body/ring frame. Check whether the
reported 4 cm median is a centre bias, oscillation, or both.

Do not simply subtract the mean from collision geometry or contact error: the
magnet really occupies that displaced position. Nor should a biased swing
estimate automatically prompt larger swing penalties or another estimator.
If practical later, compare props-off resting geometry in the actual cable
configuration against freely hanging geometry. This is a diagnostic, not a new
flight gate, online cable-length estimator or required extra flight campaign.

## 5. C++ blockers need separate answers

- Search floor: the configured floor must be compatible with actual hold height
  and terminal geometry. A timeout cannot make an infeasible endpoint feasible.
- Handoff: 0.12 m/s is an understandable rig trial for 0.08–0.09 m/s hover motion.
  Increasing allowed drift to 0.30 m is a separate concession, not the same fix.
- Carrier geometry: reducing the assumed swing to about 1 degree simply to admit
  an endpoint is not supported by reported 40–95 mm tip excursions. Inspect the
  exact failed component relation and target height before changing this bound.
- Peer start-skew/tracking latch: this is distinct from the ego tracking-only
  brake. Our current IRL profile disables both tracking-enforcement flags, but
  valid time coverage/freshness and collision inflation remain necessary.
- Your latest documented upstream sync is fd495db0. A targeted later completion-
  collection fix is identified in UPSTREAM_NOTES.md. It is not a blanket cure or
  permission to remove atomic version checks. The newer navigation-specific
  policies do not automatically belong in your combined M1 rig.
- Backdating commitments must describe the actual trajectory at those times;
  changing timestamps alone must not fabricate measured trajectory history.

## 6. Mass, thrust and swing damping

0.583 versus 0.55 kg is roughly a 6% difference worth recording. Whether changing
it affects this controller depends on the active model/estimator path; the main
translational model uses acceleration-per-throttle gain. Do not infer a simple
6% thrust-ratio correction without tracing that path.

Our swing states/cost can influence swing when enabled, but do not currently
time contact to a favourable swing phase. Their effectiveness on your bent,
1 m tether needs data rather than a promise of damping.

Tejen's simultaneous 35.2→38 standalone obstacle-controller trial uses a different
affine thrust convention. Those values are NOT settings for tejen_mpc.

## Requested next step

Send sq5_2 and k1_4 first, plus sq2_5 if convenient, with resolved parameters,
actual code revision/dirty changes and controller profile. See EVIDENCE_REQUEST.md.
Then we can identify a small gate fix without broad tuning or a wholesale merge.
