# Tejen → Wesley: decisions on the 29 October rig follow-ups

## Scope and evidence

This is a read-only review and proposed next steps, not an applied patch or flight
authorization. No repository code, configuration or shared records were changed.

Reviewed: `rig-evidence-20261029.zip`, `cable-geometry-20261029.md`, and the override
excerpt in `prompt-for-codex-20261029.md`. The source comparisons use Tejen HEAD
`73e7b7fed86a584c7e6642dfc473c0fe9fe1a8a6` and the historical `11a0683f` oct10
baseline. Your renamed, locally modified fork is distinct from both. The new
modified MPC main.py/OCP files were not supplied in full, so the timing override
can be assessed in principle, not certified end-to-end from that excerpt.

The current source names below map as follows:
`controller_mpc_payload` → `tejen_mpc`, `drone_communication` → `tejen_mission`,
`dynamic_planner_ros` → `tejen_dynamic_planner`.

## 1. Timing override: yes, this is the intended correction

**Decision:** `mpc_command_execution:=partial_step` with oct10 weights/bounds is
the right isolated change to evaluate. Do not change to oct14 merely to obtain
the timing fix. Keep the carriers' controller unchanged for this comparison.

Historical oct10 has a 2 s / 20-stage OCP: nominal first stage100ms. Its command
state remains `x[13:17] = [roll, pitch, throttle, yaw]`, with command derivatives
as the four OCP inputs. The full OCP is23 states, including two XY integrals;
the original physical payload model is21 states. Cost dimensions26/22 and the
15-parameter OCP do not change for this runtime correction.

No oct14-specific assumption is needed. Under piecewise-constant first-stage
command derivatives, interpolating command states is exact for that interval:

`command_send = command_start + clip(elapsed / stage_duration, 0, 1) * (command_end - command_start)`.

Read stage duration from the constructed solver, not a copied constant. Oct10
roll/pitch/yaw derivative bounds are±1/s and throttle±0.5/s; retain them. At a
33.33ms publication interval, a saturated throttle derivative changes throttle
by0.01667, not the full100ms endpoint change0.05. Existing output quantization
adds its small rounding error; distinguish model throttle from radio channels.

Check that:

- The override flag is not overwritten later in construction or at runtime.
- The initial command state equals the previously applied rounded command.
- History and UKF input handling receive the command actually sent, not the
  discarded endpoint. Preserve existing arm/idle/takeoff output semantics.
- The first active publication uses the existing nominal interval; idle/re-arm
  resets timing. Paused simulation time must not advance the command.
- Simulation uses its trajectory/node clock consistently. No new clock-policy
  change is necessary for this port.
- Saved metadata says oct10 AND `first_stage_partial_step`; weights, derivative
  bounds and OCP layout are unchanged. Keep the existing cache identity valid.

**Tests:** the upstream arithmetic helper passed seven isolated elapsed-time
checks here, including20/33.33/50ms and clipping. This is not a test of your new
fork. In your twin, test actual send/history behavior, idle→TAKEOFF and re-arm,
variable intervals, and identical oct10 OCP weights/bounds. Then compare one
matched rehearsal using the same geometry/profile. Report swing/tracking as
outcomes; the timing bug is verified but its causal share of swing is not.

## 2. Descent reapproach: position debounce is a reasonable next small patch

**Proposed, after the timing comparison:** debounce the existing8cm POSITION
restart for0.30s of continuous fresh violation. Reset on recovery inside the
existing bound, phase/attempt reset, or stale/invalid evidence. Missing samples
must not earn dwell; retain existing input-loss handling. Use a dedicated timer
so MATCH/contact dwells are not accidentally shared.

For this isolated patch, keep the existing body-relative0.25m/s gross-speed
restart unchanged. Do not silently replace it with tip speed at the same number:
that changes its meaning and could create more restarts during ordinary swing.

For actual contact/proof, the relevant speed IS calibrated magnet-to-plate speed,
not body speed. Keep that evidence separate. If a later tip-speed guard is added,
derive it from fresh relative positions/timestamps and review its filtering and
threshold separately; use the measured moving plate, not an artificially zeroed
target velocity. Pose differentiation is not a new global estimator project.

The debounce alone leaves the existing bounded descent active. Before accepting
it, check extra commanded travel `descent_speed * 0.30s` against the actual
terminal geometry. Do not conceal a new Z-pause/reference-progress mechanism or
deeper endpoint inside a transition-guard patch. Nor can the recorded, already
aborted trajectories prove what the next0.3s would have done without the abort.

Passing MATCH, surviving a debounce, or finishing the descent reference never
proves attachment. Preserve fresh near-contact evidence, explicit confirmation/
co-motion proof and existing release/recovery ownership.

**Tests:** a sub0.3s position excursion does not restart; continuous fresh loss
does; recovery/staleness cannot accumulate hidden time; speed behavior and manual
LAND remain unchanged; terminal/profile completion does not fabricate contact.

## 3. Pickup capture, MATCH, LOW and bottom hold

**Capture:**6cm /0.15m/s /0.5s is a defensible provisional ABOVE-OBJECT trial if
the timing-corrected run still cannot earn capture. It is not a validated safe
contact window and should not be promoted before the timing comparison. Do not
change capture and the execution timing in the same diagnostic flight.

I reproduced pickup error4.07cm median, mean(-1.39,+0.31)cm and5.09cm RMS about
the mean. However,17% XY occupancy alone is not the capture predicate: replay
the conjunction of fresh calibrated face XY, filtered relative speed, clearance
and dwell. Near the centre of a swing, speed is often largest, so XY and speed
requirements must be assessed together.

**LOW/HIGH:** your10/12cm rig bands are not our commissioned2.5/5cm profile.
They may be used as explicitly experimental progression bands, but I cannot
certify10cm near-contact error from a100×85mm slot: usable capture width also
depends on magnet footprint and registration. Increasing the capture radius to
6cm does not validate those other bands. No new wider LOW band is recommended.
Keep lateral trim frozen close to the surface; do not add sideways searching.
The present1s LOW dwell may itself be difficult under swing, so replay it after
the timing fix rather than adding another wait or silently deleting it.

**MATCH:**8cm /0.5s is reasonable as a provisional staging trigger, not attachment
confirmation. The previous change also zeroed target velocity, so the12.3→1.7s
comparison does not isolate the position/dwell thresholds. Keep raw ring motion
for diagnostics/contact checks. A nominal stationary-ring command and a measured
ring velocity used for evidence are different quantities.

**Bottom hold:** ordinary oscillation or an elapsed time alone should not switch
the magnet off. Preserve the bounded terminal hold and explicit loss/depth guards.
A quiet-miss timer should count only genuinely slow, fresh magnet-relative
evidence, not low body speed while the magnet is moving. Keep the fork's contact/
co-motion proof independent of these wider approach bands. No new proof scheme
or changed proof thresholds are proposed here.

**Tests:** wider staging capture cannot certify contact; stale evidence cannot
earn capture/proof dwell; LOW cannot add trim or extra overtravel; swinging-tip
evidence cannot be mistaken for a stationary quiet miss; manual LAND works.

## 4. Swing: the new geometry strongly supports real oscillation

I independently reproduced the main chord/angle table from body+marker poses
using the stated rotated anchor offset. For k1_4 settling: chord median1.0085m,
5–95%1.0070–1.0101m; mean angles(-0.64,+0.36)deg and RMS(1.75,1.33)deg. MPC
means are(-0.64,+0.35)deg. The short sq5_2 windows also reproduce closely.

This substantially strengthens the case for actual oscillation and against
explaining all target error as one large constant bend. It does NOT prove the
cable was perfectly straight or rule out an anchor kink. Agreement with the MPC
validates signal plumbing, not independent physical calibration: both calculations
share the same poses and assumed anchor transform.

In particular, chord shortening is quadratic for small bends. A symmetric4cm
midpoint bow on a1m two-segment cable shortens the endpoint chord by only about
3.2mm. Millimetre agreement with a bench chord therefore cannot exclude every
centimetre-scale bow. The observed mean angle is not a strict standalone upper
bound on an unobserved bend if other forces/offsets can compensate it. Likewise,
frequency estimates from1–2s windows are weak evidence for a roughly2s-period
pendulum; use the long settling window for a meaningful spectrum.

**Swing cost is active:** all supplied MPC rows report FREE_SWING. In historical
oct10, running AND terminal weights for `[phi, theta, phi_dot, theta_dot]` are
`[20,20,2,2]`, with zero-swing references. VERTICAL_TAKEOFF, ATTACH_PROOF and
ATTACHED_HOLD are the modes that suppress that block upstream; FREE_SWING does
not. The baseline equations are
`phi_ddot=-(g/L)*phi-a_x/L`, `theta_ddot=-(g/L)*theta-a_y/L`.

Improved damping after correct command timing is plausible, not guaranteed. The
model does not include cable stiffness or downwash. Do not retune swing weights,
add a bend estimator, or change L in this first comparison. L0.997 versus a marker
chord near1.009 differs by~1.2%; an effective pendulum length is not automatically
identical to anchor-to-marker chord length.

**Judge the next run by:** phase-aligned angular mean, de-meaned RMS/p95 and rates;
long-window spectrum; body tracking versus magnet-to-plate/object tracking;
measured relative contact speed; mean/changing chord and vertical drop; actual
command intervals/slew, solver status, reference freshness and kT history. Keep
takeoff/slack/floor-contact samples out of free-swing metrics. Retain failures.

## 5. C++ transit

### Handoff

Keep the provisional0.12m/s settle-speed setting; recommend retaining0.15m drift
rather than automatically doubling it to0.30m. If0.15m fails, first identify
whether the hold was latched from the measured state or an unreachable nominal
point. The speed and drift changes are separate decisions. None of this changes
the later P/V/A continuity or authority checks.

### Carrier tethers / low endpoint

First inspect the exact failed component pair and collision witness. The supplied
scalar separation does not identify whether the bottleneck is ego magnet/cable/
body against a carrier body, tether segment or another component.

My first bounded option is a higher feasible transit endpoint under the measured
geometry and existing swing allowance, not reducing the allowance to1degree.
Confirm the subsequent local approach is physically clear too: raising the C++
endpoint does not collision-check the later Python descent. If box over-coverage
is the actual blocker, a conservative segment/capsule tether representation is
a separate geometry change through search, refinement and final checking, not
an emergency margin override. No such redesign is recommended for this timing
comparison.

### Start-skew: do not call the40ms prefix measured history

The workaround keeps refresh cadence, but its past first point is sampled from
the PRESENT measured ring. Even if a prefix is nearly constant, it is not a
reconstruction of where a moving/bouncing ring was40ms earlier. It therefore
cannot be generally endorsed as an actual past trajectory. Treat any retained
hover-only use as a documented approximation with an error budget, not a fact.

Publishing states before commitments is a better producer ordering, but separate
ROS topics can still be processed in another order. Reordering alone cannot be
claimed to remove the race. A proper bounded fix must ensure validation compares
a fresh state time covered by the advertised curve, while never accepting an
uncovered interval. Test both callback orders. Do not simply remove the coverage
check, disable peer updates or add unrestricted tolerance. This needs its own
small producer/receiver contract review, not a controller timing change.

### Search floor

Your10cm nominal climb-to-floor reserve is a useful launch consistency check,
not a guarantee that the measured hold/end is feasible. Keep the floor compatible
with the actual geometry/workspace; do not lower it below genuine collision
limits. The backend correctly does not invent an unplanned climb while it lacks
authority. Fix the launch/mission height relationship first. If measured recovery
is later needed, the existing mission-owned climb should own it explicitly—not
an aborting timeout or a surprise backend manoeuvre.

### Installation correction to earlier advice

The supplied fork already includes the completion-wakeup changes. sq2_5 accepted
and installed27/27 candidates; completion-to-collection median0.20ms, p951.82ms.
There is no world-version installation-starvation evidence in that run. Do not
port the same fix again or remove the atomic installation checks.

## 6. Mass: distinguish a fork gain-schedule parameter from the dynamics

`object_vehicle_mass_kg` is a fork addition, not a mass parameter in our normal
acceleration-per-throttle payload OCP. The supplied launch reconstruction sets
`object_mass_kg=0.0`, and your change notes describe zero as disabling the added
object-attached gain schedule. On that documented path, the0.55 denominator is
inactive for these trials; it is not evidence the whole controller assumes a
0.55kg vehicle in its equations. Our separate taut-load model has explicit mass
parameters, but is not the normal model generated by this payload OCP.

Record the measured0.583kg, but do not expect changing this inactive setting to
fix the current swing or rescale kT blindly. Before enabling a mass schedule,
set the physically correct value and check interaction with existing kT feedback.
The exact fork function using this parameter was not supplied; send that small
function/call sites if you want a fully verified fork-specific trace. I will not
present the documented zero/off behavior as a completed source-level fork audit.

## Patch and validation boundary

No new unified code diff is included: this chat is explicitly read-only, the
profile override exists only in your fork, and upstream already executes partial
steps unconditionally. A blind upstream diff could duplicate or conflict with
your implementation. The supplied override matches the intended timing design;
items2–6 above are bounded recommendations, not bundled implementation approval.

First validate the timing-only fork patch with oct10 unchanged. Reassess measured
swing before committing to wider capture/debounced restart. Keep each subsequent
change independently logged and reversible. No new simulator or physical flight
was run during this review.
