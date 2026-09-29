# Design note — fleet safety layer (findings F3, F4)

**Status: IMPLEMENTED 2026-08-04.**
`utility_objects/safety.py` (pure class, 15 tests) · wired into `controller_mpc.py` ·
emergency path in `main.py` · pre-arm interlock hook in `callback_manager_multi.py`.

This note was drafted before the code was read, revised twice as the code turned out
to be different from assumed, and then rewritten to describe what was actually built.
The corrections are kept in §6 because they are the useful part.

---

## 1. Policy

**Any fault disarms the whole fleet, immediately. There is no descent manoeuvre.**

Wesley's decision, and a sound trade here: everything operates at ~1 m, so a drop costs
props and maybe the payload, not a catastrophe — at the reference paper's 15 m it would
be unacceptable. A safety path with no manoeuvre in it cannot have a bug in the
manoeuvre, and the alternative (four coupled drones descending under a swinging payload
with one member already dead) is not obviously safer.

**There is deliberately NO geofence.** One was implemented and then removed: the
operator is at the cage with a kill switch throughout testing, and a position check
catches nothing a ready human does not. **This is a decision, not an oversight** — said
explicitly here and in the code so nobody re-adds one by reflex.

What is kept is what a human **cannot** react to in time, or cannot see at all:

| Check | Fault when | Why a human cannot cover it |
|---|---|---|
| **Reference staleness** | no planner message for > 1.0 s | Nothing on screen says the planner died; the trackers keep flying the last reference and drift |
| **Payload / drone tilt** | > 60° | The attach runaway develops in about a second |
| **Speed** | > 3 m/s | Same |
| Battery | **never — warn only** | Voltage sags under thrust transients; a threshold would fire mid-manoeuvre, and a layer that cries wolf gets switched off |

Rejected and recorded so it is not re-litigated: *automatic failure→dissipative
redistribution*. Attractive — it is what the network is for, and it is Quan et al.'s
motivating scenario — but it would make the research code part of the safety path. It
stays a **sim-only experiment (E10-F)**.

## 2. Architecture

No new node, no new topic aggregation. The propagation path already existed and is
proven in flight; the envelope simply feeds into it.

```
tracker (50 Hz)                                 fleet manager
  envelope check each cycle
        |  FAULT -> cb.disarm()
        |             |
        |             +--> /drone_i/arming_state_feedback --> _arming_feedback_callback
        |                                                            |
        |                                          _disarm_fleet(emergency=True)
        |                                                            |
        |   <---------------- /fleet/abort (String, reason) ---------+
        |   <---------------- /drone_i/ELRSCommand armed=false ------+
        |                                                            |
        v                                          + SetArming services (acknowledged)
   immediate disarm
```

**The emergency fast path is synchronous.** `/fleet/abort` and the direct
`ELRSCommand(armed=False)` publishes happen inline in `_disarm_fleet`, before any
thread is spawned or any service is touched. Measured at ~1 ms from command to
broadcast. The services still run afterwards as the acknowledged disarm.

Both cases are covered: a live tracker acts on `/fleet/abort`; a wedged tracker is not
publishing anything, so the manager's direct publish reaches the radio unopposed.

**Loop safety:** a tracker aborting sets `_aborted`, and the manager sets
`flying = False`, so the feedback→abort→feedback cycle terminates on both sides.

## 3. Behaviour details that matter

- **Debounce, 3 samples (60 ms at 50 Hz).** One corrupt mocap frame must not drop four
  drones out of the sky.
- **Faults latch until re-arm.** The condition going away must not silently re-arm the
  fleet mid-flight. Cleared on a rising edge of `armed` (detected in `_safety_check`,
  because arming is handled inside `CallbackManagerMulti` and there is no hook).
- **Nothing faults on the ground.** A drone on its stand may sit tilted and has no
  reference yet; `require_airborne` gates every fault on `armed and takeoff_requested`.
- **Missing signals are not faults.** Signals arrive at different times during startup;
  a checker that faulted on absent data would make every launch an emergency.

## 4. Pre-arm interlock

`CallbackManagerMulti.handle_arming_service` calls an **optional**
`safety_preflight_block()` on the node — returning a reason string refuses arming.
Nodes that do not define it are entirely unaffected, so no other controller changes
behaviour.

Deliberately narrow: no mocap pose, or mocap already stale past the watchdog threshold.
An interlock that blocks arming for marginal reasons gets bypassed, and then protects
nothing.

## 5. Parameters

```yaml
safety_enabled:        true    # false disables the layer entirely (bench tests)
max_tilt_deg:          60.0
max_payload_tilt_deg:  60.0    # 60 so ring-attach at 10-30 deg is not aborted
warn_tilt_deg:         40.0
max_speed:             3.0
safety_ref_timeout_s:  1.0
warn_battery_v:        15.0    # WARN ONLY
```

## 6. What the original draft got wrong (the useful part)

**F4 was overstated.** The draft claimed a drone disarming told nobody. False: the
chain `cb.disarm()` → `publish_arming_state()` → `/drone_i/arming_state_feedback` →
`_arming_feedback_callback` (`main.py:180`) → `_disarm_fleet(emergency=True)` already
existed and works once `flying` is set at TAKEOFF.

The genuine defect was narrower and would have been missed by trusting the summary:
`_disarm_fleet_thread` disarms through each drone's `SetArming` **service** with a 3 s
deadline and **silently skips any drone whose service is not ready**. Fine for an
orderly landing; wrong for an emergency, which must not depend on service
responsiveness at the moment the system is misbehaving. Hence the direct-publish path.

**The health-topic architecture was unnecessary.** The draft proposed `/drone_i/health`
at 10 Hz aggregated by the manager. Since the arming-feedback path already is the abort
bus, that would have duplicated a proven mechanism with an unproven one — in the safety
path, of all places. Deleted before it was built.

**Lesson:** the draft was written from the audit summary rather than the code. Reading
`main.py` first would have produced a smaller, better design immediately.

## 7. Verification status

| Level | Status |
|---|---|
| Unit — `EnvelopeChecker` | **15 tests passing**, including a pose-layout contract test that fails if `current_pose` slicing ever changes |
| Live — manager | **Done.** `/fleet/abort` advertised, per-drone ELRS publishers up, ESTOP broadcast ~1 ms after command |
| Offline gate — dissipative A–K | **ALL PASS** after every change |
| **Gazebo** | **NOT DONE.** Neither the envelope nor the abort path has been exercised in sim |
| **Hardware, props off** | **NOT DONE.** Force one fault, confirm all drones disarm. Required each session |

**The last two are outstanding and this is not flight-verified until they are done.**
