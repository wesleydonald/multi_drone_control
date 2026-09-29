# Card: fleet abort latched in every ELRS mux (G7 prerequisite P7)

Date: 2026-09-28. Type: safety gate that can hold the fleet down -> critic required (working rules). Status: v3 2026-09-28 (critic pass 2 NOT READY on text; v3 applies F1-F7 and Wesley's F5 ruling).

## Problem
A fault anywhere disarms the fleet through the fleet manager: it publishes `/fleet/abort` and sends a direct ELRS disarm
to every drone (controller_quad_load main.py:116-129, 173-186). Behind a mux (src/drone_magnet/drone_magnet/elrs_mux.py)
the radio's last hop is the mux output `/drone_{id}/ELRSCommand`. Tejen's MPC ignores `/fleet/abort`, and while his stream
is selected (before the weld/hand-over in M1, before the M2 hand-over) his next command (armed=True) passes the mux
and re-arms the drone after our disarm. In the M2 launch, before the hand-over, our emergency stop reaches no radio at
all (ladder §2, P7). On the rig this means an abort can be undone within one command period (30 Hz).

## Change (one variable)
`elrs_mux` subscribes `/fleet/abort` (the same message type the fleet manager publishes). On the first abort it latches:
from then on every forwarded command has `armed=False` and throttle channel at idle (-1), whichever input is selected,
until the node is restarted. Parameter `abort_latch` (default true), `abort_topic` (default `/fleet/abort`). Log once, loud.
Optional second hop (P7b, separate card if wanted): the same latch in `elrs_interface` (the last hop on the rig, and the
only one that also covers a drone with no mux).

## Why a latch and not a pass-through of the disarm
A disarm message is one sample; the next input sample re-arms. Only a state that outlives the next sample holds the drone
down. Restart-to-clear matches the fleet manager (an abort ends the flight).

## Risks the critic should weigh
- A spurious `/fleet/abort` (e.g. from a stale test node on the same ROS domain) grounds the fleet mid-flight: the same
  consequence as today's fleet disarm, but no longer reversible by Tejen's stream. Mitigation: clean_slate before every
  run; ROS_DOMAIN_ID per lab.
- Sim: M1/M2 runs that currently finish with an abort path must still end the same way (forbid_abort criteria unchanged).
- A drone disarmed in the air falls: the locked decision is fleet-wide disarm on any envelope fault; the latch only makes
  that decision stick. It does not add a new trigger.

## Verification plan
1. pytest on the policy: abort before/after hand-over, both inputs, restart clears; no effect without abort.
2. One Gazebo M1 (partner_attached_orbit) and one M2 bench run: identical outcome with no abort (the latch is inert).
3. One Gazebo abort test: M2 bench with a forced `/fleet/abort` published before the hand-over (runner GZ/ROS publish event):
   all four drones end disarmed and stay disarmed while Tejen's stream keeps publishing armed=True (on the bench his stream
   is absent -> add a stub publisher, or use the full M2 before the hand-over).
Falsifier: any forwarded armed=True after an abort; any change in a no-abort run's outcome.

## Critic
VERDICT: NOT READY (critic 2026-09-28; pasted by the loop, the critic had read-only tools).
1. Mechanism: real in sim and stronger than stated: elrs_interface.py:206 takes `armed` from the latest message, and the
   sim bridges ignore `armed` for thrust (payload_betaflight_comm.py:239-246 fly on channel_2; tejen bridge :271 armed
   check commented out), so his next sample restores thrust regardless. On hardware the re-arm is UNVERIFIED (Betaflight
   arming-disable flags probably refuse a re-arm at high throttle). Simpler rig explanation = wiring: in M2 the manager's
   direct emergency ELRS publishers are remapped to `_diss` (dissipative_launch.py:184, 279-282) and the mux drops `_diss`
   before the hand-over (elrs_mux.py:139-140), so our ESTOP reaches no radio; and until P14 his IRL MPC publishes straight
   to the radio topic (no `_tejen` remap), so a mux latch covers none of his stream on the rig.
2. Falsifier checks `armed`, which the sim ignores: require channel_2 == -1 and height/motor-speed loss, plus a
   latch-off baseline arm with the same forced abort. R0618's log (our tracker fired the abort at 100.7 s while the mux
   forwarded his stream) shows the problem for free.
3. It ADDS a trigger: /fleet/abort (std_msgs/String, one publisher main.py:125, fired at :296) also fires on ARM failed
   (main.py:246), which happens (R0560, T0007). In M2 we ARM while his four drones hold the ring in the air; with the latch
   an ARM failure of our non-flying stack drops his fleet and the ring. Needs Wesley's ruling (and whether ARM-failed should
   raise /fleet/abort at all: separate manager card).
4. "Restart-to-clear matches the manager" is wrong: the manager accepts ARM again after ESTOP (main.py:155-156, 288) and
   trackers clear their fault on re-arm (controller_mpc.py:646-652) -> re-ARM without relaunching the muxes gives armed
   trackers with motors held off and integrators winding up. Test it.
5. Design: latch as a node attribute checked in `_forward` (elrs_mux.py:116), NOT in HandoverPolicy (rebuilt on release,
   :123); publish one disarm immediately on abort (sim bridges hold the last command); override armed=False, channel_2=-1,
   rates 0, keep the magnet merge; pure function + pytest. P7b (elrs_interface) on its own card with the rig sheet.
6. Verification: trigger by ESTOP on /fleet/command (real remap path), latch-off baseline, neighbours (M1 ESTOP during his
   mission; full M2 before the hand-over with his real MPC; ARM-failed before the hand-over; re-ARM without relaunch).
7. Not needed before Wed (R0-R4 have no mux); prerequisite for R8a and M2b.


## v2 (2026-09-28, after the critic and Wesley's rulings Q6a/Q6b)
Rulings: (a) an operator ESTOP or a fault on a flying drone grounds EVERY drone, the partner's included; (b) an ARM failure
of our stack refuses TAKEOFF and disarms our drones but no longer raises /fleet/abort.

Mechanism (critic 1): sim re-arm is real and stronger than v1 said: the sim bridges ignore `armed` and fly on channel_2
(payload_betaflight_comm.py:239-246; tejen bridge :271 armed check commented out), so the partner's next sample restores
thrust after our disarm. On hardware the re-arm is unverified (Betaflight arming checks); the rig gap is wiring: before the
hand-over our ESTOP reaches no radio (manager's direct ELRS publishers remapped to `_diss`, dissipative_launch.py:184,
279-282; the mux drops `_diss` before the hand-over, elrs_mux.py:139-140). R0618's log (our tracker fired the abort at
100.7 s while the mux forwarded the partner's stream) is read first as the free baseline.

Changes (two variables, two commits, tested separately):
1. Manager (main.py:246): ARM failed -> disarm our drones + refuse TAKEOFF, no /fleet/abort publish. Test: pytest on the
   manager's ARM-failed path; one M2 bench run with a forced ARM failure shows no abort and no effect on the partner.
2. Mux latch: a node attribute checked in `_forward` (elrs_mux.py:116), NOT in HandoverPolicy (rebuilt on release, :123).
   On the first /fleet/abort: publish one disarm immediately (the sim bridges hold the last command), then every forwarded
   command has armed=False, channel_2=-1, rates 0 (magnet merge kept), whichever input is selected. Cleared ONLY by a node
   restart; because the manager accepts ARM again after an ESTOP and the trackers clear their fault on re-arm, the manager
   must REFUSE ARM while any mux reports latched (mux publishes /drone_i/mux_latched Bool, manager reads it) - otherwise a
   re-ARM gives armed trackers with motors held off and integrators winding up. Pure function + pytest.
   Parameter `abort_latch` default true.
Verification (critic 3/6): trigger by ESTOP on /fleet/command (the real remap path), never a runner publish of /fleet/abort.
Arms, one Gazebo run each: (A) latch-off baseline on the full M2 BEFORE the hand-over (his real MPC flying): ESTOP ->
expected today: his drones keep flying; (B) latch-on, same: all four drones end with channel_2 = -1 and motor speed 0 / a
height loss within 1 s, and stay down while his stream publishes armed=True; (C) M1 ESTOP during his mission (drone 3
under his MPC) latch-on: drone 3 grounded; (D) re-ARM after a latch without relaunching: refused with a clear log; (E) a
no-abort canonical M1 and M2 bench: unchanged outcome (latch inert). Falsifier: any forwarded command with channel_2 > -1
after an abort, any partner drone still producing thrust 1 s after the ESTOP in B/C, any change in E, a re-ARM accepted in D.
Rig: P7b (the same latch in elrs_interface, the last hop that also covers a partner stream with no mux) on its own card with
a props-off desk test and the rig sheet before R10b/M2b. Not needed for Wednesday (no mux in R0-R4).


## Critic pass 2 (2026-09-28, condensed)
NOT READY, card text only. F1 change 1 must disarm through the SERVICE (_disarm_fleet(emergency=False)), never the emergency
direct-ELRS path (in M1 the manager is not remapped: its direct publisher for drone 3 writes the mux OUTPUT = the radio),
and set shutdown_requested (relaunch to retry). F2 /drone_i/mux_latched TRANSIENT_LOCAL both ends; no publisher = not
latched (R0-R4 and canonical launches unchanged); check first in _arm_fleet_thread with its own log; holds for the manager
path only (drone 3's tracker takes ARM from /fleet/command directly in M1: a known hole, physically safe because the latch
holds the motors). F3 arm D as written cannot fail (an ESTOP already SIGTERMs every tracker; a same-launch re-ARM fails
today): test the split M2 launch (muxes stay latched, controllers part relaunched, ARM refused on the gate's log). F4 add an
ESTOP event to the runner (config.py EVENT_KINDS + runner_node _fire); abort arms need forbid_abort false and
stop_after_abort_s >= the stay-down window; arms A/B are drive_m2_handover.py runs with a switch
(M2_ESTOP_BEFORE_HANDOVER=1: ESTOP on /fleet/command after 'our fleet armed' instead of /fleet/handover, record ~10 s);
the bench has no partner stream -> change 1 is pytest only; arm C = partner_attached_orbit + ESTOP after
WAIT_PARTNER_RELEASE (R0618 = free latch-off baseline). F5 idle-tracker faults (R0618) -> Wesley ruled. F6 observables:
d{i}_thr / d{i}_armed / d{i}_z (no motor speed logged); B must prove his stream stayed armed above idle after the ESTOP
(bag /drone_*/ELRSCommand*); per-message property belongs to the pytest. F7 before our controllers launch there is no
manager: out of scope (his kill path governs). Optional: the latched mux republishes the disarm on a timer.

## v3 (2026-09-28): build spec
Ruling F5 (Wesley): faults of a tracker whose drone is not under our command (mux still forwarding the partner) do NOT fire
the fleet abort; operator ESTOP and faults of commanded drones ground everyone.
1. Manager (main.py): ARM failed -> _disarm_fleet(emergency=False) (service path) + refuse TAKEOFF + shutdown_requested;
   no /fleet/abort, no direct ELRS publish. Pytest only.
2. Manager: a fault reported by a tracker whose drone is not under our command does not escalate to /fleet/abort. "Under our
   command" = its mux has switched to our stream (the mux publishes /drone_i/mux_state String 'partner'|'ours'|'latched',
   TRANSIENT_LOCAL); a drone with no mux is always ours. The tracker still disarms itself (its own safety) - check whether that
   disarm reaches the radio when the mux forwards the partner (it must not: the mux forwards only the selected stream).
3. Mux latch (elrs_mux.py): attribute checked in _forward; on the first /fleet/abort publish one disarm immediately and then
   every forwarded command armed=False, channel_2=-1, rates 0 (magnet merge kept), whichever input is selected; also
   republish the disarm on a 20 Hz timer while latched; mux_state 'latched'. Cleared only by a node restart.
4. Manager ARM gate: FIRST in _arm_fleet_thread, refuse ARM while any /drone_i/mux_state is 'latched' (TRANSIENT_LOCAL
   subscription; no publisher = not latched), own log line "ARM REFUSED: mux latched on drone i (relaunch the muxes)".
   Known hole: M1's drone-3 tracker takes ARM from /fleet/command directly; the latch still holds its motors.
5. Runner: ESTOP event (config.py EVENT_KINDS, runner_node _fire -> /fleet/command 'ESTOP' as the arm panel sends it; check
   the exact string). Driver: M2_ESTOP_BEFORE_HANDOVER=1 switch; bag /drone_*/ELRSCommand* in those runs.
Verification: pytest for 1-4 (latch function per message, mux_state QoS, gate, fault scoping). Gazebo, one run each:
C (partner_attached_orbit + ESTOP after WAIT_PARTNER_RELEASE: drone 3 under his MPC grounded, d3_thr 0 and d3_z falling
within 1 s, his stream still armed in the bag) vs R0618; B (driver M2 ESTOP before hand-over, latch on: all four grounded,
his streams still armed); A (same, abort_latch false: expected today, his drones keep flying) only if B is ambiguous;
E (canonical M1 no abort: unchanged vs R0724); D as a desk test (split launch, no Gazebo). Budget: C + E runner runs,
B (+A) driver runs: 3-4 Gazebo runs.

## Addendum: operator DISARM latches too (Wesley 2026-09-28)
The rig kill switch is the panel's DISARM, not ESTOP. main.py's DISARM branch now publishes /fleet/abort
('operator DISARM') before the service disarm, so the muxes latch on it. Runner gained a DISARM event (same
forbid_abort / stay-down validation as ESTOP); arm C was flown with DISARM, the rig's actual button.
Pytest: test_operator_disarm_latches_the_muxes_through_fleet_abort.

## Result (2026-09-29)
| arm | run | baseline | measure | value | verdict |
|---|---|---|---|---|---|
| C (DISARM after PARTNER RELEASE, drone 3 on his MPC) | R0751 | R0618 | d3 on the ground after DISARM | 0.52 s | PASS |
| C | R0751 | | his /drone_3/ELRSCommand_tejen after the abort | armed, thr -0.55..-0.60 to bag end | latch holds it off |
| E (canonical M1, no abort) | R0752 | R0724 | DETACH / DROP / REWELD peak (deg) | 2.80 / 2.53 / 2.74 (3.00 / 5.29 / 1.71) | PASS, latch inert |
| B (M2 ESTOP before hand-over) | T0032, T0033 | T0031 | | | VOID x2 (his join stalled in drone 0 transit capture before our stack ran; time box) |
| B re-fly with his padding fix | T0034 | T0031 | mux output armed-or-above-idle after ESTOP; his streams armed | 0 x4 (2511 msgs each); 301 armed each; all four on the floor | PASS |
