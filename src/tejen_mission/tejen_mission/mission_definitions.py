"""Declarative per-drone mission definitions.

The phase graph is still advanced by explicit guards in the ROS node, but the
reference semantics, target source, magnet state and future controller-handoff
metadata are defined here instead of being scattered through reference code.
"""

from __future__ import annotations

from .mission_types import (
    ControllerAuthority,
    MissionDefinition,
    MissionPhase,
    PhaseSpec,
    ReferenceType,
    TargetSource,
)


def _spec(
    phase: MissionPhase,
    reference_type: ReferenceType,
    target_source: TargetSource,
    magnet_command: str,
    description: str,
    *,
    authority: ControllerAuthority = ControllerAuthority.LOCAL_NMPC,
    handoff_ready: bool = False,
) -> PhaseSpec:
    return PhaseSpec(
        phase=phase,
        reference_type=reference_type,
        target_source=target_source,
        magnet_command=magnet_command,
        controller_authority=authority,
        handoff_ready=handoff_ready,
        description=description,
    )


def pickup_delivery_mission() -> MissionDefinition:
    specs = [
        _spec(
            MissionPhase.WAIT_FOR_TAKEOFF,
            ReferenceType.STATIONARY_REGULATION,
            TargetSource.CURRENT_POSE,
            "OFF",
            "Hold the measured start pose while takeoff is owned externally.",
        ),
        _spec(
            MissionPhase.TAKEOFF,
            ReferenceType.RATE_CONTROLLED_MANOEUVRE,
            TargetSource.CURRENT_POSE,
            "OFF",
            "Rise slowly from the start pose so that takeoff detection can be triggered."
        ),
        _spec(
            MissionPhase.APPROACH_ABOVE_PICKUP,
            ReferenceType.TRANSFER,
            TargetSource.PICKUP_OBJECT,
            "OFF",
            "Smooth transfer to the pickup staging hover.",
        ),
        _spec(
            MissionPhase.SETTLE_ABOVE_PICKUP,
            ReferenceType.STATIONARY_REGULATION,
            TargetSource.PICKUP_OBJECT,
            "OFF",
            "Regulate at the static pickup staging hover.",
        ),
        _spec(
            MissionPhase.DESCEND_TO_PICKUP,
            ReferenceType.RATE_CONTROLLED_MANOEUVRE,
            TargetSource.PICKUP_OBJECT,
            "ON",
            "Descend toward the pickup object at a prescribed relative rate.",
        ),
        _spec(
            MissionPhase.MAGNET_ATTACH_WAIT,
            ReferenceType.STATIONARY_REGULATION,
            TargetSource.PICKUP_OBJECT,
            "ON",
            "Hold the contact configuration until attachment is confirmed.",
        ),
        _spec(
            MissionPhase.LIFT_OBJECT,
            ReferenceType.RATE_CONTROLLED_MANOEUVRE,
            TargetSource.PICKUP_OBJECT,
            "ON",
            "Lift the newly attached object to the loaded staging hover.",
        ),
        _spec(
            MissionPhase.TRANSIT_TO_DROP_POINT,
            ReferenceType.TRANSFER,
            TargetSource.DROP_POINT,
            "ON",
            "Loaded transfer to the drop staging region.",
        ),
        _spec(
            MissionPhase.SETTLE_ABOVE_DROP_POINT,
            ReferenceType.TARGET_RELATIVE_TRACKING,
            TargetSource.DROP_POINT,
            "ON",
            "Establish a target-relative hover above the drop point.",
        ),
        _spec(
            MissionPhase.DESCEND_TO_DROP_HEIGHT,
            ReferenceType.RATE_CONTROLLED_MANOEUVRE,
            TargetSource.DROP_POINT,
            "ON",
            "Descend in the moving drop-point frame.",
        ),
        _spec(
            MissionPhase.DROP_OBJECT,
            ReferenceType.TARGET_RELATIVE_TRACKING,
            TargetSource.DROP_POINT,
            "OFF",
            "Maintain the release pose while the magnet is disabled.",
        ),
        _spec(
            MissionPhase.CLEAR_DROP_ZONE,
            ReferenceType.RATE_CONTROLLED_MANOEUVRE,
            TargetSource.DROP_POINT,
            "OFF",
            "Rise away from the drop point while retaining relative XY tracking.",
        ),
        _spec(
            MissionPhase.TRANSIT_TO_REATTACH,
            ReferenceType.TRANSFER,
            TargetSource.ATTACHMENT_POINT,
            "OFF",
            "Route into the moving-target rendezvous task.",
        ),
        _spec(
            MissionPhase.APPROACH_ABOVE_TARGET,
            ReferenceType.TRANSFER,
            TargetSource.ATTACHMENT_POINT,
            "OFF",
            "Smooth rendezvous transfer to a point above the attachment target.",
        ),
        _spec(
            MissionPhase.MATCH_VELOCITY,
            ReferenceType.TARGET_RELATIVE_TRACKING,
            TargetSource.ATTACHMENT_POINT,
            "OFF",
            "Converge to a target-relative hover and match horizontal velocity.",
        ),
        _spec(
            MissionPhase.DESCEND_TO_ATTACHMENT,
            ReferenceType.RATE_CONTROLLED_MANOEUVRE,
            TargetSource.ATTACHMENT_POINT,
            "ON",   # rejoin: the weld happens at the end of this descent (Wesley 2026-09-25)
            "Descend in the moving attachment-point frame.",
        ),
        _spec(
            MissionPhase.ATTACH_READY,
            ReferenceType.TARGET_RELATIVE_TRACKING,
            TargetSource.ATTACHMENT_POINT,
            "ON",   # hold the weld until the fleet side folds the drone in
            "Maintain the terminal attachment configuration and expose handoff readiness.",
            handoff_ready=True,
        ),
        _spec(
            MissionPhase.LANDING,
            ReferenceType.RATE_CONTROLLED_MANOEUVRE,
            TargetSource.LANDING_POINT,
            "OFF",
            "Operator-requested controlled descent at fixed XY.",
        ),
        _spec(
            MissionPhase.LANDED_DISARMED,
            ReferenceType.STATIONARY_REGULATION,
            TargetSource.LANDING_POINT,
            "OFF",
            "Terminal disarmed state.",
            authority=ControllerAuthority.DISARMED,
        ),
    ]
    return MissionDefinition(
        name="pickup_delivery_reattach",
        initial_phase=MissionPhase.WAIT_FOR_TAKEOFF,
        phases={spec.phase: spec for spec in specs},
    )


def join_only_mission() -> MissionDefinition:
    phases = [
        _spec(
            MissionPhase.APPROACH_ABOVE_TARGET,
            ReferenceType.TRANSFER,
            TargetSource.ATTACHMENT_POINT,
            "OFF",
            "Smooth rendezvous transfer to a point above the attachment target.",
        ),
        _spec(
            MissionPhase.MATCH_VELOCITY,
            ReferenceType.TARGET_RELATIVE_TRACKING,
            TargetSource.ATTACHMENT_POINT,
            "OFF",
            "Converge to a target-relative hover and match horizontal velocity.",
        ),
        _spec(
            MissionPhase.DESCEND_TO_ATTACHMENT,
            ReferenceType.RATE_CONTROLLED_MANOEUVRE,
            TargetSource.ATTACHMENT_POINT,
            "ON",   # rejoin: the weld happens at the end of this descent (Wesley 2026-09-25)
            "Descend in the moving attachment-point frame.",
        ),
        _spec(
            MissionPhase.ATTACH_READY,
            ReferenceType.TARGET_RELATIVE_TRACKING,
            TargetSource.ATTACHMENT_POINT,
            "ON",   # hold the weld until the fleet side folds the drone in
            "Maintain the terminal attachment configuration and expose handoff readiness.",
            handoff_ready=True,
        ),
        _spec(
            MissionPhase.LANDING,
            ReferenceType.RATE_CONTROLLED_MANOEUVRE,
            TargetSource.LANDING_POINT,
            "OFF",
            "Operator-requested controlled descent at fixed XY.",
        ),
        _spec(
            MissionPhase.LANDED_DISARMED,
            ReferenceType.STATIONARY_REGULATION,
            TargetSource.LANDING_POINT,
            "OFF",
            "Terminal disarmed state.",
            authority=ControllerAuthority.DISARMED,
        ),
    ]
    return MissionDefinition(
        name="join_only",
        initial_phase=MissionPhase.APPROACH_ABOVE_TARGET,
        phases={spec.phase: spec for spec in phases},
    )



def m2b_single_attachment_mission() -> MissionDefinition:
    """Single-drone M2B mission using the same declarative phase framework."""
    specs = [
        _spec(MissionPhase.M2_BOOTSTRAP_DETACH, ReferenceType.STATIONARY_REGULATION, TargetSource.CURRENT_POSE, "OFF", "Keep X3 grounded/disarmed while requesting release of the simulator startup joint."),
        _spec(MissionPhase.M2_BOOTSTRAP_SETTLE, ReferenceType.STATIONARY_REGULATION, TargetSource.CURRENT_POSE, "OFF", "Keep X3 grounded/disarmed while fresh detached truth settles."),
        _spec(MissionPhase.M2_WAIT_FOR_ARM, ReferenceType.STATIONARY_REGULATION, TargetSource.CURRENT_POSE, "OFF", "Bootstrap complete; expose arm permission while holding the measured ground pose."),
        _spec(MissionPhase.M2_VERTICAL_TAKEOFF, ReferenceType.RATE_CONTROLLED_MANOEUVRE, TargetSource.CURRENT_POSE, "OFF", "Vertical takeoff using the payload MPC with swing cost disabled."),
        _spec(MissionPhase.M2_TAKEOFF_HOVER, ReferenceType.STATIONARY_REGULATION, TargetSource.CURRENT_POSE, "OFF", "B0 stable free-swing hover after the vertical takeoff."),
        _spec(MissionPhase.M2_CPP_TRANSIT_CAPTURE, ReferenceType.STATIONARY_REGULATION, TargetSource.CURRENT_POSE, "OFF", "B2 C++ collision-avoiding transit to the exact B1 capture hover."),
        _spec(MissionPhase.M2_SETTLE_CAPTURE, ReferenceType.STATIONARY_REGULATION, TargetSource.CURRENT_POSE, "OFF", "B1 commissioning entry: settle at the known capture hover."),
        _spec(MissionPhase.M2_ATTACH_APPROACH_HIGH, ReferenceType.RATE_CONTROLLED_MANOEUVRE, TargetSource.CURRENT_POSE, "OFF", "High-band local descent with XY-conditioned Z progress."),
        _spec(MissionPhase.M2_ATTACH_APPROACH_LOW, ReferenceType.RATE_CONTROLLED_MANOEUVRE, TargetSource.CURRENT_POSE, "OFF", "Low-band precision descent with the tighter XY gate."),
        _spec(MissionPhase.M2_ATTACH_CAPTURE_WAIT, ReferenceType.STATIONARY_REGULATION, TargetSource.CURRENT_POSE, "ON", "Near-contact hold while simulated physical latch and software candidate evidence develop; after latch, freeze the pre-proof reference until measured dynamics settle."),
        _spec(MissionPhase.M2_ATTACH_PROOF, ReferenceType.RATE_CONTROLLED_MANOEUVRE, TargetSource.CURRENT_POSE, "ON", "Execute the configured measured-excitation proof pull; simulation defaults to radial-outward."),
        _spec(MissionPhase.M2_ATTACHED_HOLD, ReferenceType.STATIONARY_REGULATION, TargetSource.CURRENT_POSE, "ON", "Hold the confirmed attachment at the outward tension angle."),
        _spec(MissionPhase.M2_DETACHED_RETREAT, ReferenceType.RATE_CONTROLLED_MANOEUVRE, TargetSource.CURRENT_POSE, "OFF", "Retreat clear and return to the same capture point before a same-plate retry."),
        _spec(MissionPhase.M2_LANDING_STAGE, ReferenceType.TRANSFER, TargetSource.CURRENT_POSE, "OFF", "After terminal failure, move clear to the landing column before vertical landing."),
        _spec(MissionPhase.M2_FAULT, ReferenceType.STATIONARY_REGULATION, TargetSource.CURRENT_POSE, "OFF", "Latched pre-arm or mission safety fault; flight authority is not granted.", authority=ControllerAuthority.DISARMED),
        _spec(MissionPhase.LANDING, ReferenceType.RATE_CONTROLLED_MANOEUVRE, TargetSource.LANDING_POINT, "OFF", "Controlled vertical landing after a clear retreat."),
        _spec(MissionPhase.LANDED_DISARMED, ReferenceType.STATIONARY_REGULATION, TargetSource.LANDING_POINT, "OFF", "Terminal disarmed state.", authority=ControllerAuthority.DISARMED),
    ]
    return MissionDefinition(
        name="m2b_single_attachment",
        initial_phase=MissionPhase.M2_BOOTSTRAP_DETACH,
        phases={spec.phase: spec for spec in specs},
    )

def mission_for_mode(mode: str) -> MissionDefinition:
    normalized = str(mode).strip().lower()
    if normalized in {"pickup_delivery", "pickup-delivery", "full"}:
        return pickup_delivery_mission()
    if normalized in {"join", "join_only", "reattach"}:
        return join_only_mission()
    if normalized in {"m2b", "m2b_single_attachment", "single_attachment"}:
        return m2b_single_attachment_mission()
    raise ValueError(
        f"Unknown mission_mode={mode!r}; expected 'pickup_delivery', 'join', or 'm2b'."
    )
