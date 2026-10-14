"""Declarative per-drone mission definitions.

The phase graph is still advanced by explicit guards in the ROS node, but the
reference semantics, target source, magnet state and future controller-handoff
metadata are defined here instead of being scattered through reference code.
"""

from __future__ import annotations

from dataclasses import replace

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


def _contact_attach_specs(specs):
    """contact_attach (combined M1 subtest): the fleet's attach confirmation hands the vehicle
    over, so ATTACH_READY drops the hand-back flag; three phases join the rejoin."""
    specs = [
        replace(spec, handoff_ready=False,
                description="Hold the contact target until the fleet's manager reports contact.")
        if spec.phase == MissionPhase.ATTACH_READY else spec
        for spec in specs
    ]
    return specs + [
        _spec(
            MissionPhase.LIFT_PROOF,
            ReferenceType.RATE_CONTROLLED_MANOEUVRE,
            TargetSource.ATTACHMENT_POINT,
            "ON",
            "Rise plate-relative by at most proof_rise_max while the fleet's manager proves the catch.",
        ),
        _spec(
            MissionPhase.RING_ATTACHED,
            ReferenceType.TARGET_RELATIVE_TRACKING,
            TargetSource.ATTACHMENT_POINT,
            "NONE",
            "Attach confirmed: hold the lifted plate offset until the fleet's controller has the "
            "vehicle, then hold where it is. Terminal.",
            authority=ControllerAuthority.EXTERNAL_CONTROLLER,
        ),
        _spec(
            MissionPhase.TAKEN_BACK,
            ReferenceType.STATIONARY_REGULATION,
            TargetSource.CURRENT_POSE,
            "NONE",
            "The fleet's controller took the vehicle back without an attach: hold the last "
            "reference, no magnet commands. Terminal.",
            authority=ControllerAuthority.EXTERNAL_CONTROLLER,
        ),
    ]


def pickup_delivery_mission(contact_attach: bool = False) -> MissionDefinition:
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
            "Descend toward pickup; legacy uses a timed profile/magnet ON, "
            "measured-tip opt-in gates HIGH/LOW progress and switches ON at LOW entry.",
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
            "Lift a pickup candidate; measured-tip M1 proves co-motion before loaded transfer.",
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
    if contact_attach:
        specs = _contact_attach_specs(specs)
    return MissionDefinition(
        name="pickup_delivery_reattach",
        initial_phase=MissionPhase.WAIT_FOR_TAKEOFF,
        phases={spec.phase: spec for spec in specs},
        takeoff_exit=MissionPhase.APPROACH_ABOVE_PICKUP,
    )


PICKUP_PHASES = (
    MissionPhase.APPROACH_ABOVE_PICKUP,
    MissionPhase.SETTLE_ABOVE_PICKUP,
    MissionPhase.DESCEND_TO_PICKUP,
    MissionPhase.MAGNET_ATTACH_WAIT,
    MissionPhase.LIFT_OBJECT,
)


def floor_join_mission(contact_attach: bool = False) -> MissionDefinition:
    """floor_join (Wesley 14 Oct 2026): pickup_delivery's start on the mat, then join_only's rejoin; no pickup, no
    drop. The planner's takeoff climbs to the approach height before the transfer to the plate."""
    start = pickup_delivery_mission(contact_attach).phases
    phases = {
        MissionPhase.WAIT_FOR_TAKEOFF: start[MissionPhase.WAIT_FOR_TAKEOFF],
        MissionPhase.TAKEOFF: start[MissionPhase.TAKEOFF],
        **join_only_mission(contact_attach).phases,
    }
    return MissionDefinition(
        name="floor_join",
        initial_phase=MissionPhase.WAIT_FOR_TAKEOFF,
        phases=phases,
        takeoff_exit=MissionPhase.APPROACH_ABOVE_TARGET,
    )


def delivery_rejoin_mission(contact_attach: bool = False) -> MissionDefinition:
    """delivery_rejoin (Wesley 14 Oct 2026): pickup_delivery without the pickup. The object starts on the magnet on
    the mat, so the magnet is ON from boot; the planner's takeoff ends where a loaded lift ends and prepares the
    loaded transit there, as LIFT_OBJECT does after a pickup."""
    held = {
        MissionPhase.WAIT_FOR_TAKEOFF: "Hold the measured start pose with the object on the magnet while takeoff "
                                       "is owned externally.",
        MissionPhase.TAKEOFF: "Rise with the object to the loaded-lift height, then prepare the loaded transfer.",
    }
    phases = {
        phase: replace(spec, magnet_command="ON", description=held[phase]) if phase in held else spec
        for phase, spec in pickup_delivery_mission(contact_attach).phases.items()
        if phase not in PICKUP_PHASES
    }
    return MissionDefinition(
        name="delivery_rejoin",
        initial_phase=MissionPhase.WAIT_FOR_TAKEOFF,
        phases=phases,
        takeoff_exit=MissionPhase.TRANSIT_TO_DROP_POINT,
    )


def join_only_mission(contact_attach: bool = False) -> MissionDefinition:
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
    if contact_attach:
        phases = _contact_attach_specs(phases)
    return MissionDefinition(
        name="join_only",
        initial_phase=MissionPhase.APPROACH_ABOVE_TARGET,
        phases={spec.phase: spec for spec in phases},
    )



def m2b_single_attachment_mission(attachment_policy: str = "legacy") -> MissionDefinition:
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
    if attachment_policy == "measured_contact_15deg":
        descriptions = {
            MissionPhase.M2_ATTACH_CAPTURE_WAIT: "Bounded measured contact search; initial contact dwell starts the 15-degree proof without waiting for the nominal floor.",
            MissionPhase.M2_ATTACH_PROOF: "Constant-radius outward 15-degree proof; magnet ON until explicit release or convincing loss.",
            MissionPhase.M2_ATTACHED_HOLD: "Preserve the proven contact point and snapshotted tether radius.",
            MissionPhase.M2_DETACHED_RETREAT: "Explicit release, vertical clearance, measured separation, then capture return or operator retry wait.",
            MissionPhase.M2_LANDING_STAGE: "Operator LAND: release and verify vertical clearance before lateral landing-column travel.",
            MissionPhase.M2_FAULT: "Control/authority fault; magnet state is retained by the runtime adapter, never implicitly released.",
        }
        specs = [replace(spec, description=descriptions.get(spec.phase, spec.description)) for spec in specs]
    return MissionDefinition(
        name="m2b_single_attachment",
        initial_phase=MissionPhase.M2_BOOTSTRAP_DETACH,
        phases={spec.phase: spec for spec in specs},
    )

def mission_for_mode(mode: str, *, attachment_policy: str = "legacy",
                     contact_attach: bool = False) -> MissionDefinition:
    normalized = str(mode).strip().lower()
    if normalized in {"pickup_delivery", "pickup-delivery", "full"}:
        return pickup_delivery_mission(contact_attach)
    if normalized in {"join", "join_only", "reattach"}:
        return join_only_mission(contact_attach)
    if normalized in {"floor_join", "floor-join"}:
        return floor_join_mission(contact_attach)
    if normalized in {"delivery_rejoin", "delivery-rejoin"}:
        return delivery_rejoin_mission(contact_attach)
    if normalized in {"m2b", "m2b_single_attachment", "single_attachment"}:
        if contact_attach:
            raise ValueError("contact_attach is an M1 rejoin mode; m2b has its own attachment phases.")
        return m2b_single_attachment_mission(attachment_policy)
    raise ValueError(
        f"Unknown mission_mode={mode!r}; expected 'pickup_delivery', 'join', 'floor_join', "
        "'delivery_rejoin', or 'm2b'."
    )
