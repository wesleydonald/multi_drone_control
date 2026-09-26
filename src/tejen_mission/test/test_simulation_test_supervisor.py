import unittest

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from tejen_mission.online_join_planner import mission_phase_qos_profile
from tejen_mission.simulation_test_supervisor import (
    SupervisorAction,
    SupervisorConfig,
    SupervisorCore,
    SupervisorSnapshot,
    SupervisorState,
)


def snapshot(*, ready=True, phase="WAIT_FOR_TAKEOFF", handoff=False, armed=None):
    return SupervisorSnapshot(ready, phase, handoff, armed)


class MissionPhaseQosTest(unittest.TestCase):
    def test_phase_qos_is_reliable_transient_local_depth_one(self):
        qos = mission_phase_qos_profile()
        self.assertEqual(qos.depth, 1)
        self.assertEqual(qos.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(qos.durability, DurabilityPolicy.TRANSIENT_LOCAL)


class SupervisorCoreTest(unittest.TestCase):
    def test_automatic_arm_takeoff_success_and_disarm(self):
        core = SupervisorCore(SupervisorConfig(control_mode="automatic"), 0.0)
        decision = core.step(0.1, snapshot(armed=False))
        self.assertEqual(decision.actions, (SupervisorAction.REQUEST_ARM,))
        core.set_arm_result(0.2, True)
        decision = core.step(0.3, snapshot(armed=True))
        self.assertEqual(decision.actions, (SupervisorAction.PUBLISH_TAKEOFF,))
        core.step(0.4, snapshot(phase="TAKEOFF", armed=True))
        core.step(1.0, snapshot(phase="ATTACH_READY", handoff=True, armed=True))
        decision = core.step(3.01, snapshot(phase="ATTACH_READY", handoff=True, armed=True))
        self.assertEqual(decision.actions, (SupervisorAction.REQUEST_DISARM,))
        self.assertEqual(decision.outcome, "PASS")
        decision = core.step(3.1, snapshot(phase="ATTACH_READY", handoff=True, armed=False))
        self.assertEqual(decision.actions, (SupervisorAction.FINISH,))
        self.assertEqual(decision.exit_code, 0)

    def test_success_dwell_resets(self):
        core = SupervisorCore(SupervisorConfig(control_mode="manual"), 0.0)
        core.step(0.1, snapshot(armed=False))
        core.step(0.2, snapshot(phase="TAKEOFF", armed=True))
        core.step(1.0, snapshot(phase="ATTACH_READY", handoff=True, armed=True))
        core.step(2.0, snapshot(phase="MATCH_VELOCITY", handoff=False, armed=True))
        core.step(2.1, snapshot(phase="ATTACH_READY", handoff=True, armed=True))
        self.assertEqual(core.step(3.9, snapshot(phase="ATTACH_READY", handoff=True, armed=True)).actions, ())
        self.assertEqual(
            core.step(4.11, snapshot(phase="ATTACH_READY", handoff=True, armed=True)).actions,
            (SupervisorAction.REQUEST_DISARM,),
        )

    def test_mission_timeout_is_failure(self):
        config = SupervisorConfig(control_mode="manual", mission_timeout_s=2.0)
        core = SupervisorCore(config, 0.0)
        core.step(0.1, snapshot(armed=False))
        core.step(0.2, snapshot(phase="TAKEOFF", armed=True))
        decision = core.step(2.21, snapshot(phase="MATCH_VELOCITY", armed=True))
        self.assertEqual(decision.reason, "MISSION_TIMEOUT")
        self.assertEqual(decision.exit_code, 1)

    def test_premature_disarm_is_failure(self):
        core = SupervisorCore(SupervisorConfig(control_mode="manual"), 0.0)
        core.step(0.1, snapshot(armed=False))
        core.step(0.2, snapshot(phase="TAKEOFF", armed=True))
        decision = core.step(0.3, snapshot(phase="TAKEOFF", armed=False))
        self.assertEqual(decision.reason, "PREMATURE_DISARM")

    def test_early_terminal_phase_is_failure(self):
        core = SupervisorCore(SupervisorConfig(control_mode="manual"), 0.0)
        core.step(0.1, snapshot(armed=False))
        decision = core.step(0.2, snapshot(phase="LANDED_DISARMED", armed=False))
        self.assertEqual(decision.reason, "EARLY_TERMINAL_DISARM")

    def test_takeoff_retries_are_bounded(self):
        config = SupervisorConfig(
            control_mode="automatic", takeoff_retry_s=1.0, takeoff_max_publications=3
        )
        core = SupervisorCore(config, 0.0)
        core.step(0.1, snapshot(armed=False))
        core.set_arm_result(0.2, True)
        self.assertIn(SupervisorAction.PUBLISH_TAKEOFF, core.step(0.3, snapshot(armed=True)).actions)
        self.assertIn(SupervisorAction.PUBLISH_TAKEOFF, core.step(1.31, snapshot(armed=True)).actions)
        self.assertIn(SupervisorAction.PUBLISH_TAKEOFF, core.step(2.32, snapshot(armed=True)).actions)
        decision = core.step(3.33, snapshot(armed=True))
        self.assertEqual(decision.reason, "TAKEOFF_NOT_OBSERVED")

    def test_stale_input_is_infrastructure_failure(self):
        core = SupervisorCore(SupervisorConfig(control_mode="manual"), 0.0)
        core.step(0.1, snapshot(armed=False))
        core.step(0.2, snapshot(phase="TAKEOFF", armed=True))
        decision = core.step(0.3, snapshot(ready=False, phase="TAKEOFF", armed=True))
        self.assertEqual(decision.category, "infrastructure")
        self.assertEqual(decision.exit_code, 2)

    def test_startup_timeout(self):
        config = SupervisorConfig(control_mode="manual", startup_timeout_s=1.0)
        core = SupervisorCore(config, 0.0)
        decision = core.step(1.01, snapshot(ready=False))
        self.assertEqual(decision.reason, "STARTUP_TIMEOUT")
        self.assertEqual(core.state, SupervisorState.STOPPING)


class SupervisorLandingCheckTest(unittest.TestCase):
    def test_landing_check_requests_planner_land_and_accepts_landed_disarmed(self):
        config = SupervisorConfig(
            control_mode="manual",
            success_phase="LANDED_DISARMED",
            success_dwell_s=0.2,
            require_handoff_ready=False,
            failure_phase="__NO_FAILURE_PHASE__",
            landing_request_phase="APPROACH_ABOVE_PICKUP",
        )
        core = SupervisorCore(config, 0.0)
        core.step(0.1, snapshot(armed=False))
        core.step(0.2, snapshot(phase="TAKEOFF", armed=True))
        decision = core.step(
            1.0,
            snapshot(phase="APPROACH_ABOVE_PICKUP", armed=True),
        )
        self.assertEqual(decision.actions, (SupervisorAction.PUBLISH_LAND,))
        self.assertTrue(core.landing_requested)

        decision = core.step(
            2.0,
            snapshot(phase="LANDED_DISARMED", armed=False),
        )
        self.assertEqual(decision.actions, ())
        decision = core.step(
            2.21,
            snapshot(phase="LANDED_DISARMED", armed=False),
        )
        self.assertEqual(decision.actions, (SupervisorAction.REQUEST_DISARM,))
        self.assertEqual(decision.outcome, "PASS")

    def test_normal_supervisor_still_rejects_early_landed_disarmed(self):
        core = SupervisorCore(SupervisorConfig(control_mode="manual"), 0.0)
        core.step(0.1, snapshot(armed=False))
        decision = core.step(0.2, snapshot(phase="LANDED_DISARMED", armed=False))
        self.assertEqual(decision.reason, "EARLY_TERMINAL_DISARM")


if __name__ == "__main__":
    unittest.main()
