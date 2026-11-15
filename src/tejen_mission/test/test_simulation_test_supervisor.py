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


class StopActionTest(unittest.TestCase):
    """multi_drone_control combined rig (H5): stop_action 'land' turns the stop's disarm request into
    /join_planner/land_now; 'disarm' (the default) is his behaviour."""

    def _run(self, stop_action):
        import os
        import rclpy
        from tejen_mission.simulation_test_supervisor import SimulationTestSupervisor, SupervisorDecision

        class Rec:
            def __init__(self):
                self.msgs = []

            def publish(self, m):
                self.msgs.append(m)

            def get_subscription_count(self):
                return 1

        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        args = ['--ros-args', '-p', 'result_path:=/tmp/stop_action_test.json']
        if stop_action:
            args += ['-p', f'stop_action:={stop_action}']
        rclpy.init(args=args, domain_id=95)
        try:
            node = SimulationTestSupervisor()
            node.land_publisher, node.command_publisher = Rec(), Rec()
            if node.give_up_publisher is not None:
                node.give_up_publisher = Rec()
                self.give_ups = node.give_up_publisher
            node.arming_client = type('C', (), {'service_is_ready': lambda self: False})()
            node.core.step = lambda now, snap: SupervisorDecision(
                actions=(SupervisorAction.REQUEST_DISARM,), outcome='FAIL', reason='REQUIRED_INPUT_STALE')
            node._tick()
            out = ([m.data for m in node.land_publisher.msgs], [m.data for m in node.command_publisher.msgs])
            node.destroy_node()
            return out
        finally:
            rclpy.shutdown()

    def test_default_disarms(self):
        self.assertEqual(self._run(''), ([], ['DISARM']))

    def test_land_lands_and_never_disarms(self):
        self.assertEqual(self._run('land'), ([True], []))

    def test_give_up_gives_up_and_neither_lands_nor_disarms(self):
        self.assertEqual(self._run('give_up'), ([], []))
        self.assertEqual([m.data for m in self.give_ups.msgs], [True])


class ReturnLineSuccessTest(unittest.TestCase):
    """success_after_phase (15 Nov, Te-Jen and Codex's v3 review): on a return-after-drop line the twin supervisor counts
    LANDED_DISARMED at the start as success once AT_START was seen; a landing before it and a disarm in flight still
    fail, and the default configuration is unchanged."""

    RETURN = dict(control_mode="manual", success_phase="LANDED_DISARMED", success_after_phase="AT_START",
                  require_handoff_ready=False, failure_phase="LANDED_DISARMED")

    def fly(self, config, phases):
        core = SupervisorCore(config, 0.0)
        core.step(0.1, snapshot(armed=False))
        core.step(0.2, snapshot(phase="TAKEOFF", armed=True))
        t, decision = 0.2, None
        for phase, armed in phases:
            t += 0.5
            decision = core.step(t, snapshot(phase=phase, armed=armed))
            if decision.outcome:
                return decision
        return decision

    def test_landing_at_the_start_after_at_start_is_a_pass(self):
        decision = self.fly(SupervisorConfig(**self.RETURN), [
            ("CLEAR_DROP_ZONE", True), ("RETURN_TO_START", True), ("AT_START", True), ("LANDING", True),
            ("LANDED_DISARMED", False), ("LANDED_DISARMED", False), ("LANDED_DISARMED", False),
            ("LANDED_DISARMED", False), ("LANDED_DISARMED", False), ("LANDED_DISARMED", False)])
        self.assertEqual((decision.outcome, decision.reason), ("PASS", "SUCCESS_DWELL_REACHED"))

    def test_a_landing_before_at_start_is_still_an_early_terminal_disarm(self):
        decision = self.fly(SupervisorConfig(**self.RETURN), [
            ("CLEAR_DROP_ZONE", True), ("RETURN_TO_START", True), ("LANDING", True), ("LANDED_DISARMED", False)])
        self.assertEqual((decision.outcome, decision.reason), ("FAIL", "EARLY_TERMINAL_DISARM"))

    def test_a_disarm_in_flight_is_still_premature(self):
        decision = self.fly(SupervisorConfig(**self.RETURN), [
            ("RETURN_TO_START", True), ("AT_START", True), ("AT_START", False)])
        self.assertEqual((decision.outcome, decision.reason), ("FAIL", "PREMATURE_DISARM"))

    def test_the_default_still_fails_the_same_landing(self):
        decision = self.fly(SupervisorConfig(control_mode="manual", success_phase="NONE_PARTNER_WELD_OWNS_SUCCESS"), [
            ("RETURN_TO_START", True), ("AT_START", True), ("LANDING", True), ("LANDED_DISARMED", False)])
        self.assertEqual((decision.outcome, decision.reason), ("FAIL", "EARLY_TERMINAL_DISARM"))
