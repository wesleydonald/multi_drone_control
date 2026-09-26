"""Source-level regression contract for the normal M1 landing authority path."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PLANNER = (
    Path(__file__).resolve().parents[1]
    / 'tejen_mission'
    / 'online_join_planner.py'
)


def test_external_landing_request_enters_planner_landing_state():
    text = PLANNER.read_text(encoding='utf-8')
    assert 'def external_landing_callback(self, msg: Bool) -> None:' in text
    assert 'self.transition_to(\n            MissionPhase.LANDING,' in text
    assert 'f"external landing request on {self.external_landing_topic}"' in text


def test_landing_state_owns_descent_reference_and_disarm_transition():
    text = PLANNER.read_text(encoding='utf-8')
    assert 'elif new_phase == MissionPhase.LANDING:' in text
    assert 'self.initialise_landing()' in text
    assert 'if self.phase == MissionPhase.LANDING:\n            return self.build_landing_reference()' in text
    assert 'def build_landing_reference(self) -> TrajectoryReference:' in text
    assert 'rate=self.landing_descent_speed' in text
    assert 'def maybe_publish_landing_disarm(self) -> None:' in text
    assert 'message.data = self.landing_disarm_command' in text
    assert 'MissionPhase.LANDED_DISARMED' in text


def test_main_planner_tick_publishes_landing_reference_before_disarm_check():
    text = PLANNER.read_text(encoding='utf-8')
    publish_index = text.rfind('self.publish_reference(reference)')
    disarm_index = text.rfind('self.maybe_publish_landing_disarm()')
    assert publish_index > 0
    assert disarm_index > publish_index


def test_sim_supervisor_can_request_and_accept_state_machine_landing():
    supervisor = (
        ROOT / "src" / "tejen_mission" / "tejen_mission"
        / "simulation_test_supervisor.py"
    ).read_text()
    assert 'PUBLISH_LAND = "PUBLISH_LAND"' in supervisor
    assert 'landing_request_phase: str = ""' in supervisor
    assert 'self.land_publisher = self.create_publisher(Bool, "/join_planner/land_now", 5)' in supervisor
    assert 'snapshot.phase == self.config.landing_request_phase' in supervisor
    assert 'snapshot.phase == self.config.success_phase' in supervisor
    assert 'or not self.config.require_handoff_ready' in supervisor
