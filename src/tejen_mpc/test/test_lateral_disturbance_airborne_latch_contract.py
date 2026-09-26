import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MAIN = ROOT / 'tejen_mpc' / 'tejen_mpc' / 'main.py'


def _load_model_valid_method():
    tree = ast.parse(MAIN.read_text(encoding='utf-8'))
    controller = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'Controller'
    )
    method = next(
        node for node in controller.body
        if isinstance(node, ast.FunctionDef)
        and node.name == '_lateral_disturbance_model_is_valid'
    )
    module = ast.Module(body=[method], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {}
    exec(compile(module, str(MAIN), 'exec'), namespace)
    return namespace['_lateral_disturbance_model_is_valid']


class _Logger:
    def info(self, *args, **kwargs):
        pass


class _DummyController:
    def __init__(self):
        self.enable_lateral_disturbance_observer = True
        self.current_pose = [0.0, 0.0, 0.10]
        self.armed = True
        self.takeoff_requested = False
        self.lateral_disturbance_initial_z = None
        self.lateral_disturbance_airborne_height_m = 0.20
        self.lateral_disturbance_airborne_latched = False

    def get_logger(self):
        return _Logger()


def test_airborne_height_is_a_latch_not_a_continuous_descent_gate():
    model_valid = _load_model_valid_method()
    controller = _DummyController()

    # Armed but still pre-takeoff: establish the ground baseline and clear any
    # stale latch from a prior flight.
    assert not model_valid(controller)
    assert controller.lateral_disturbance_initial_z == 0.10
    assert not controller.lateral_disturbance_airborne_latched

    controller.takeoff_requested = True
    controller.current_pose[2] = 0.29
    assert not model_valid(controller)
    assert not controller.lateral_disturbance_airborne_latched

    controller.current_pose[2] = 0.30
    assert model_valid(controller)
    assert controller.lateral_disturbance_airborne_latched

    # The M1 pickup descent can go back below +0.20 m without disabling/resetting
    # the disturbance observer.
    controller.current_pose[2] = 0.13
    assert model_valid(controller)
    assert controller.lateral_disturbance_airborne_latched


def test_pre_takeoff_state_clears_latch_and_requires_requalification():
    model_valid = _load_model_valid_method()
    controller = _DummyController()
    controller.takeoff_requested = True
    controller.lateral_disturbance_initial_z = 0.10
    controller.current_pose[2] = 0.35

    assert model_valid(controller)
    assert controller.lateral_disturbance_airborne_latched

    # A fresh armed/no-TAKEOFF state is the per-flight reset point used by the
    # existing CallbackManager/controller lifecycle.
    controller.takeoff_requested = False
    controller.current_pose[2] = 0.11
    assert not model_valid(controller)
    assert not controller.lateral_disturbance_airborne_latched
    assert controller.lateral_disturbance_initial_z == 0.11

    controller.takeoff_requested = True
    controller.current_pose[2] = 0.20
    assert not model_valid(controller)
    assert not controller.lateral_disturbance_airborne_latched

    controller.current_pose[2] = 0.31
    assert model_valid(controller)
    assert controller.lateral_disturbance_airborne_latched
