import rclpy
import signal
import sys
import numpy as np
import copy
import json
import math
import os
import threading
from rclpy.node import Node
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import time
from collections import deque
# from .acados import generate_ocp_controller, set_initial_guess, warm_start_from_previous_solution, set_trajectory_reference_aligned, update_ocp_parameters
from .acados import (
    generate_payload_ocp_controller,
    set_initial_guess,
    warm_start_from_previous_solution,
    set_payload_trajectory_reference_aligned,
    update_payload_ocp_parameters
)
from .trajectories import zeng_simplified_payload_trajectory, hover_trajectory, z_sin_trajectory, xyz_sine_trajectory, circle_trajectory, power_loop_trajectory, figure8_zsine_trajectory, fast_xyz_sine_trajectory
from tejen_utility_objects.visualization import TrajectoryVisualizer
from tejen_utility_objects.data_logger import DataLogger
from tejen_utility_objects.callback_manager import CallbackManager
from interfaces.msg import MotionCaptureState, ELRSCommand, Telemetry
from trajectory_msgs.msg import MultiDOFJointTrajectory
from std_msgs.msg import Bool, String
from scipy.linalg import cholesky
from .thrust_ratio_ukf import ThrustRatioUKF
from .thrust_ratio_vertical_ukf import VerticalThrustRatioUKF
from .thrust_ratio_full_model_ukf import FullModelThrustRatioUKF
from .mpc_modes import (
    PayloadMpcMode,
    apply_payload_mpc_mode,
    base_payload_cost_matrices,
    parse_payload_mpc_mode,
)
from .lateral_disturbance_observer import (
    LateralDisturbanceObserver,
    nominal_lateral_acceleration,
    parse_xy_bias_mode,
)
# from vector_map import VectorMap


POSE_TIMEOUT_THRESHOLD = 0.25  # seconds
USE_MOTION_CAPTURE = True  # Set to False to use ORB-SLAM data instead
FREQUENCY_HZ = 30.0
DT = 1.0 / FREQUENCY_HZ

LOGGING_NAME = 'tejen_mpc'
    
class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.current_pose = None
        self.last_pose_update_time = None
        self.pendulum_state = np.zeros(4)
        self.last_pendulum_update_time = None

        # General Settings
        self.cb = CallbackManager(self)

        # Optional external arm interlock. Historical controller behavior is
        # unchanged unless a mission explicitly opts in. M2B uses this to make
        # the Gazebo bootstrap-detach verification a real pre-arm gate.
        self.require_external_arm_permission = bool(
            self.declare_parameter('require_external_arm_permission', False).value
        )
        self.external_arm_permission_topic = str(
            self.declare_parameter(
                'external_arm_permission_topic', '/join_planner/arm_permission'
            ).value
        )
        self.external_arm_permission = not self.require_external_arm_permission
        self.external_arm_permission_received = False
        self.external_arm_permission_subscription = None
        if self.require_external_arm_permission:
            self.external_arm_permission_subscription = self.create_subscription(
                Bool,
                self.external_arm_permission_topic,
                self.external_arm_permission_callback,
                10,
            )
            self.get_logger().info(
                'External arm permission required on '
                f'{self.external_arm_permission_topic}'
            )

        # Runtime payload-MPC cost mode is mission-selected. The default is the
        # exact historical FREE_SWING cost, so existing missions are unchanged.
        self.payload_mpc_mode_topic = str(
            self.declare_parameter(
                'payload_mpc_mode_topic', '/join_planner/mpc_mode'
            ).value
        )
        self.current_payload_mpc_mode = PayloadMpcMode.FREE_SWING
        self.payload_mpc_mode_subscription = None
        self.payload_mpc_mode_status_topic = str(
            self.declare_parameter(
                'payload_mpc_mode_status_topic', 'tejen_mpc/mpc_mode_status'
            ).value
        )
        self.payload_mpc_mode_status_publisher = self.create_publisher(
            String, self.payload_mpc_mode_status_topic, 10
        )

        self.traj, trajectory_name = zeng_simplified_payload_trajectory(DT,
            # np.array([
            #     [ 1.000,  0.000, 1.000],
            #     [ 0.707,  0.707, 1.212],
            #     [ 0.000,  1.000, 1.300],
            #     [-0.707,  0.707, 1.212],
            #     [-1.000,  0.000, 1.000],
            #     [-0.707, -0.707, 0.788],
            #     [-0.000, -1.000, 0.700],
            #     [ 0.707, -0.707, 0.788],
            # ])
            np.array([
                [ 0.00,  0.00, 1.20],
                [ 0.55,  0.10, 1.45],
                [ 0.85,  0.55, 1.75],
                [ 0.25,  0.85, 1.55],
                [-0.45,  0.70, 1.25],
                [-0.85,  0.20, 1.65],
                [-0.65, -0.55, 1.85],
                [ 0.00, -0.90, 1.45],
                [ 0.65, -0.60, 1.15],
                [ 0.90, -0.10, 1.60],
                [ 0.40,  0.45, 1.80],
                [ 0.00,  0.00, 1.20],
            ]), total_time = 20.0)
        #self.traj, trajectory_name = xyz_sine_trajectory(DT)
        # self.traj, trajectory_name = zeng_simplified_payload_trajectory(DT,
        #     np.array([
        #         [0.0,  0.0, 1.2],

        #         # outward slalom
        #         [0.8,  0.6, 1.4],
        #         [1.6, -0.6, 1.6],
        #         [2.4,  0.6, 1.3],
        #         [3.2, -0.6, 1.5],
        #         [4.0,  0.6, 1.7],
        #         [4.8, -0.6, 1.4],
        #         [5.6,  0.6, 1.6],

        #         # wide turn / loop section
        #         [6.2,  1.4, 1.5],
        #         [5.6,  2.2, 1.3],
        #         [4.8,  2.8, 1.6],
        #         [4.0,  2.2, 1.8],
        #         [3.2,  1.4, 1.5],

        #         # cross-back section
        #         [2.4,  2.2, 1.4],
        #         [1.6,  2.8, 1.7],
        #         [0.8,  2.2, 1.5],
        #         [0.0,  1.4, 1.3],
        #         [-0.8, 0.6, 1.6],

        #         # reverse sweep
        #         [0.0, -0.2, 1.4],
        #         [0.8, -1.0, 1.7],
        #         [1.6, -1.6, 1.5],
        #         [2.4, -1.0, 1.3],
        #         [3.2, -0.2, 1.6],

        #         # return home with altitude variation
        #         [2.4,  0.8, 1.7],
        #         [1.6,  1.2, 1.4],
        #         [0.8,  0.6, 1.5],
        #         [0.4,  0.2, 1.3],
        #         [0.0,  0.0, 1.2],
        #     ]))
        self.trajectory_visualizer = TrajectoryVisualizer(self, frame_id="map")
        self.trajectory_visualizer.publish_all_visualizations(self.traj,  pose_subsample=15, show_velocity=False,  velocity_scale=0.3, color_by_time=True )

        # pendulum initialisation thing
        self.pendulum_state = np.zeros(4)

        self.timer = self.create_timer(DT, self.control_loop)
        self.step_counter = 0
        self.steps = self.traj.shape[1] - 1

        self.armed = False
        self.takeoff_requested = False
        self.shutdown_requested = False

        # Optional durable arming-state heartbeat. Default-off preserves the
        # historical edge-triggered feedback semantics for existing missions;
        # M2D opts in so a missed state-transition sample cannot permanently
        # leave its planner or commissioning runner believing the controller is
        # disarmed. The timer follows this node's ROS clock (simulation time in
        # M2D, system time on hardware).
        self.arming_state_feedback_period_s = float(
            self.declare_parameter('arming_state_feedback_period_s', 0.0).value
        )
        self.arming_state_feedback_timer = None
        self.takeoff_state_feedback_publisher = None
        if self.arming_state_feedback_period_s > 0.0:
            # M2D uses the same low-rate ROS-time heartbeat for both operator
            # command states. Keeping this behind the existing opt-in period
            # preserves historical missions while making TAKEOFF delivery just
            # as observable as ARM during four-drone commissioning.
            self.takeoff_state_feedback_publisher = self.create_publisher(
                Bool, 'takeoff_state_feedback', 5
            )
            self.arming_state_feedback_timer = self.create_timer(
                self.arming_state_feedback_period_s,
                self.publish_operator_state_feedback,
            )
            self.get_logger().info(
                'Periodic ARM/TAKEOFF state feedback enabled at '
                f'{self.arming_state_feedback_period_s:.3f} s'
            )


        # MPC settings
        self.N = 20
        self.skip_steps = 3
        self.first_solve = True

        # Optional online replanning interface. Disabled by default so existing
        # fixed-trajectory experiments behave exactly as before.
        self.use_external_reference = bool(
            self.declare_parameter('use_external_reference', False).value
        )
        self.external_reference_topic = str(
            self.declare_parameter('external_reference_topic', '/join_planner/reference').value
        )
        self.external_reference_timeout_s = float(
            self.declare_parameter('external_reference_timeout_s', 0.5).value
        )
        # P2.5 M2C-only optimization.  Default-off so ordinary controller
        # behavior still parses and validates every external trajectory in full.
        # M2C never arms; while disarmed it only needs reference-route health,
        # so the M2C launch can skip repeated 61-point Python conversion work.
        self.disarmed_external_reference_fast_path = bool(
            self.declare_parameter('disarmed_external_reference_fast_path', False).value
        )

        # Payload swing is part of the MPC state. Prevent TAKEOFF from starting
        # without a recent finite estimate, and latch any later dropout for
        # diagnostics. The airborne control response is deliberately unchanged
        # here; fleet/controller-authority handling is deferred to M2 integration.
        self.pendulum_state_timeout_s = max(
            0.01,
            float(self.declare_parameter('pendulum_state_timeout_s', 0.25).value),
        )
        self.pendulum_state_age_s = float('nan')
        self.pendulum_state_fault_latched = False
        self.pendulum_state_fault_reason = ''
        self.pendulum_state_fault_latch_count = 0

        self.external_traj = None
        self.last_external_reference_time = None

        # C.1d reference-fault containment. This deliberately does NOT change the
        # normal external-reference/MPC path. If the external reference becomes
        # unusable after TAKEOFF while mocap remains healthy, latch a stationary
        # position reference at the latest measured position and keep solving the
        # existing MPC. The latch is not cleared by resumed planner messages; the
        # normal DISARM/controller restart cycle clears it. This is a controlled
        # stop/hold response, NOT an obstacle-safety certificate.
        self.external_acceleration_feedforward_enabled = bool(
            self.declare_parameter(
                'external_acceleration_feedforward_enabled', True
            ).value
        )
        self.external_reference_state = 'DISABLED' if not self.use_external_reference else 'WAITING'
        self.external_reference_age_s = float('nan')
        self.external_reference_fault_latched = False
        self.external_reference_fault_reason = ''
        self.external_reference_fault_position = None
        self.external_reference_accepted_count = 0
        self.external_reference_rejected_count = 0
        self.external_reference_fault_latch_count = 0

        self.external_reference_subscription = self.create_subscription(
            MultiDOFJointTrajectory,
            self.external_reference_topic,
            self.external_reference_callback,
            1
        )
        if self.use_external_reference:
            self.get_logger().info(
                f'External reference enabled. Listening on {self.external_reference_topic}'
            )
            self.get_logger().info(
                'External acceleration feedforward: '
                f'enabled={self.external_acceleration_feedforward_enabled}'
            )

        # C.1d timing/fault diagnostics only. These do not alter MPC weights,
        # dynamics, horizon, warm-start policy, UKF, or command mapping.
        self.c1d_status_topic = str(
            self.declare_parameter('c1d_status_topic', 'tejen_mpc/c1d_status').value
        )
        self.c1d_status_publisher = self.create_publisher(
            String, self.c1d_status_topic, 10
        )
        self._last_c1d_status_publish_monotonic = 0.0
        self._last_control_loop_start_perf = None
        self.control_loop_period_ms = float('nan')
        self.control_loop_work_time_ms = float('nan')
        self.reference_setup_time_ms = float('nan')
        self.mpc_solve_time_ms = float('nan')
        self.control_loop_work_overrun_count = 0
        self.control_loop_period_samples_ms = deque(maxlen=300)
        self.control_loop_work_samples_ms = deque(maxlen=300)
        self.mpc_solve_samples_ms = deque(maxlen=300)

        # XY steady-bias rejection is deliberately mutually exclusive.  Keep the
        # commissioned integral path intact for rollback/A-B testing while allowing
        # the new low-bandwidth lateral-disturbance path to use the same OCP without
        # ever applying both compensators at once.
        legacy_integral_enabled_param = bool(
            self.declare_parameter('enable_xy_integral_action', True).value
        )
        requested_xy_bias_mode = str(
            self.declare_parameter('xy_bias_mode', '').value
        ).strip()
        if requested_xy_bias_mode:
            self.xy_bias_mode = parse_xy_bias_mode(requested_xy_bias_mode)
        else:
            # Backward compatibility for launch/config files written before the
            # explicit mode parameter existed.
            self.xy_bias_mode = (
                'legacy_integral' if legacy_integral_enabled_param else 'none'
            )
        self.enable_xy_integral_action = self.xy_bias_mode == 'legacy_integral'
        self.enable_lateral_disturbance_observer = self.xy_bias_mode in (
            'lateral_disturbance',
            'lateral_disturbance_shadow',
        )
        self.enable_lateral_disturbance_compensation = (
            self.xy_bias_mode == 'lateral_disturbance'
        )

        self.xy_integral_weight = max(
            0.0,
            float(self.declare_parameter('xy_integral_weight', 15.0).value),
        )
        self.xy_integral_terminal_weight = max(
            0.0,
            float(self.declare_parameter('xy_integral_terminal_weight', 30.0).value),
        )
        self.xy_integral_limit = max(
            0.0,
            float(self.declare_parameter('xy_integral_limit', 0.30).value),
        )
        self.xy_integral_reference_jump_reset_m = max(
            0.0,
            float(
                self.declare_parameter(
                    'xy_integral_reference_jump_reset_m',
                    0.25,
                ).value
            ),
        )
        self.xy_integral_error = np.zeros(2, dtype=float)
        self.xy_integral_last_reference = None
        self.xy_integral_last_tracking_error = np.zeros(2, dtype=float)
        self.xy_integral_saturated = np.zeros(2, dtype=bool)

        self.lateral_disturbance_observer_bandwidth_rad_s = max(
            0.0,
            float(
                self.declare_parameter(
                    'lateral_disturbance_observer_bandwidth_rad_s', 0.30
                ).value
            ),
        )
        self.lateral_disturbance_max_abs_mps2 = max(
            0.0,
            float(
                self.declare_parameter(
                    'lateral_disturbance_max_abs_mps2', 2.0
                ).value
            ),
        )
        self.lateral_disturbance_airborne_height_m = max(
            0.0,
            float(
                self.declare_parameter(
                    'lateral_disturbance_airborne_height_m', 0.20
                ).value
            ),
        )
        self.lateral_disturbance_max_dt_s = max(
            DT,
            float(
                self.declare_parameter(
                    'lateral_disturbance_max_dt_s', 0.15
                ).value
            ),
        )
        self.lateral_disturbance_observer = LateralDisturbanceObserver(
            bandwidth_rad_s=self.lateral_disturbance_observer_bandwidth_rad_s,
            max_abs_disturbance_mps2=self.lateral_disturbance_max_abs_mps2,
            max_dt_s=self.lateral_disturbance_max_dt_s,
        )
        self.lateral_disturbance_estimate = np.zeros(2, dtype=float)
        self.lateral_disturbance_innovation = np.zeros(2, dtype=float)
        self.lateral_disturbance_observer_status = (
            'waiting_for_airborne'
            if self.enable_lateral_disturbance_observer
            else 'disabled'
        )
        self.lateral_disturbance_update_count = 0
        self.lateral_disturbance_last_update_time_s = None
        self.lateral_disturbance_initial_z = None
        # Airborne qualification is a one-way latch for each takeoff. The observer
        # must stay valid when M1 deliberately descends back below the qualification
        # height during pickup. It is re-armed only by returning to the pre-takeoff
        # state (armed with no TAKEOFF request) or by starting a fresh controller.
        self.lateral_disturbance_airborne_latched = False

        integral_running_weight = (
            self.xy_integral_weight if self.enable_xy_integral_action else 0.0
        )
        integral_terminal_weight = (
            self.xy_integral_terminal_weight
            if self.enable_xy_integral_action
            else 0.0
        )
        # Empty by default so non-M2C launches retain the historical per-process
        # ACADOS generation behavior.  The M2C controller launch supplies one
        # shared cache directory for all four independent controller processes.
        self.acados_cache_dir = str(
            self.declare_parameter('acados_cache_dir', '').value
        ).strip()
        if self.acados_cache_dir:
            self.get_logger().info(
                f'Payload-MPC ACADOS artifact cache: {self.acados_cache_dir}'
            )

        # self.ocp, self.sim_integrator = generate_ocp_controller()
        self.ocp, self.sim_integrator = generate_payload_ocp_controller(
            xy_integral_weight=integral_running_weight,
            xy_integral_terminal_weight=integral_terminal_weight,
            acados_cache_dir=self.acados_cache_dir or None,
        )
        self.base_payload_mpc_W, self.base_payload_mpc_W_e = base_payload_cost_matrices(
            integral_running_weight,
            integral_terminal_weight,
        )
        # Subscribe only after the solver exists so a fast publisher cannot race
        # ACADOS construction. No callback rewrites the matrices at 30 Hz - mode
        # changes update them once.
        self.payload_mpc_mode_subscription = self.create_subscription(
            String,
            self.payload_mpc_mode_topic,
            self.payload_mpc_mode_callback,
            10,
        )
        self._publish_payload_mpc_mode_status()

        self.get_logger().info(
            'XY bias rejection: '
            f'mode={self.xy_bias_mode}, '
            f'legacy_integral={self.enable_xy_integral_action}, '
            f'disturbance_observer={self.enable_lateral_disturbance_observer}, '
            f'disturbance_compensation={self.enable_lateral_disturbance_compensation}'
        )
        if self.enable_xy_integral_action:
            self.get_logger().info(
                'Legacy XY integral: '
                f'W_i={integral_running_weight:.2f}, '
                f'W_i_terminal={integral_terminal_weight:.2f}, '
                f'limit={self.xy_integral_limit:.3f} m*s, '
                f'reference_jump_reset={self.xy_integral_reference_jump_reset_m:.3f} m'
            )
        elif self.enable_lateral_disturbance_observer:
            self.get_logger().info(
                'Lateral disturbance observer: '
                f'bandwidth={self.lateral_disturbance_observer_bandwidth_rad_s:.3f} rad/s, '
                f'limit={self.lateral_disturbance_max_abs_mps2:.3f} m/s^2, '
                f'airborne_height={self.lateral_disturbance_airborne_height_m:.3f} m'
            )

        # Parameters: [Thrust ratio, drag ratio, angular velocity tau, centre rate, max rate, expo]
        # self.est_params = np.array([30.0, 0.0, 0.12, 100.0, 100.0, 0.5])
        # self.est_params = np.array([44.0, 0.0, 0.12, 100.0, 100.0, 0.5, 0.531])
        # pose-timeout self-disarm as a parameter (multi_drone_control fork): the shared
        # sim runs at ~0.25 real time and a slow solve stalled pose callbacks past 0.25 s
        # of sim time with the mocap flowing (R0620). Default unchanged.
        global POSE_TIMEOUT_THRESHOLD
        POSE_TIMEOUT_THRESHOLD = float(
            self.declare_parameter('pose_timeout_s', POSE_TIMEOUT_THRESHOLD).value)
        # thrust ratio and Betaflight rate curve as parameters (multi_drone_control fork):
        # 22 is the cage value; our linear sim plant (x3 motorConstant 0.62e-6) is ~83.
        self.est_params = np.array([
            float(self.declare_parameter('thrust_ratio', 22.0).value), 0.0, 0.12,
            float(self.declare_parameter('rates_centre_deg', 100.0).value),
            float(self.declare_parameter('rates_max_deg', 100.0).value),
            float(self.declare_parameter('rates_expo', 0.5).value),
            # pivot to magnet tip; his airframe 0.531, our x3_drone3 sim model 0.49
            float(self.declare_parameter('cable_length', 0.531).value)])

        # NEW UKF ACTIVATION THRUST RATIO PARAMETERS
        self.enable_thrust_ratio_feedback = bool(
            self.declare_parameter(
                'enable_thrust_ratio_feedback',
                False,
            ).value
        )

        self.thrust_ratio_feedback_min_updates = max(
            1,
            int(
                self.declare_parameter(
                    'thrust_ratio_feedback_min_updates',
                    15,
                ).value
            ),
        )

        self.thrust_ratio_feedback_max_std = max(
            0.0,
            float(
                self.declare_parameter(
                    'thrust_ratio_feedback_max_std',
                    2.0,
                ).value
            ),
        )

        self.thrust_ratio_feedback_rate_per_s = max(
            0.0,
            float(
                self.declare_parameter(
                    'thrust_ratio_feedback_rate_per_s',
                    1.0,
                ).value
            ),
        )

        self.thrust_ratio_feedback_max_fractional_change = float(
            np.clip(
                self.declare_parameter(
                    'thrust_ratio_feedback_max_fractional_change',
                    0.35,
                ).value,
                0.0,
                0.90,
            )
        )

        self.thrust_ratio_feedback_deadband = max(
            0.0,
            float(
                self.declare_parameter(
                    'thrust_ratio_feedback_deadband',
                    0.20,
                ).value
            ),
        )

        self.initial_mpc_thrust_ratio = float(self.est_params[0])
        self.thrust_ratio_feedback_last_time = None
        self.thrust_ratio_feedback_applied = False
        self.thrust_ratio_feedback_target = self.initial_mpc_thrust_ratio
        self.thrust_ratio_feedback_status = (
            'disabled'
            if not self.enable_thrust_ratio_feedback
            else 'waiting_for_estimator'
        )
        # Selectable thrust-ratio estimators:
        #   full_model_kt_ukf: three kT sigma points propagated through the full
        #       payload model; intended for normal simulation use.
        #   vertical_ukf: lightweight three-state vertical baseline.
        #   full_state_ukf: original supervisor 14-state/29-sigma-point UKF for
        #       high-compute or non-Gazebo runs.
        # Estimation remains shadow-only unless enable_thrust_ratio_feedback is true.
        # Feedback is deliberately restricted to full_model_kt_ukf below.
        self.enable_thrust_ratio_ukf = bool(
            self.declare_parameter('enable_thrust_ratio_ukf', False).value
        )

        self.thrust_ratio_estimator_backend = str(
            self.declare_parameter(
                'thrust_ratio_estimator_backend',
                'full_model_kt_ukf',
            ).value
        ).strip().lower()

        if self.thrust_ratio_estimator_backend == 'full_ukf':
            # Backward compatibility with the earlier backend name.
            self.thrust_ratio_estimator_backend = 'full_state_ukf'

        valid_thrust_ratio_backends = (
            'full_model_kt_ukf',
            'vertical_ukf',
            'full_state_ukf',
        )

        if self.thrust_ratio_estimator_backend not in valid_thrust_ratio_backends:
            self.get_logger().warn(
                f"Unknown thrust-ratio backend "
                f"{self.thrust_ratio_estimator_backend!r}; "
                "using full_model_kt_ukf."
            )
            self.thrust_ratio_estimator_backend = 'full_model_kt_ukf'

        self.thrust_ratio_estimator_rate_hz = max(
            0.1,
            float(self.declare_parameter(
                'thrust_ratio_estimator_rate_hz', 10.0
            ).value),
        )
        self.thrust_ratio_estimator_period_s = 1.0 / self.thrust_ratio_estimator_rate_hz

        self.thrust_ratio_ukf_min = float(
            self.declare_parameter('thrust_ratio_ukf_min', 12.0).value
        )
        # Simulation currently uses about 44, so the old maximum of 35 was too low.
        self.thrust_ratio_ukf_max = float(
            self.declare_parameter('thrust_ratio_ukf_max', 60.0).value
        )
        self.thrust_ratio_ukf_control_delay_steps = max(1, int(
            self.declare_parameter('thrust_ratio_ukf_control_delay_steps', 1).value
        ))
        self.thrust_ratio_ukf_min_height_above_start = float(
            self.declare_parameter('thrust_ratio_ukf_min_height_above_start', 0.20).value
        )
        self.thrust_ratio_ukf_min_throttle = float(
            self.declare_parameter('thrust_ratio_ukf_min_throttle', 0.15).value
        )
        # Used only by full_ukf. vertical_ukf estimates continuously while airborne.
        self.thrust_ratio_ukf_update_duration_s = float(
            self.declare_parameter('thrust_ratio_ukf_update_duration_s', 5.0).value
        )
        self.thrust_ratio_ukf_filtered_alpha = float(
            self.declare_parameter('thrust_ratio_ukf_filtered_alpha', 0.10).value
        )
        self.thrust_ratio_ukf_initial_std = float(
            self.declare_parameter('thrust_ratio_ukf_initial_std', 4.0).value
        )
        self.thrust_ratio_ukf_process_std = float(
            self.declare_parameter('thrust_ratio_ukf_process_std', 0.03).value
        )
        self.thrust_ratio_vertical_process_std_per_sqrt_s = float(
            self.declare_parameter(
                'thrust_ratio_vertical_process_std_per_sqrt_s', 0.20
            ).value
        )
        self.thrust_ratio_vertical_measurement_z_std = float(
            self.declare_parameter(
                'thrust_ratio_vertical_measurement_z_std', 0.01
            ).value
        )
        self.thrust_ratio_vertical_measurement_vz_std = float(
            self.declare_parameter(
                'thrust_ratio_vertical_measurement_vz_std', 0.10
            ).value
        )
        self.thrust_ratio_vertical_min_r33 = float(
            self.declare_parameter('thrust_ratio_vertical_min_r33', 0.70).value
        )
        self.thrust_ratio_vertical_max_dt_s = float(
            self.declare_parameter('thrust_ratio_vertical_max_dt_s', 0.25).value
        )

        # Full-model scalar kT UKF settings. It uses the existing 30 Hz payload
        # simulator for each stored applied-control interval, but only three UKF
        # sigma points instead of the legacy full-state UKF's 29.
        self.thrust_ratio_full_model_process_std_per_sqrt_s = float(
            self.declare_parameter(
                'thrust_ratio_full_model_process_std_per_sqrt_s', 0.20
            ).value
        )
        self.thrust_ratio_full_model_measurement_position_std = float(
            self.declare_parameter(
                'thrust_ratio_full_model_measurement_position_std', 0.01
            ).value
        )
        self.thrust_ratio_full_model_measurement_velocity_std = float(
            self.declare_parameter(
                'thrust_ratio_full_model_measurement_velocity_std', 0.10
            ).value
        )
        self.thrust_ratio_full_model_nis_threshold = float(
            self.declare_parameter(
                'thrust_ratio_full_model_nis_threshold', 22.46
            ).value
        )
        self.thrust_ratio_full_model_max_dt_s = float(
            self.declare_parameter(
                'thrust_ratio_full_model_max_dt_s', 0.25
            ).value
        )
        self.thrust_ratio_full_model_max_substeps = max(
            1,
            int(self.declare_parameter(
                'thrust_ratio_full_model_max_substeps', 6
            ).value),
        )
        self.thrust_ratio_full_model_body_rate_reference = max(
            1e-3,
            float(self.declare_parameter(
                'thrust_ratio_full_model_body_rate_reference', 2.0
            ).value),
        )
        self.thrust_ratio_full_model_swing_reference = max(
            1e-3,
            float(self.declare_parameter(
                'thrust_ratio_full_model_swing_reference', 0.35
            ).value),
        )
        self.thrust_ratio_full_model_max_measurement_scale = max(
            1.0,
            float(self.declare_parameter(
                'thrust_ratio_full_model_max_measurement_scale', 25.0
            ).value),
        )

        self.thrust_ratio_ukf = None
        self.thrust_ratio_vertical_ukf = None
        self.thrust_ratio_full_model_ukf = None
        if self.enable_thrust_ratio_ukf:
            if self.thrust_ratio_estimator_backend == 'full_state_ukf':
                self.thrust_ratio_ukf = ThrustRatioUKF(
                    thrust_ratio_min=self.thrust_ratio_ukf_min,
                    thrust_ratio_max=self.thrust_ratio_ukf_max,
                    initial_thrust_ratio_std=self.thrust_ratio_ukf_initial_std,
                    process_thrust_ratio_std=self.thrust_ratio_ukf_process_std,
                    filtered_estimate_alpha=self.thrust_ratio_ukf_filtered_alpha,
                )
                self.get_logger().warn(
                    'Using full_state_ukf in shadow mode. Its 29 acados sigma-point '
                    'propagations per control cycle may make the 30 Hz controller '
                    'miss deadlines.'
                )
            elif self.thrust_ratio_estimator_backend == 'vertical_ukf':
                self.thrust_ratio_vertical_ukf = VerticalThrustRatioUKF(
                    thrust_ratio_min=self.thrust_ratio_ukf_min,
                    thrust_ratio_max=self.thrust_ratio_ukf_max,
                    initial_thrust_ratio_std=self.thrust_ratio_ukf_initial_std,
                    process_thrust_ratio_std_per_sqrt_s=(
                        self.thrust_ratio_vertical_process_std_per_sqrt_s
                    ),
                    measurement_z_std=self.thrust_ratio_vertical_measurement_z_std,
                    measurement_vz_std=self.thrust_ratio_vertical_measurement_vz_std,
                    filtered_estimate_alpha=self.thrust_ratio_ukf_filtered_alpha,
                )
                self.get_logger().info(
                    f'Using vertical_ukf at {self.thrust_ratio_estimator_rate_hz:.1f} Hz '
                    'in shadow mode.'
                )
            else:
                self.thrust_ratio_full_model_ukf = FullModelThrustRatioUKF(
                    thrust_ratio_min=self.thrust_ratio_ukf_min,
                    thrust_ratio_max=self.thrust_ratio_ukf_max,
                    initial_thrust_ratio_std=self.thrust_ratio_ukf_initial_std,
                    process_thrust_ratio_std_per_sqrt_s=(
                        self.thrust_ratio_full_model_process_std_per_sqrt_s
                    ),
                    measurement_position_std=(
                        self.thrust_ratio_full_model_measurement_position_std
                    ),
                    measurement_velocity_std=(
                        self.thrust_ratio_full_model_measurement_velocity_std
                    ),
                    filtered_estimate_alpha=self.thrust_ratio_ukf_filtered_alpha,
                    nis_threshold=self.thrust_ratio_full_model_nis_threshold,
                )
                feedback_mode = (
                    'feedback-enabled'
                    if self.enable_thrust_ratio_feedback
                    else 'shadow'
                )
                self.get_logger().info(
                    f'Using full_model_kt_ukf at '
                    f'{self.thrust_ratio_estimator_rate_hz:.1f} Hz in '
                    f'{feedback_mode} mode.'
                )

        self.thrust_ratio_ukf_input_history = []
        self.thrust_ratio_ukf_initial_z = None
        self.thrust_ratio_ukf_start_time = None
        self.thrust_ratio_estimator_last_update_time_s = None
        self.thrust_ratio_full_model_start_state = None
        self.thrust_ratio_full_model_start_time_s = None
        self.thrust_ratio_ukf_last_result = self._empty_thrust_ratio_ukf_result(
            'disabled' if not self.enable_thrust_ratio_ukf else 'waiting_for_takeoff'
        )

        # Logging. Keep high-rate rows focused on values that can actually vary.
        # Run-constant configuration is written once to metadata.json below.
        log_headers = [
            'step', 'timestamp', 'u0', 'u1', 'u2', 'u3',
            'control_loop_period_ms', 'control_loop_work_time_ms',
            'reference_setup_time_ms', 'mpc_solve_time_ms',
            'external_reference_age_ms', 'external_reference_state',
            'payload_mpc_mode', 'external_arm_permission',
            'external_reference_fault_latched', 'control_loop_work_overrun_count',
            'pose_x', 'pose_y', 'pose_z', 'pose_qw', 'pose_qx', 'pose_qy', 'pose_qz',
            'pose_vx', 'pose_vy', 'pose_vz', 'pose_wx', 'pose_wy', 'pose_wz',
            'reference_step',
            'ref_x', 'ref_y', 'ref_z', 'ref_qw', 'ref_qx', 'ref_qy', 'ref_qz',
            'ref_vx', 'ref_vy', 'ref_vz', 'ref_ax', 'ref_ay', 'ref_az',
            'phi', 'theta', 'phi_dot', 'theta_dot',
            'payload_swing_error',
            'mpc_thrust_ratio', 'mpc_thrust_ratio_next',
            'thrust_ratio_feedback_applied',
            'thrust_ratio_feedback_status', 'thrust_ratio_feedback_target',
            'ukf_initialized', 'ukf_updated', 'ukf_status',
            'ukf_thrust_ratio_raw', 'ukf_thrust_ratio_filtered',
            'ukf_thrust_ratio_variance', 'ukf_innovation_norm',
            'ukf_normalized_innovation_squared', 'ukf_update_time_s',
            'ukf_update_count',
            'xy_integral_state_x', 'xy_integral_state_y',
            'xy_integral_tracking_error_x', 'xy_integral_tracking_error_y',
            'xy_integral_saturated_x', 'xy_integral_saturated_y',
            'xy_bias_mode',
            'lateral_disturbance_hat_x', 'lateral_disturbance_hat_y',
            'lateral_disturbance_innovation_x', 'lateral_disturbance_innovation_y',
            'lateral_disturbance_observer_status',
            'lateral_disturbance_update_count',
            'lateral_disturbance_applied_x', 'lateral_disturbance_applied_y',
        ]
        logging_run_name = (
            'external_reference' if self.use_external_reference else trajectory_name
        )
        logging_instance = str(
            self.declare_parameter('logging_instance', '').value
        ).strip().strip('/')
        logging_name = (
            LOGGING_NAME if not logging_instance else f'{LOGGING_NAME}/{logging_instance}'
        )
        self.data_logger = DataLogger(logging_name, logging_run_name, log_headers)
        metadata = {
            'logging_name': logging_name,
            'logging_instance': logging_instance,
            'trajectory_source': logging_run_name,
            'legacy_internal_trajectory_name': trajectory_name,
            'use_external_reference': self.use_external_reference,
            'external_acceleration_feedforward_enabled': (
                self.external_acceleration_feedforward_enabled
            ),
            'external_reference_topic': self.external_reference_topic,
            'external_reference_timeout_s': self.external_reference_timeout_s,
            'payload_mpc_mode_topic': self.payload_mpc_mode_topic,
            'initial_payload_mpc_mode': self.current_payload_mpc_mode.value,
            'require_external_arm_permission': self.require_external_arm_permission,
            'external_arm_permission_topic': self.external_arm_permission_topic,
            'control_frequency_hz': FREQUENCY_HZ,
            'mpc_horizon_stages': self.N,
            'reference_skip_steps': self.skip_steps,
            'thrust_ratio_ukf_enabled': self.enable_thrust_ratio_ukf,
            'thrust_ratio_ukf_backend': self.thrust_ratio_estimator_backend,
            'thrust_ratio_ukf_effective_rate_hz': (
                self.thrust_ratio_estimator_rate_hz
                if self.thrust_ratio_estimator_backend in (
                    'vertical_ukf', 'full_model_kt_ukf'
                )
                else FREQUENCY_HZ
            ),
            'thrust_ratio_ukf_control_delay_steps': (
                self.thrust_ratio_ukf_control_delay_steps
            ),
            'thrust_ratio_feedback_enabled': self.enable_thrust_ratio_feedback,
            'xy_bias_mode': self.xy_bias_mode,
            'xy_integral_enabled': self.enable_xy_integral_action,
            'xy_integral_weight': self.xy_integral_weight,
            'xy_integral_terminal_weight': self.xy_integral_terminal_weight,
            'xy_integral_limit': self.xy_integral_limit,
            'xy_integral_reference_jump_reset_m': (
                self.xy_integral_reference_jump_reset_m
            ),
            'lateral_disturbance_observer_enabled': (
                self.enable_lateral_disturbance_observer
            ),
            'lateral_disturbance_compensation_enabled': (
                self.enable_lateral_disturbance_compensation
            ),
            'lateral_disturbance_observer_bandwidth_rad_s': (
                self.lateral_disturbance_observer_bandwidth_rad_s
            ),
            'lateral_disturbance_max_abs_mps2': self.lateral_disturbance_max_abs_mps2,
            'lateral_disturbance_airborne_height_m': (
                self.lateral_disturbance_airborne_height_m
            ),
            'acados_cache_dir': self.acados_cache_dir,
        }
        metadata_path = os.path.join(
            os.path.dirname(self.data_logger.csv_path), 'metadata.json'
        )
        with open(metadata_path, 'w', encoding='utf-8') as metadata_file:
            json.dump(metadata, metadata_file, indent=2, sort_keys=True)
            metadata_file.write('\n')

        self.observed_state_history = []       
        self.control_history = []
        self.estimated_state_history = []

        self.pendulum_state = np.zeros(4)
        self.last_pendulum_update_time = None

        # The arming service is discoverable while this constructor is still
        # creating the acados solver. Publish the initial disarmed state only after
        # initialisation is genuinely complete so supervisors can use it as the
        # controller-ready handshake.
        self.cb.publish_arming_state()


    def publish_operator_state_feedback(self) -> None:
        """Publish durable manual ARM/TAKEOFF state for supervised M2D runs."""
        self.cb.publish_arming_state()
        if self.takeoff_state_feedback_publisher is not None:
            msg = Bool()
            msg.data = bool(self.takeoff_requested)
            self.takeoff_state_feedback_publisher.publish(msg)

    def external_arm_permission_callback(self, msg: Bool):
        self.external_arm_permission = bool(msg.data)
        self.external_arm_permission_received = True

    def can_arm(self):
        """Optional CallbackManager hook for a mission-owned pre-arm interlock."""
        if not self.require_external_arm_permission:
            return True, ""
        if not self.external_arm_permission_received:
            return False, (
                f"waiting for external arm permission on "
                f"{self.external_arm_permission_topic}"
            )
        if not self.external_arm_permission:
            return False, "M2B bootstrap detach has not been verified"
        return True, "external arm permission granted"

    def _publish_payload_mpc_mode_status(self):
        msg = String()
        msg.data = self.current_payload_mpc_mode.value
        self.payload_mpc_mode_status_publisher.publish(msg)

    def payload_mpc_mode_callback(self, msg: String):
        try:
            requested = parse_payload_mpc_mode(msg.data)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            self._publish_payload_mpc_mode_status()
            return
        if requested == self.current_payload_mpc_mode:
            return
        try:
            apply_payload_mpc_mode(
                self.ocp,
                horizon_stages=self.N,
                mode=requested,
                base_running_cost=self.base_payload_mpc_W,
                base_terminal_cost=self.base_payload_mpc_W_e,
            )
        except Exception as exc:
            self.get_logger().error(
                f"Failed to apply payload MPC mode {requested.value}: {exc}"
            )
            self._publish_payload_mpc_mode_status()
            return
        previous = self.current_payload_mpc_mode
        self.current_payload_mpc_mode = requested
        self._publish_payload_mpc_mode_status()
        self.get_logger().info(
            f"Payload MPC mode: {previous.value} -> {requested.value}"
        )

    def _node_now_s(self):
        """Return physical/controller time from the ROS node clock.

        In simulation this follows Gazebo /clock via use_sim_time; on hardware
        the same code naturally uses the normal ROS/system clock.
        """
        return self.get_clock().now().nanoseconds * 1e-9

    def _pendulum_state_fault_reason(self, now_s=None):
        """Return None only when the latest swing estimate is recent and finite."""
        if self.last_pendulum_update_time is None:
            self.pendulum_state_age_s = float('inf')
            return 'missing pendulum state'

        now = self._node_now_s() if now_s is None else float(now_s)
        age = now - float(self.last_pendulum_update_time)
        self.pendulum_state_age_s = age
        if not np.isfinite(age) or age < -0.1:
            return f'invalid pendulum state age {age:.3f} s'
        if age > self.pendulum_state_timeout_s:
            return f'pendulum state stale ({age:.3f} s)'

        state = np.asarray(self.pendulum_state, dtype=float)
        if state.shape != (4,) or not np.all(np.isfinite(state)):
            return 'pendulum state contains NaN/Inf or has invalid shape'
        return None

    def _latch_pendulum_state_fault(self, reason):
        """Latch an airborne swing-state fault without changing control authority."""
        if self.pendulum_state_fault_latched:
            return
        self.pendulum_state_fault_latched = True
        self.pendulum_state_fault_reason = str(reason)
        self.pendulum_state_fault_latch_count += 1
        self.get_logger().error(
            'Critical pendulum-state fault after TAKEOFF: '
            f'{self.pendulum_state_fault_reason}. Fault status is latched until '
            'DISARM/controller reset. This patch does not change airborne control '
            'authority; the higher-level response is deferred to M2 integration.',
            throttle_duration_sec=1.0,
        )

    def _required_external_reference_samples(self):
        return self.N * self.skip_steps + 1

    def _make_stationary_external_reference(self, position):
        """Return the existing 17-row format as an exact stationary 61-sample hold."""
        required_samples = self._required_external_reference_samples()
        p = np.asarray(position, dtype=float).reshape(3)
        if not np.all(np.isfinite(p)):
            raise ValueError(f'Cannot build stationary external reference from {p}.')
        traj = np.zeros((17, required_samples), dtype=float)
        traj[0:3, :] = p.reshape(3, 1)
        traj[3, :] = 1.0  # identity quaternion [qw,qx,qy,qz]
        return traj

    def _latch_external_reference_fault(self, reason):
        if self.external_reference_fault_latched:
            return
        if self.current_pose is None:
            return
        self.external_reference_fault_position = np.asarray(
            self.current_pose[0:3], dtype=float
        ).copy()
        self.external_reference_fault_latched = True
        self.external_reference_fault_reason = str(reason)
        self.external_reference_fault_latch_count += 1
        self.external_reference_state = 'FAULT_LATCHED_HOLD'

        # Do not carry integral trim or a trajectory-specific solver guess into the
        # abrupt fault-containment hold. The measured control state is still kept.
        self.xy_integral_error[:] = 0.0
        self.xy_integral_last_reference = None
        self.xy_integral_last_tracking_error[:] = 0.0
        self.xy_integral_saturated[:] = False
        if hasattr(self, 'lateral_disturbance_observer'):
            self._reset_lateral_disturbance_observer('external_reference_fault')
        self.first_solve = True
        self.get_logger().error(
            'External-reference fault latched after TAKEOFF: '
            f'{reason}. Holding measured position '
            f'{self.external_reference_fault_position}. '
            'Planner messages will be ignored until DISARM/controller restart. '
            'This is controlled-stop/hold fault containment, not an obstacle-safety certificate.'
        )

    def _handle_disarmed_external_reference_fast_path(self, msg: MultiDOFJointTrajectory):
        """Accept M2C disarmed reference health without converting all 61 points.

        This path is launch-gated and default-off.  As soon as the controller is
        armed, it falls through to the existing full parser on the next reference
        message.  M2C itself is ground-only and never arms.
        """
        if not self.disarmed_external_reference_fast_path or self.armed:
            return False

        required_samples = self._required_external_reference_samples()
        if len(msg.points) < required_samples:
            self.external_reference_rejected_count += 1
            self.get_logger().warn(
                f'External reference too short while disarmed: got {len(msg.points)} samples, '
                f'need at least {required_samples}.',
                throttle_duration_sec=1.0,
            )
            return True

        # Probe the first and last samples used by the controller.  This keeps
        # the M2C health check meaningful while avoiding allocation/conversion of
        # the full 17x61 NumPy trajectory on every 30 Hz callback.
        probe_values = []
        for index in (0, required_samples - 1):
            point = msg.points[index]
            if len(point.transforms) == 0:
                self.external_reference_rejected_count += 1
                self.get_logger().warn(
                    'External reference probe point has no transform.',
                    throttle_duration_sec=1.0,
                )
                return True
            transform = point.transforms[0]
            q = transform.rotation
            probe_values.extend((
                transform.translation.x, transform.translation.y, transform.translation.z,
                q.w, q.x, q.y, q.z,
            ))

        if not np.all(np.isfinite(np.asarray(probe_values, dtype=float))):
            self.external_reference_rejected_count += 1
            self.get_logger().warn(
                'External reference probe contains NaN or Inf.',
                throttle_duration_sec=1.0,
            )
            return True

        # Do not retain an old fully-parsed trajectory while using the lightweight
        # route-health path.  If this launch mode were ever armed unexpectedly,
        # the existing selector holds measured position until a subsequent armed
        # callback performs the normal full parse.
        self.external_traj = None
        self.last_external_reference_time = self._node_now_s()
        self.external_reference_accepted_count += 1
        self.external_reference_state = 'DISARMED_LIGHTWEIGHT'
        return True

    def external_reference_callback(self, msg: MultiDOFJointTrajectory):
        """Convert a rolling MultiDOFJointTrajectory into the existing 17-row trajectory format."""
        if self._handle_disarmed_external_reference_fast_path(msg):
            return
        if len(msg.points) == 0:
            self.external_reference_rejected_count += 1
            self.get_logger().warn(
                'Received empty external reference trajectory.',
                throttle_duration_sec=1.0
            )
            return

        traj = np.zeros((17, len(msg.points)), dtype=float)

        for i, point in enumerate(msg.points):
            if len(point.transforms) == 0:
                self.external_reference_rejected_count += 1
                self.get_logger().warn(
                    'External reference point has no transform.',
                    throttle_duration_sec=1.0
                )
                return

            transform = point.transforms[0]
            q = transform.rotation

            traj[0, i] = transform.translation.x
            traj[1, i] = transform.translation.y
            traj[2, i] = transform.translation.z

            # Internal trajectory format is [qw, qx, qy, qz].
            if abs(q.w) + abs(q.x) + abs(q.y) + abs(q.z) < 1e-9:
                traj[3, i] = 1.0
                traj[4, i] = 0.0
                traj[5, i] = 0.0
                traj[6, i] = 0.0
            else:
                traj[3, i] = q.w
                traj[4, i] = q.x
                traj[5, i] = q.y
                traj[6, i] = q.z

            if len(point.velocities) > 0:
                v = point.velocities[0].linear
                traj[7, i] = v.x
                traj[8, i] = v.y
                traj[9, i] = v.z

            if len(point.accelerations) > 0:
                a = point.accelerations[0].linear
                traj[10, i] = a.x
                traj[11, i] = a.y
                traj[12, i] = a.z

            # Rows 13:17 are the nominal input references. Leave them as zeros.

        required_samples = self._required_external_reference_samples()
        if traj.shape[1] < required_samples:
            self.external_reference_rejected_count += 1
            self.get_logger().warn(
                f'External reference too short: got {traj.shape[1]} samples, '
                f'need at least {required_samples}.',
                throttle_duration_sec=1.0
            )
            return
        if not np.all(np.isfinite(traj)):
            self.external_reference_rejected_count += 1
            self.get_logger().warn(
                'External reference contains NaN or Inf.',
                throttle_duration_sec=1.0
            )
            return

        self.external_traj = traj
        self.last_external_reference_time = self._node_now_s()
        self.external_reference_accepted_count += 1

    def get_reference_trajectory(self):
        """
        Select either the fixed trajectory, the newest rolling reference, or the
        C.1d latched controlled-stop/hold reference.

        Normal valid external-reference behavior is unchanged. Before TAKEOFF, an
        unavailable external reference produces a non-latched measured-position
        hold so the existing ARMED/no-TAKEOFF branch can continue solving while
        still commanding zero thrust. After TAKEOFF, the same fault latches until
        the normal DISARM/controller-restart cycle.
        """
        if not self.use_external_reference:
            self.external_reference_state = 'FIXED_INTERNAL'
            self.external_reference_age_s = float('nan')
            return self.traj, self.step_counter, False

        if self.external_reference_fault_latched:
            self.external_reference_state = 'FAULT_LATCHED_HOLD'
            hold = self._make_stationary_external_reference(
                self.external_reference_fault_position
            )
            return hold, 0, True

        invalid_reason = None
        if self.external_traj is None or self.last_external_reference_time is None:
            invalid_reason = 'missing external reference'
            self.external_reference_age_s = float('inf')
        else:
            age = self._node_now_s() - self.last_external_reference_time
            self.external_reference_age_s = age
            if not np.isfinite(age) or age < -0.1:
                invalid_reason = f'invalid external reference age {age:.3f} s'
            elif age > self.external_reference_timeout_s:
                invalid_reason = f'external reference stale ({age:.3f} s)'
            elif self.external_traj.shape[1] < self._required_external_reference_samples():
                invalid_reason = (
                    f'external reference too short ({self.external_traj.shape[1]} samples)'
                )
            elif not np.all(np.isfinite(self.external_traj)):
                invalid_reason = 'external reference contains NaN/Inf'

        if invalid_reason is not None:
            if self.takeoff_requested:
                self._latch_external_reference_fault(invalid_reason)
                if self.external_reference_fault_latched:
                    return self._make_stationary_external_reference(
                        self.external_reference_fault_position
                    ), 0, True
            self.external_reference_state = 'WAITING_GROUND_HOLD'
            self.get_logger().warn(
                f'{invalid_reason}; using non-latched measured hold before TAKEOFF.',
                throttle_duration_sec=1.0
            )
            if self.current_pose is not None:
                return self._make_stationary_external_reference(
                    self.current_pose[0:3]
                ), 0, True
            return None, 0, True

        self.external_reference_state = 'VALID'
        return self.external_traj, 0, True

    @staticmethod
    def _timing_summary(samples):
        if len(samples) == 0:
            return float('nan'), float('nan'), float('nan')
        a = np.asarray(samples, dtype=float)
        return float(np.mean(a)), float(np.percentile(a, 95.0)), float(np.max(a))

    def _publish_c1d_status_if_due(self):
        now = time.monotonic()
        if now - self._last_c1d_status_publish_monotonic < 1.0:
            return
        self._last_c1d_status_publish_monotonic = now
        period = self._timing_summary(self.control_loop_period_samples_ms)
        work = self._timing_summary(self.control_loop_work_samples_ms)
        solve = self._timing_summary(self.mpc_solve_samples_ms)
        age_ms = (
            1000.0 * self.external_reference_age_s
            if np.isfinite(self.external_reference_age_s)
            else float('nan')
        )
        ukf_result = self.thrust_ratio_ukf_last_result
        thrust_ratio_estimate = float(
            ukf_result.get('filtered_thrust_ratio', float('nan'))
        )
        thrust_ratio_variance = float(
            ukf_result.get('thrust_ratio_variance', float('nan'))
        )
        thrust_ratio_std = (
            math.sqrt(thrust_ratio_variance)
            if math.isfinite(thrust_ratio_variance) and thrust_ratio_variance >= 0.0
            else float('nan')
        )
        thrust_ratio_update_count = int(ukf_result.get('update_count', 0))
        msg = String()
        msg.data = (
            'R6.3C.1d MPC INTEGRATION\n'
            f'armed/takeoff: {self.armed} / {self.takeoff_requested}\n'
            f'external_reference_state: {self.external_reference_state} '
            f'age_ms: {age_ms:.3f} accepted/rejected: '
            f'{self.external_reference_accepted_count} / {self.external_reference_rejected_count}\n'
            f'fault_latched: {self.external_reference_fault_latched} '
            f'reason: {self.external_reference_fault_reason or "none"} '
            f'latch_count: {self.external_reference_fault_latch_count}\n'
            f'pendulum_state_age_ms: '
            f'{(1000.0 * self.pendulum_state_age_s) if np.isfinite(self.pendulum_state_age_s) else float("nan"):.3f} '
            f'fault_latched: {self.pendulum_state_fault_latched} '
            f'reason: {self.pendulum_state_fault_reason or "none"} '
            f'latch_count: {self.pendulum_state_fault_latch_count}\n'
            f'thrust_ratio_feedback: enabled={self.enable_thrust_ratio_feedback} '
            f'applied={self.thrust_ratio_feedback_applied} '
            f'status={self.thrust_ratio_feedback_status} '
            f'estimate={thrust_ratio_estimate:.3f} std={thrust_ratio_std:.3f} '
            f'target={self.thrust_ratio_feedback_target:.3f} '
            f'mpc={float(self.est_params[0]):.3f} updates={thrust_ratio_update_count}\n'
            f'xy_bias_mode: {self.xy_bias_mode} '
            f'd_hat=({self.lateral_disturbance_estimate[0]:.3f}, '
            f'{self.lateral_disturbance_estimate[1]:.3f}) m/s^2 '
            f'd_status={self.lateral_disturbance_observer_status} '
            f'd_updates={self.lateral_disturbance_update_count}\n'
            f'control loop period mean/p95/max ms: '
            f'{period[0]:.3f} / {period[1]:.3f} / {period[2]:.3f}\n'
            f'control loop work mean/p95/max ms: '
            f'{work[0]:.3f} / {work[1]:.3f} / {work[2]:.3f}\n'
            f'MPC solve mean/p95/max ms: '
            f'{solve[0]:.3f} / {solve[1]:.3f} / {solve[2]:.3f}\n'
            f'work-time overruns > {1000.0 * DT:.3f} ms: '
            f'{self.control_loop_work_overrun_count}\n'
            'NOTE: FAULT_LATCHED_HOLD is controlled-stop/hold fault containment, '
            'not an obstacle-safety certificate.'
        )
        self.c1d_status_publisher.publish(msg)
        self.get_logger().info(msg.data)


    def _reset_lateral_disturbance_observer(self, status='reset'):
        measured_velocity = None
        if self.current_pose is not None:
            measured_velocity = np.asarray(self.current_pose[7:9], dtype=float)
        self.lateral_disturbance_observer.reset(
            measured_velocity,
            status=str(status),
        )
        self.lateral_disturbance_estimate[:] = 0.0
        self.lateral_disturbance_innovation[:] = 0.0
        self.lateral_disturbance_observer_status = str(status)
        self.lateral_disturbance_update_count = 0
        self.lateral_disturbance_last_update_time_s = None

    def _lateral_disturbance_model_is_valid(self):
        """Return True once airborne qualification has latched for this takeoff.

        The height threshold is only a ground-contact exclusion gate. Once the
        vehicle has risen above it, later mission descent (including M1 pickup)
        must not reset the disturbance observer. Returning to the armed/no-TAKEOFF
        state clears the latch and refreshes the ground-height baseline for the
        next flight.
        """
        if not self.enable_lateral_disturbance_observer:
            return False
        if self.current_pose is None:
            return False

        if not self.armed or not self.takeoff_requested:
            self.lateral_disturbance_airborne_latched = False
            self.lateral_disturbance_initial_z = float(self.current_pose[2])
            return False

        if self.lateral_disturbance_initial_z is None:
            self.lateral_disturbance_initial_z = float(self.current_pose[2])

        if not self.lateral_disturbance_airborne_latched:
            height_above_start = (
                float(self.current_pose[2]) - self.lateral_disturbance_initial_z
            )
            if height_above_start >= self.lateral_disturbance_airborne_height_m - 1e-9:
                self.lateral_disturbance_airborne_latched = True

        return bool(self.lateral_disturbance_airborne_latched)

    def update_lateral_disturbance_observer(self):
        """Update the low-bandwidth XY acceleration-mismatch estimate.

        Ground contact is deliberately excluded.  Once clearly airborne, the
        observer uses measured XY velocity plus the same nominal thrust model as
        the MPC.  It does not depend on planner mission-phase names, so a changed
        loaded equilibrium can be learned slowly without an attachment reset.
        """
        if self.current_pose is None:
            return np.zeros(2, dtype=float)

        measured_velocity = np.asarray(self.current_pose[7:9], dtype=float)
        now_s = self._node_now_s()

        if not self.enable_lateral_disturbance_observer:
            self.lateral_disturbance_estimate[:] = 0.0
            self.lateral_disturbance_innovation[:] = 0.0
            self.lateral_disturbance_observer_status = 'disabled'
            return self.lateral_disturbance_estimate.copy()

        if not self._lateral_disturbance_model_is_valid():
            self.lateral_disturbance_observer.reset(
                measured_velocity,
                status='waiting_for_airborne',
            )
            self.lateral_disturbance_estimate[:] = 0.0
            self.lateral_disturbance_innovation[:] = 0.0
            self.lateral_disturbance_observer_status = 'waiting_for_airborne'
            self.lateral_disturbance_update_count = 0
            self.lateral_disturbance_last_update_time_s = now_s
            return self.lateral_disturbance_estimate.copy()

        if self.lateral_disturbance_last_update_time_s is None:
            dt_s = DT
        else:
            dt_s = now_s - self.lateral_disturbance_last_update_time_s
        self.lateral_disturbance_last_update_time_s = now_s

        hover_throttle = 9.81 / float(self.est_params[0])
        if len(self.control_history) > 0:
            applied_control_state = np.asarray(
                self.control_history[-1][0:4], dtype=float
            )
        else:
            applied_control_state = np.array(
                [0.0, 0.0, hover_throttle, 0.0], dtype=float
            )

        nominal_acceleration_xy = nominal_lateral_acceleration(
            self.current_pose[3:7],
            applied_control_state[2],
            float(self.est_params[0]),
        )
        result = self.lateral_disturbance_observer.update(
            measured_velocity,
            nominal_acceleration_xy,
            dt_s,
        )
        self.lateral_disturbance_estimate = np.asarray(
            result['disturbance_hat'], dtype=float
        ).copy()
        self.lateral_disturbance_innovation = np.asarray(
            result['innovation'], dtype=float
        ).copy()
        self.lateral_disturbance_observer_status = str(result['status'])
        self.lateral_disturbance_update_count = int(result['update_count'])
        return self.lateral_disturbance_estimate.copy()

    def update_xy_integral_state(self, traj_for_mpc, ref_step):
        """Update the measured XY integral-error state used by the augmented MPC.

        The state convention matches acados.py:
            xi_dot = position_reference_xy - measured_position_xy

        A hard clamp prevents long transients from winding the integrator up
        indefinitely. A large stage-0 reference jump also clears the accumulated
        trim so an offset learned for one target is not carried into a distant one.
        """
        reference_xy = np.asarray(
            traj_for_mpc[0:2, ref_step],
            dtype=float,
        ).reshape(2)
        tracking_error = reference_xy - np.asarray(
            self.current_pose[0:2],
            dtype=float,
        )
        self.xy_integral_last_tracking_error = tracking_error.copy()

        if not self.enable_xy_integral_action or not self.takeoff_requested:
            self.xy_integral_error[:] = 0.0
            self.xy_integral_last_reference = reference_xy.copy()
            self.xy_integral_saturated[:] = False
            return tracking_error

        if self.xy_integral_last_reference is not None:
            reference_jump = float(
                np.linalg.norm(reference_xy - self.xy_integral_last_reference)
            )
            if reference_jump > self.xy_integral_reference_jump_reset_m:
                self.get_logger().info(
                    'Resetting XY integral state after reference jump of '
                    f'{reference_jump:.3f} m.',
                    throttle_duration_sec=1.0,
                )
                self.xy_integral_error[:] = 0.0

        proposed = self.xy_integral_error + DT * tracking_error

        if self.xy_integral_limit > 0.0:
            clipped = np.clip(
                proposed,
                -self.xy_integral_limit,
                self.xy_integral_limit,
            )
            self.xy_integral_saturated = np.abs(clipped - proposed) > 1e-12
            self.xy_integral_error = clipped
        else:
            self.xy_integral_saturated[:] = False
            self.xy_integral_error = proposed

        self.xy_integral_last_reference = reference_xy.copy()
        return tracking_error


    @staticmethod
    def _empty_thrust_ratio_ukf_result(status):
        return {
            'status': str(status),
            'initialized': False,
            'updated': False,
            'raw_thrust_ratio': float('nan'),
            'filtered_thrust_ratio': float('nan'),
            'thrust_ratio_variance': float('nan'),
            'innovation_norm': float('nan'),
            'normalized_innovation_squared': float('nan'),
            'update_time_s': 0.0,
            'update_count': 0,
        }
    
    def apply_thrust_ratio_feedback(self, ukf_result) -> None:
        """Safely feed the filtered full-model UKF estimate into the MPC model."""

        self.thrust_ratio_feedback_applied = False
        self.thrust_ratio_feedback_target = float(self.est_params[0])

        if not self.enable_thrust_ratio_feedback:
            self.thrust_ratio_feedback_status = 'disabled'
            return

        # Only the scalar kT UKF has been validated for active MPC feedback.
        if self.thrust_ratio_estimator_backend != 'full_model_kt_ukf':
            self.thrust_ratio_feedback_status = 'unsupported_backend'
            return

        # Once the vehicle is mechanically constrained by the attachment proof/hold,
        # the free-flight thrust model no longer explains all measured forces.  Freeze
        # the learned kT rather than letting ring/tether reaction forces masquerade as
        # a motor-thrust change.  Feedback resumes automatically after a detach/retreat.
        mode_value = getattr(
            self.current_payload_mpc_mode,
            'value',
            str(self.current_payload_mpc_mode),
        ).strip().upper()
        if mode_value in {'ATTACH_PROOF', 'ATTACHED_HOLD'}:
            self.thrust_ratio_feedback_status = 'suspended_for_constrained_mode'
            return

        if not bool(ukf_result.get('initialized', False)):
            self.thrust_ratio_feedback_status = 'waiting_for_initialization'
            return

        # A rejected NIS update, a rate-limited snapshot, or any other non-update
        # must not alter the model. Preserve the UKF status in the feedback log.
        if not bool(ukf_result.get('updated', False)):
            self.thrust_ratio_feedback_status = (
                f"ukf_{ukf_result.get('status', 'not_updated')}"
            )
            return

        # Advance the feedback rate-limit clock on every accepted UKF update,
        # including updates that are still inside the warm-up/uncertainty gates.
        # This prevents the first eligible update from accumulating a large jump.
        now = self._node_now_s()
        if self.thrust_ratio_feedback_last_time is None:
            feedback_dt = self.thrust_ratio_estimator_period_s
        else:
            feedback_dt = max(
                0.0,
                now - self.thrust_ratio_feedback_last_time,
            )
        self.thrust_ratio_feedback_last_time = now
        feedback_dt = min(
            feedback_dt,
            max(
                self.thrust_ratio_estimator_period_s,
                self.thrust_ratio_full_model_max_dt_s,
            ),
        )

        update_count = int(ukf_result.get('update_count', 0))
        if update_count < self.thrust_ratio_feedback_min_updates:
            self.thrust_ratio_feedback_status = 'waiting_for_min_updates'
            return

        estimate = float(
            ukf_result.get(
                'filtered_thrust_ratio',
                float('nan'),
            )
        )
        variance = float(
            ukf_result.get(
                'thrust_ratio_variance',
                float('nan'),
            )
        )

        if not math.isfinite(estimate):
            self.thrust_ratio_feedback_status = 'invalid_estimate'
            return

        if not math.isfinite(variance) or variance < 0.0:
            self.thrust_ratio_feedback_status = 'invalid_variance'
            return

        estimate_std = math.sqrt(variance)
        if estimate_std > self.thrust_ratio_feedback_max_std:
            self.thrust_ratio_feedback_status = 'uncertainty_too_high'
            return

        # Do not let an estimator fault move the model arbitrarily far from its
        # configured starting value. Intersect the relative limit with UKF bounds.
        fractional_limit = self.thrust_ratio_feedback_max_fractional_change
        safe_min = max(
            self.thrust_ratio_ukf_min,
            self.initial_mpc_thrust_ratio * (1.0 - fractional_limit),
        )
        safe_max = min(
            self.thrust_ratio_ukf_max,
            self.initial_mpc_thrust_ratio * (1.0 + fractional_limit),
        )
        if safe_min > safe_max:
            self.thrust_ratio_feedback_status = 'invalid_feedback_bounds'
            return

        target = float(np.clip(estimate, safe_min, safe_max))
        self.thrust_ratio_feedback_target = target
        current = float(self.est_params[0])
        error = target - current

        if abs(error) <= self.thrust_ratio_feedback_deadband:
            self.thrust_ratio_feedback_status = 'within_deadband'
            return

        maximum_step = self.thrust_ratio_feedback_rate_per_s * feedback_dt
        if maximum_step <= 0.0:
            self.thrust_ratio_feedback_status = 'rate_limit_zero'
            return

        applied_step = float(np.clip(error, -maximum_step, maximum_step))
        new_value = float(np.clip(current + applied_step, safe_min, safe_max))
        if abs(new_value - current) <= 1e-12:
            self.thrust_ratio_feedback_status = 'no_change'
            return

        self.est_params[0] = new_value
        self.thrust_ratio_feedback_applied = True
        self.thrust_ratio_feedback_status = 'applied'

        self.get_logger().info(
            'Applied thrust-ratio feedback: '
            f'estimate={estimate:.3f}, '
            f'std={estimate_std:.3f}, '
            f'target={target:.3f}, '
            f'MPC={self.est_params[0]:.3f}',
            throttle_duration_sec=1.0,
        )

    @staticmethod
    def _quaternion_r33(quaternion):
        quaternion = np.asarray(quaternion, dtype=float).reshape(4)
        norm = float(np.linalg.norm(quaternion))
        if norm <= 1e-12:
            return 1.0
        _, qx, qy, _ = quaternion / norm
        return float(1.0 - 2.0 * (qx * qx + qy * qy))

    def reset_thrust_ratio_ukf(self):
        if self.thrust_ratio_ukf is not None:
            self.thrust_ratio_ukf.reset()
        if self.thrust_ratio_vertical_ukf is not None:
            self.thrust_ratio_vertical_ukf.reset()
        if self.thrust_ratio_full_model_ukf is not None:
            self.thrust_ratio_full_model_ukf.reset()
        self.thrust_ratio_ukf_input_history.clear()
        self.thrust_ratio_ukf_start_time = None
        self.thrust_ratio_estimator_last_update_time_s = None
        self.thrust_ratio_full_model_start_state = None
        self.thrust_ratio_full_model_start_time_s = None
        self.est_params[0] = self.initial_mpc_thrust_ratio
        self.thrust_ratio_feedback_last_time = None
        self.thrust_ratio_feedback_applied = False
        self.thrust_ratio_feedback_target = self.initial_mpc_thrust_ratio
        self.thrust_ratio_feedback_status = (
            'reset_to_nominal'
            if self.enable_thrust_ratio_feedback
            else 'disabled'
        )
        self.thrust_ratio_ukf_last_result = self._empty_thrust_ratio_ukf_result(
            'waiting_for_takeoff' if self.enable_thrust_ratio_ukf else 'disabled'
        )
        if self.current_pose is not None:
            self.thrust_ratio_ukf_initial_z = float(self.current_pose[2])

    def append_thrust_ratio_ukf_input(self, control_state, control_rate, payload_state):
        if not self.enable_thrust_ratio_ukf or self.current_pose is None:
            return

        control_state = np.asarray(control_state, dtype=float).copy()
        self.thrust_ratio_ukf_input_history.append({
            'timestamp_time_s': self._node_now_s(),
            'physical_state': np.asarray(
                self.current_pose[:13], dtype=float
            ).copy(),
            'control_state': control_state,
            'control_rate': np.asarray(control_rate, dtype=float).copy(),
            'payload_state': np.asarray(payload_state, dtype=float).copy(),
            'throttle': float(control_state[2]),
            'r33': self._quaternion_r33(self.current_pose[3:7]),
        })

        estimator_window_samples = max(
            1,
            int(math.ceil(FREQUENCY_HZ / self.thrust_ratio_estimator_rate_hz)),
        )
        maximum_history = (
            self.thrust_ratio_ukf_control_delay_steps
            + 3 * estimator_window_samples
            + 5
        )
        if len(self.thrust_ratio_ukf_input_history) > maximum_history:
            del self.thrust_ratio_ukf_input_history[:-maximum_history]

    def propagate_thrust_ratio_sigma_point(self, augmented_state, input_sample):
        augmented_state = np.asarray(augmented_state, dtype=float).reshape(-1)
        if augmented_state.shape != (14,):
            raise ValueError(f'Expected 14 UKF states, got {augmented_state.shape}')

        physical_state = augmented_state[:13].copy()
        quaternion_norm = float(np.linalg.norm(physical_state[3:7]))
        if quaternion_norm <= 1e-12:
            physical_state[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
        else:
            physical_state[3:7] /= quaternion_norm

        control_state = np.asarray(input_sample['control_state'], dtype=float)
        control_rate = np.asarray(input_sample['control_rate'], dtype=float)
        payload_state = np.asarray(input_sample['payload_state'], dtype=float)

        simulator_state = np.concatenate((
            physical_state,
            control_state,
            payload_state,
        ))
        simulator_parameters = np.concatenate((
            np.array([augmented_state[13]], dtype=float),
            np.asarray(self.est_params[1:7], dtype=float),
            np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
        ))

        self.sim_integrator.set('x', simulator_state)
        self.sim_integrator.set('u', control_rate)
        self.sim_integrator.set('p', simulator_parameters)
        status = self.sim_integrator.solve()
        if status != 0:
            raise RuntimeError(f'acados simulator returned status {status}')

        next_state = np.asarray(self.sim_integrator.get('x'), dtype=float).reshape(-1)
        next_physical_state = next_state[:13].copy()
        next_quaternion_norm = float(np.linalg.norm(next_physical_state[3:7]))
        if next_quaternion_norm <= 1e-12:
            next_physical_state[3:7] = physical_state[3:7]
        else:
            next_physical_state[3:7] /= next_quaternion_norm
        if float(np.dot(next_physical_state[3:7], physical_state[3:7])) < 0.0:
            next_physical_state[3:7] *= -1.0

        return np.concatenate((
            next_physical_state,
            np.array([augmented_state[13]], dtype=float),
        ))

    def propagate_full_model_kt_candidate(
        self,
        start_physical_state,
        candidate_thrust_ratio,
        input_samples,
    ):
        """Propagate one scalar kT sigma point through applied control samples."""
        physical_state = np.asarray(
            start_physical_state, dtype=float
        ).reshape(13).copy()
        candidate_thrust_ratio = float(np.clip(
            candidate_thrust_ratio,
            self.thrust_ratio_ukf_min,
            self.thrust_ratio_ukf_max,
        ))

        quaternion_norm = float(np.linalg.norm(physical_state[3:7]))
        if quaternion_norm <= 1e-12:
            physical_state[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
        else:
            physical_state[3:7] /= quaternion_norm

        simulator_parameters = np.concatenate((
            np.array([candidate_thrust_ratio], dtype=float),
            np.asarray(self.est_params[1:7], dtype=float),
            np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
        ))

        for sample in input_samples:
            control_state = np.asarray(
                sample['control_state'], dtype=float
            ).reshape(4)
            control_rate = np.asarray(
                sample['control_rate'], dtype=float
            ).reshape(4)
            payload_state = np.asarray(
                sample['payload_state'], dtype=float
            ).reshape(4)

            simulator_state = np.concatenate((
                physical_state,
                control_state,
                payload_state,
            ))
            self.sim_integrator.set('x', simulator_state)
            self.sim_integrator.set('u', control_rate)
            self.sim_integrator.set('p', simulator_parameters)
            status = self.sim_integrator.solve()
            if status != 0:
                raise RuntimeError(
                    f'acados simulator returned status {status}'
                )

            next_state = np.asarray(
                self.sim_integrator.get('x'), dtype=float
            ).reshape(-1)
            physical_state = next_state[:13].copy()
            next_quaternion_norm = float(
                np.linalg.norm(physical_state[3:7])
            )
            if next_quaternion_norm <= 1e-12:
                physical_state[3:7] = np.array(
                    [1.0, 0.0, 0.0, 0.0], dtype=float
                )
            else:
                physical_state[3:7] /= next_quaternion_norm

        return np.concatenate((
            physical_state[0:3],
            physical_state[7:10],
        ))

    def _resync_full_model_kt_interval(self, now_s):
        if self.current_pose is None:
            self.thrust_ratio_full_model_start_state = None
        else:
            self.thrust_ratio_full_model_start_state = np.asarray(
                self.current_pose[:13], dtype=float
            ).copy()
        self.thrust_ratio_full_model_start_time_s = float(now_s)

    def run_full_model_thrust_ratio_ukf(self):
        estimator = self.thrust_ratio_full_model_ukf
        if not self.enable_thrust_ratio_ukf or estimator is None:
            self.thrust_ratio_ukf_last_result = (
                self._empty_thrust_ratio_ukf_result('disabled')
            )
            return self.thrust_ratio_ukf_last_result

        def snapshot_or_empty(status):
            if estimator.initialized:
                return estimator.snapshot(status)
            return self._empty_thrust_ratio_ukf_result(status)

        if self.current_pose is None:
            self.thrust_ratio_ukf_last_result = snapshot_or_empty(
                'waiting_for_pose'
            )
            return self.thrust_ratio_ukf_last_result

        if self.thrust_ratio_ukf_initial_z is None:
            self.thrust_ratio_ukf_initial_z = float(self.current_pose[2])

        if not self.takeoff_requested:
            self.thrust_ratio_ukf_last_result = snapshot_or_empty(
                'waiting_for_takeoff'
            )
            return self.thrust_ratio_ukf_last_result

        height_above_start = (
            float(self.current_pose[2]) - self.thrust_ratio_ukf_initial_z
        )
        if height_above_start < self.thrust_ratio_ukf_min_height_above_start:
            self.thrust_ratio_ukf_last_result = snapshot_or_empty(
                'waiting_for_height'
            )
            return self.thrust_ratio_ukf_last_result

        now_s = self._node_now_s()
        if not estimator.initialized:
            try:
                result = estimator.initialize(float(self.est_params[0]))
                self._resync_full_model_kt_interval(now_s)
                self.thrust_ratio_ukf_last_result = result
                mode = (
                    'feedback-enabled'
                    if self.enable_thrust_ratio_feedback
                    else 'shadow'
                )
                self.get_logger().info(
                    f'Full-model scalar thrust-ratio UKF initialized in {mode} mode.',
                    throttle_duration_sec=1.0,
                )
            except Exception as exc:
                self.thrust_ratio_ukf_last_result = (
                    self._empty_thrust_ratio_ukf_result(
                        f'initialization_error: {exc}'
                    )
                )
                self.get_logger().error(
                    f'Full-model scalar UKF initialization failed: {exc}',
                    throttle_duration_sec=1.0,
                )
            return self.thrust_ratio_ukf_last_result

        if (
            self.thrust_ratio_full_model_start_state is None
            or self.thrust_ratio_full_model_start_time_s is None
        ):
            self._resync_full_model_kt_interval(now_s)
            self.thrust_ratio_ukf_last_result = estimator.snapshot(
                'resynced_interval'
            )
            return self.thrust_ratio_ukf_last_result

        elapsed = (
            now_s - self.thrust_ratio_full_model_start_time_s
        )
        if elapsed < self.thrust_ratio_estimator_period_s:
            self.thrust_ratio_ukf_last_result = estimator.snapshot('rate_limited')
            return self.thrust_ratio_ukf_last_result

        if elapsed > self.thrust_ratio_full_model_max_dt_s:
            self._resync_full_model_kt_interval(now_s)
            self.thrust_ratio_ukf_last_result = estimator.snapshot(
                'skipped_large_dt'
            )
            return self.thrust_ratio_ukf_last_result

        extra_delay_s = max(
            0, self.thrust_ratio_ukf_control_delay_steps - 1
        ) * DT
        interval_start = (
            self.thrust_ratio_full_model_start_time_s - extra_delay_s
        )
        interval_end = now_s - extra_delay_s
        input_samples = [
            sample for sample in self.thrust_ratio_ukf_input_history
            if interval_start < sample['timestamp_time_s'] <= interval_end
        ]
        if not input_samples:
            self.thrust_ratio_ukf_last_result = estimator.snapshot(
                'waiting_for_delayed_input'
            )
            return self.thrust_ratio_ukf_last_result

        if len(input_samples) > self.thrust_ratio_full_model_max_substeps:
            self._resync_full_model_kt_interval(now_s)
            self.thrust_ratio_ukf_last_result = estimator.snapshot(
                'skipped_too_many_substeps'
            )
            return self.thrust_ratio_ukf_last_result

        mean_throttle = float(np.mean([
            sample['throttle'] for sample in input_samples
        ]))
        if mean_throttle < self.thrust_ratio_ukf_min_throttle:
            self._resync_full_model_kt_interval(now_s)
            self.thrust_ratio_ukf_last_result = estimator.snapshot(
                'waiting_for_throttle'
            )
            return self.thrust_ratio_ukf_last_result

        maximum_body_rate = max(
            float(np.linalg.norm(sample['physical_state'][10:13]))
            for sample in input_samples
        )
        maximum_swing = max(
            float(np.linalg.norm(sample['payload_state'][0:2]))
            for sample in input_samples
        )
        measurement_scale = 1.0
        measurement_scale += (
            maximum_body_rate
            / self.thrust_ratio_full_model_body_rate_reference
        ) ** 2
        measurement_scale += (
            maximum_swing
            / self.thrust_ratio_full_model_swing_reference
        ) ** 2
        measurement_scale = float(np.clip(
            measurement_scale,
            1.0,
            self.thrust_ratio_full_model_max_measurement_scale,
        ))

        start_state = np.asarray(
            self.thrust_ratio_full_model_start_state, dtype=float
        ).copy()
        measurement = np.concatenate((
            np.asarray(self.current_pose[0:3], dtype=float),
            np.asarray(self.current_pose[7:10], dtype=float),
        ))

        try:
            result = estimator.update(
                measurement,
                process_model=lambda candidate_kt: (
                    self.propagate_full_model_kt_candidate(
                        start_state,
                        candidate_kt,
                        input_samples,
                    )
                ),
                dt=elapsed,
                measurement_covariance_scale=measurement_scale,
            )
            self.thrust_ratio_ukf_last_result = result
            self.get_logger().info(
                'Full-model scalar UKF estimate: '
                f"raw={result['raw_thrust_ratio']:.3f}, "
                f"filtered={result['filtered_thrust_ratio']:.3f}, "
                f"variance={result['thrust_ratio_variance']:.4f}, "
                f"substeps={len(input_samples)}, "
                f"R_scale={measurement_scale:.2f}, "
                f"runtime={1e3 * result['update_time_s']:.2f} ms",
                throttle_duration_sec=1.0,
            )
        except Exception as exc:
            self.thrust_ratio_ukf_last_result = estimator.snapshot(
                f'update_error: {exc}'
            )
            self.get_logger().error(
                f'Full-model scalar UKF update failed; MPC kT was unchanged this cycle: {exc}',
                throttle_duration_sec=1.0,
            )
        finally:
            # Each update interval starts from a fresh measured physical state.
            # Only kT and its covariance persist between UKF updates.
            self._resync_full_model_kt_interval(now_s)

        return self.thrust_ratio_ukf_last_result

    def run_vertical_thrust_ratio_ukf(self):
        estimator = self.thrust_ratio_vertical_ukf
        if not self.enable_thrust_ratio_ukf or estimator is None:
            self.thrust_ratio_ukf_last_result = self._empty_thrust_ratio_ukf_result('disabled')
            return self.thrust_ratio_ukf_last_result

        def snapshot_or_empty(status):
            if estimator.initialized:
                return estimator.snapshot(status)
            return self._empty_thrust_ratio_ukf_result(status)

        if self.current_pose is None:
            self.thrust_ratio_ukf_last_result = snapshot_or_empty('waiting_for_pose')
            return self.thrust_ratio_ukf_last_result

        if self.thrust_ratio_ukf_initial_z is None:
            self.thrust_ratio_ukf_initial_z = float(self.current_pose[2])

        if not self.takeoff_requested:
            self.thrust_ratio_ukf_last_result = snapshot_or_empty('waiting_for_takeoff')
            return self.thrust_ratio_ukf_last_result

        height_above_start = float(self.current_pose[2]) - self.thrust_ratio_ukf_initial_z
        if height_above_start < self.thrust_ratio_ukf_min_height_above_start:
            self.thrust_ratio_ukf_last_result = snapshot_or_empty('waiting_for_height')
            return self.thrust_ratio_ukf_last_result

        if len(self.thrust_ratio_ukf_input_history) < self.thrust_ratio_ukf_control_delay_steps:
            self.thrust_ratio_ukf_last_result = snapshot_or_empty('waiting_for_control_history')
            return self.thrust_ratio_ukf_last_result

        now_s = self._node_now_s()
        if not estimator.initialized:
            try:
                result = estimator.initialize(self.current_pose[:13], float(self.est_params[0]))
                self.thrust_ratio_estimator_last_update_time_s = now_s
                self.thrust_ratio_ukf_last_result = result
                self.get_logger().info(
                    'Vertical thrust-ratio UKF initialized in shadow mode.',
                    throttle_duration_sec=1.0,
                )
            except Exception as exc:
                self.thrust_ratio_ukf_last_result = self._empty_thrust_ratio_ukf_result(
                    f'initialization_error: {exc}'
                )
                self.get_logger().error(
                    f'Vertical thrust-ratio UKF initialization failed: {exc}',
                    throttle_duration_sec=1.0,
                )
            return self.thrust_ratio_ukf_last_result

        assert self.thrust_ratio_estimator_last_update_time_s is not None
        elapsed = now_s - self.thrust_ratio_estimator_last_update_time_s
        if elapsed < self.thrust_ratio_estimator_period_s:
            self.thrust_ratio_ukf_last_result = estimator.snapshot('rate_limited')
            return self.thrust_ratio_ukf_last_result

        if elapsed > self.thrust_ratio_vertical_max_dt_s:
            self.thrust_ratio_estimator_last_update_time_s = now_s
            self.thrust_ratio_ukf_last_result = estimator.snapshot('skipped_large_dt')
            return self.thrust_ratio_ukf_last_result

        # control_state already represents the command from the preceding MPC
        # interval. delay_steps=1 therefore requires no additional time shift.
        extra_delay_s = max(0, self.thrust_ratio_ukf_control_delay_steps - 1) * DT
        interval_start = self.thrust_ratio_estimator_last_update_time_s - extra_delay_s
        interval_end = now_s - extra_delay_s
        input_samples = [
            sample for sample in self.thrust_ratio_ukf_input_history
            if interval_start < sample['timestamp_time_s'] <= interval_end
        ]
        if not input_samples:
            self.thrust_ratio_ukf_last_result = estimator.snapshot('waiting_for_delayed_input')
            return self.thrust_ratio_ukf_last_result

        mean_throttle = float(np.mean([sample['throttle'] for sample in input_samples]))
        mean_r33 = float(np.mean([sample['r33'] for sample in input_samples]))
        mean_vertical_thrust_factor = float(np.mean([
            sample['throttle'] * sample['r33'] for sample in input_samples
        ]))

        if mean_throttle < self.thrust_ratio_ukf_min_throttle:
            self.thrust_ratio_ukf_last_result = estimator.snapshot('waiting_for_throttle')
            return self.thrust_ratio_ukf_last_result
        if mean_r33 < self.thrust_ratio_vertical_min_r33:
            self.thrust_ratio_ukf_last_result = estimator.snapshot('waiting_for_attitude')
            return self.thrust_ratio_ukf_last_result

        try:
            result = estimator.update(
                self.current_pose[:13],
                vertical_thrust_factor=mean_vertical_thrust_factor,
                drag_coeff_z=float(self.est_params[1]),
                dt=elapsed,
            )
            self.thrust_ratio_estimator_last_update_time_s = now_s
            self.thrust_ratio_ukf_last_result = result
            self.get_logger().info(
                'Vertical UKF shadow estimate: '
                f"raw={result['raw_thrust_ratio']:.3f}, "
                f"filtered={result['filtered_thrust_ratio']:.3f}, "
                f"variance={result['thrust_ratio_variance']:.4f}, "
                f"runtime={1e3 * result['update_time_s']:.2f} ms",
                throttle_duration_sec=1.0,
            )
        except Exception as exc:
            self.thrust_ratio_estimator_last_update_time_s = now_s
            self.thrust_ratio_ukf_last_result = estimator.snapshot(f'update_error: {exc}')
            self.get_logger().error(
                f'Vertical thrust-ratio UKF update failed without affecting MPC: {exc}',
                throttle_duration_sec=1.0,
            )
        return self.thrust_ratio_ukf_last_result

    def run_thrust_ratio_ukf(self):
        if self.thrust_ratio_estimator_backend == 'vertical_ukf':
            return self.run_vertical_thrust_ratio_ukf()
        if self.thrust_ratio_estimator_backend == 'full_model_kt_ukf':
            return self.run_full_model_thrust_ratio_ukf()
        return self.run_full_thrust_ratio_ukf()

    def run_full_thrust_ratio_ukf(self):
        if not self.enable_thrust_ratio_ukf or self.thrust_ratio_ukf is None:
            self.thrust_ratio_ukf_last_result = self._empty_thrust_ratio_ukf_result('disabled')
            return self.thrust_ratio_ukf_last_result

        if self.current_pose is None:
            self.thrust_ratio_ukf_last_result = self._empty_thrust_ratio_ukf_result('waiting_for_pose')
            return self.thrust_ratio_ukf_last_result

        if self.thrust_ratio_ukf_initial_z is None:
            self.thrust_ratio_ukf_initial_z = float(self.current_pose[2])

        height_above_start = float(self.current_pose[2]) - self.thrust_ratio_ukf_initial_z
        if not self.takeoff_requested:
            self.thrust_ratio_ukf_last_result = self._empty_thrust_ratio_ukf_result('waiting_for_takeoff')
            return self.thrust_ratio_ukf_last_result

        if height_above_start < self.thrust_ratio_ukf_min_height_above_start:
            self.thrust_ratio_ukf_last_result = self._empty_thrust_ratio_ukf_result('waiting_for_height')
            return self.thrust_ratio_ukf_last_result

        if len(self.thrust_ratio_ukf_input_history) < self.thrust_ratio_ukf_control_delay_steps:
            self.thrust_ratio_ukf_last_result = self._empty_thrust_ratio_ukf_result('waiting_for_control_history')
            return self.thrust_ratio_ukf_last_result

        if not self.thrust_ratio_ukf.initialized:
            try:
                result = self.thrust_ratio_ukf.initialize(
                    self.current_pose[:13],
                    float(self.est_params[0]),
                )
                self.thrust_ratio_ukf_start_time = self._node_now_s()
                self.thrust_ratio_ukf_last_result = result
                self.get_logger().info(
                    'Thrust-ratio UKF initialized in shadow mode.',
                    throttle_duration_sec=1.0,
                )
            except Exception as exc:
                result = self._empty_thrust_ratio_ukf_result(f'initialization_error: {exc}')
                self.thrust_ratio_ukf_last_result = result
                self.get_logger().error(
                    f'Thrust-ratio UKF initialization failed: {exc}',
                    throttle_duration_sec=1.0,
                )
            return self.thrust_ratio_ukf_last_result

        if (
            self.thrust_ratio_ukf_update_duration_s > 0.0
            and self.thrust_ratio_ukf_start_time is not None
            and (self._node_now_s() - self.thrust_ratio_ukf_start_time)
                >= self.thrust_ratio_ukf_update_duration_s
        ):
            self.thrust_ratio_ukf_last_result = self.thrust_ratio_ukf.snapshot('frozen_after_duration')
            return self.thrust_ratio_ukf_last_result

        input_sample = self.thrust_ratio_ukf_input_history[
            -self.thrust_ratio_ukf_control_delay_steps
        ]
        if float(input_sample['control_state'][2]) < self.thrust_ratio_ukf_min_throttle:
            self.thrust_ratio_ukf_last_result = self.thrust_ratio_ukf.snapshot('waiting_for_throttle')
            return self.thrust_ratio_ukf_last_result

        try:
            result = self.thrust_ratio_ukf.update(
                self.current_pose[:13],
                lambda sigma_point: self.propagate_thrust_ratio_sigma_point(
                    sigma_point,
                    input_sample,
                ),
            )
            self.thrust_ratio_ukf_last_result = result
            self.get_logger().info(
                'UKF shadow estimate: '
                f"raw={result['raw_thrust_ratio']:.3f}, "
                f"filtered={result['filtered_thrust_ratio']:.3f}, "
                f"variance={result['thrust_ratio_variance']:.4f}",
                throttle_duration_sec=1.0,
            )
        except Exception as exc:
            result = self.thrust_ratio_ukf.snapshot(f'update_error: {exc}')
            self.thrust_ratio_ukf_last_result = result
            self.get_logger().error(
                f'Thrust-ratio UKF update failed without affecting MPC: {exc}',
                throttle_duration_sec=1.0,
            )
        return self.thrust_ratio_ukf_last_result


    def control_loop(self):
        control_loop_started = time.perf_counter()
        if self._last_control_loop_start_perf is not None:
            self.control_loop_period_ms = 1000.0 * (
                control_loop_started - self._last_control_loop_start_perf
            )
            if np.isfinite(self.control_loop_period_ms):
                self.control_loop_period_samples_ms.append(self.control_loop_period_ms)
        self._last_control_loop_start_perf = control_loop_started
        self.reference_setup_time_ms = float('nan')
        self.mpc_solve_time_ms = float('nan')

        if self.shutdown_requested:
            self.cb.request_shutdown()
            return

        # Pose freshness follows the ROS node clock. In simulation this is Gazebo
        # /clock; on hardware it remains ordinary ROS/system time.
        if self.armed and (self._node_now_s() - self.last_pose_update_time) > POSE_TIMEOUT_THRESHOLD:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0, channel_5=0.0, channel_6=0.0, channel_7=0.0, channel_8=0.0, channel_9=0.0, channel_10=0.0)
            self.cb.disarm(msg)
            return 
        
        if self.current_pose is None:
            self.get_logger().warn("Waiting for /motion_capture_state...")
            return

        if self.thrust_ratio_ukf_initial_z is None:
            self.thrust_ratio_ukf_initial_z = float(self.current_pose[2])

        if self.armed and self.current_pose is not None:
            if self.takeoff_requested:
                pendulum_fault_reason = self._pendulum_state_fault_reason()
                if pendulum_fault_reason is not None:
                    # Ground-side safety gate: do not execute the first TAKEOFF
                    # control step without a trustworthy payload state.
                    if self.step_counter == 0 and len(self.control_history) == 0:
                        self.takeoff_requested = False
                        self.get_logger().error(
                            f'{pendulum_fault_reason}; TAKEOFF rejected until '
                            '/pendulum_swing_state is fresh.',
                            throttle_duration_sec=1.0,
                        )
                        return
                    # Once airborne, preserve current controller behavior but make
                    # the unresolved critical-input fault explicit and persistent.
                    self._latch_pendulum_state_fault(pendulum_fault_reason)

            reference_setup_started = time.perf_counter()
            traj_for_mpc, ref_step, using_external_reference = self.get_reference_trajectory()
            if traj_for_mpc is None:
                return

            if (not using_external_reference) and (self.step_counter + self.N * self.skip_steps > self.steps):
                msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0, channel_5=0.0, channel_6=0.0, channel_7=0.0, channel_8=0.0, channel_9=0.0, channel_10=0.0)
                self.cb.disarm(msg)
                self.cb.request_shutdown()
                return
            
            # self.get_logger().info(
            #     f"pose={self.current_pose[0:3]}, "
            #     f"vel={self.current_pose[7:10]}, "
            #     f"ref={self.traj[0:3, self.step_counter]}"
            # )
            # Capture the value actually supplied to this MPC solve. Active UKF
            # feedback is applied later in the loop and therefore affects the next solve.
            mpc_thrust_ratio_used = float(self.est_params[0])

            xy_integral_tracking_error = self.update_xy_integral_state(
                traj_for_mpc,
                ref_step,
            )
            self.update_lateral_disturbance_observer()
            lateral_disturbance_for_mpc = (
                self.lateral_disturbance_estimate.copy()
                if self.enable_lateral_disturbance_compensation
                else np.zeros(2, dtype=float)
            )

            # set_trajectory_reference_aligned(self.ocp, self.traj, self.N, self.step_counter, self.skip_steps, self.est_params)
            set_payload_trajectory_reference_aligned(
                self.ocp,
                traj_for_mpc,
                self.N,
                ref_step,
                self.skip_steps,
                self.est_params,
                use_acceleration_feedforward=(
                    using_external_reference
                    and self.external_acceleration_feedforward_enabled
                ),
                lateral_disturbance_xy=lateral_disturbance_for_mpc,
            )
            self.reference_setup_time_ms = 1000.0 * (
                time.perf_counter() - reference_setup_started
            )
            # estimated_state = copy.deepcopy(self.current_pose[:13])

            # estimated_state_payload = np.concatenate((
            #     estimated_state,
            #     np.array(self.control_history[-1][0:4]) if len(self.control_history) > 0 else np.array([0.0, 0.0, hover_throttle, 0.0]),
            #     self.pendulum_state
            # ))

            estimated_state = copy.deepcopy(self.current_pose[:13])

            hover_throttle = 9.81 / self.est_params[0]

            if (len(self.control_history) == 0) or (not self.takeoff_requested):
                control_state = np.array([0.0, 0.0, hover_throttle, 0.0])
            else:
                control_state = np.array(self.control_history[-1][0:4])

            payload_state = np.array(self.pendulum_state, dtype=float)

            estimated_state_with_control = np.concatenate((
                estimated_state,          # 13
                control_state,            # 4
                payload_state,            # 4
                self.xy_integral_error    # 2
            ))

            assert len(estimated_state_with_control) == 23

            # if len(self.control_history) == 0:
            #     self.get_logger().warn("Control history is empty - cannot set state bounds accurately")
            #     estimated_state_with_control = np.concatenate((estimated_state, np.array([0.0, 0.0, 0.0, 0.0]))) 
            # if len(self.control_history) == 0:
            #     self.get_logger().warn("Control history is empty - using hover throttle initialisation")
            #     hover_throttle = 9.81 / self.est_params[0]
            #     estimated_state_with_control = np.concatenate((
            #         estimated_state,
            #         np.array([0.0, 0.0, hover_throttle, 0.0])
            #     ))
            # else:
            #     estimated_state_with_control = np.concatenate((estimated_state, np.array(self.control_history[-1][0:4]))) 

 
            # Fix the MPC initial state to the latest estimated state.
            initial_state = np.asarray(estimated_state_with_control, dtype=float)

            if initial_state.shape != (23,):
                raise ValueError(
                    f"Expected 23 initial states, got shape {initial_state.shape}"
                )

            if not np.all(np.isfinite(initial_state)):
                raise ValueError(f"Initial state contains NaN or Inf: {initial_state}")

            self.ocp.set(0, "lbx", initial_state)
            self.ocp.set(0, "ubx", initial_state)

            ### MPC WARM START
            if self.first_solve:
                set_initial_guess(self.ocp, self.N)
                self.first_solve = False
            else:
                warm_start_from_previous_solution(self.ocp, self.N)

            ### SOLVE OCP
            mpc_solve_started = time.perf_counter()
            status = self.ocp.solve()
            self.mpc_solve_time_ms = 1000.0 * (
                time.perf_counter() - mpc_solve_started
            )
            if np.isfinite(self.mpc_solve_time_ms):
                self.mpc_solve_samples_ms.append(self.mpc_solve_time_ms)
            if status != 0:
                raise Exception(f'acados returned status {status}.')
            x = self.ocp.get(1, "x")
            # u = x[-4:]
            # u = x[-4:]
            u = x[13:17]
            # u[0] = 0.0
            # u[1] = 0.0
            # u[3] = 0.0
            u_rate = self.ocp.get(0, "u")

            ### SEND COMMANDS
            # Only execute trajectory if takeoff has been requested
            if self.takeoff_requested:
                msg = ELRSCommand(armed=True, channel_0=round(u[0], 3), channel_1=round(u[1], 3), channel_2=round((u[2]*2)-1, 3), channel_3=round(u[3], 3), channel_5=1.0, channel_6=1.0, channel_7=1.0, channel_8=1.0, channel_9=1.0, channel_10=1.0)
            else:
                # Stay armed but don't send thrust commands until takeoff
                msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0, channel_5=1.0, channel_6=1.0, channel_7=1.0, channel_8=1.0, channel_9=1.0, channel_10=1.0)
            
            self.cb.cmd_publisher_.publish(msg)
            
            # Extract MPC trajectory for visualization
            mpc_trajectory = np.zeros((13, self.N))
            for i in range(self.N):
                x_i = self.ocp.get(i, "x")
                mpc_trajectory[:, i] = x_i[:13]
            
            # Replace the first state with the current measured pose to eliminate offset
            mpc_trajectory[:, 0] = self.current_pose[:13]
            
            # Publish MPC plan visualization
            self.trajectory_visualizer.publish_mpc_plan(mpc_trajectory)
            self.trajectory_visualizer.publish_transform_frame(self.current_pose, "drone_mocap")

            payload_swing_error = np.sqrt(
                self.pendulum_state[0]**2 + self.pendulum_state[1]**2
            )

            # control_state is the command that was applied over the interval
            # ending at the current mocap measurement. For full_ukf, the matching
            # applied control-rate command is the previous history entry.
            if self.takeoff_requested:
                if len(self.control_history) > 0:
                    applied_control_rate = np.asarray(
                        self.control_history[-1][4:8], dtype=float
                    )
                else:
                    applied_control_rate = np.zeros(4, dtype=float)
                self.append_thrust_ratio_ukf_input(
                    control_state,
                    applied_control_rate,
                    payload_state,
                )

            ukf_result = self.run_thrust_ratio_ukf()

            self.apply_thrust_ratio_feedback(ukf_result)

            self.control_loop_work_time_ms = 1000.0 * (
                time.perf_counter() - control_loop_started
            )
            if np.isfinite(self.control_loop_work_time_ms):
                self.control_loop_work_samples_ms.append(self.control_loop_work_time_ms)
                if self.control_loop_work_time_ms > 1000.0 * DT:
                    self.control_loop_work_overrun_count += 1
            external_reference_age_ms = (
                1000.0 * self.external_reference_age_s
                if np.isfinite(self.external_reference_age_s)
                else float('nan')
            )
            
            reference_sample = traj_for_mpc[:, ref_step]
            log_row = [
                self.step_counter,
                time.time(),
                float(u[0]), float(u[1]), float(u[2]), float(u[3]),
                float(self.control_loop_period_ms),
                float(self.control_loop_work_time_ms),
                float(self.reference_setup_time_ms),
                float(self.mpc_solve_time_ms),
                float(external_reference_age_ms),
                str(self.external_reference_state),
                self.current_payload_mpc_mode.value,
                bool(self.external_arm_permission),
                bool(self.external_reference_fault_latched),
                int(self.control_loop_work_overrun_count),
                float(self.current_pose[0]), float(self.current_pose[1]), float(self.current_pose[2]),
                float(self.current_pose[3]), float(self.current_pose[4]), float(self.current_pose[5]), float(self.current_pose[6]),
                float(self.current_pose[7]), float(self.current_pose[8]), float(self.current_pose[9]),
                float(self.current_pose[10]), float(self.current_pose[11]), float(self.current_pose[12]),
                int(ref_step),
                float(reference_sample[0]), float(reference_sample[1]), float(reference_sample[2]),
                float(reference_sample[3]), float(reference_sample[4]), float(reference_sample[5]), float(reference_sample[6]),
                float(reference_sample[7]), float(reference_sample[8]), float(reference_sample[9]),
                float(reference_sample[10]), float(reference_sample[11]), float(reference_sample[12]),
                float(self.pendulum_state[0]), float(self.pendulum_state[1]),
                float(self.pendulum_state[2]), float(self.pendulum_state[3]),
                float(payload_swing_error),
                float(mpc_thrust_ratio_used), float(self.est_params[0]),
                bool(self.thrust_ratio_feedback_applied),
                str(self.thrust_ratio_feedback_status),
                float(self.thrust_ratio_feedback_target),
                bool(ukf_result['initialized']),
                bool(ukf_result['updated']),
                str(ukf_result['status']),
                float(ukf_result['raw_thrust_ratio']),
                float(ukf_result['filtered_thrust_ratio']),
                float(ukf_result['thrust_ratio_variance']),
                float(ukf_result['innovation_norm']),
                float(ukf_result['normalized_innovation_squared']),
                float(ukf_result['update_time_s']),
                int(ukf_result['update_count']),
                float(self.xy_integral_error[0]),
                float(self.xy_integral_error[1]),
                float(xy_integral_tracking_error[0]),
                float(xy_integral_tracking_error[1]),
                bool(self.xy_integral_saturated[0]),
                bool(self.xy_integral_saturated[1]),
                str(self.xy_bias_mode),
                float(self.lateral_disturbance_estimate[0]),
                float(self.lateral_disturbance_estimate[1]),
                float(self.lateral_disturbance_innovation[0]),
                float(self.lateral_disturbance_innovation[1]),
                str(self.lateral_disturbance_observer_status),
                int(self.lateral_disturbance_update_count),
                float(lateral_disturbance_for_mpc[0]),
                float(lateral_disturbance_for_mpc[1]),
            ]
            self.data_logger.append_row(log_row)


            # Publish actual path visualization (dotted red line)
            self.trajectory_visualizer.publish_actual_path(self.current_pose)
            
            # Only increment step counter if takeoff was requested
            if self.takeoff_requested:
                self.step_counter += 1
                self.control_history.append( np.concatenate( (u, u_rate) ).tolist() )

            self.observed_state_history.append( self.current_pose[:13].tolist() )
            self.estimated_state_history.append( estimated_state[:13].tolist() )
            self._publish_c1d_status_if_due()

        else:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0, channel_5=0.0, channel_6=0.0, channel_7=0.0, channel_8=0.0, channel_9=0.0, channel_10=0.0)
            self.cb.cmd_publisher_.publish(msg)
            self.step_counter = 0
            self.xy_integral_error[:] = 0.0
            self.xy_integral_last_reference = None
            self.xy_integral_last_tracking_error[:] = 0.0
            self.xy_integral_saturated[:] = False
            self.lateral_disturbance_initial_z = float(self.current_pose[2])
            self._reset_lateral_disturbance_observer('disarmed')
            self.external_reference_fault_latched = False
            self.external_reference_fault_reason = ''
            self.external_reference_fault_position = None
            self.pendulum_state_fault_latched = False
            self.pendulum_state_fault_reason = ''
            if self.use_external_reference:
                self.external_reference_state = 'WAITING'
            self.thrust_ratio_ukf_initial_z = float(self.current_pose[2])
            full_ukf_initialized = (
                self.thrust_ratio_ukf is not None
                and self.thrust_ratio_ukf.initialized
            )
            vertical_ukf_initialized = (
                self.thrust_ratio_vertical_ukf is not None
                and self.thrust_ratio_vertical_ukf.initialized
            )
            full_model_ukf_initialized = (
                self.thrust_ratio_full_model_ukf is not None
                and self.thrust_ratio_full_model_ukf.initialized
            )
            if (
                full_ukf_initialized
                or vertical_ukf_initialized
                or full_model_ukf_initialized
                or self.thrust_ratio_ukf_input_history
            ):
                self.reset_thrust_ratio_ukf()
            self.control_loop_work_time_ms = 1000.0 * (
                time.perf_counter() - control_loop_started
            )
            if np.isfinite(self.control_loop_work_time_ms):
                self.control_loop_work_samples_ms.append(self.control_loop_work_time_ms)
                if self.control_loop_work_time_ms > 1000.0 * DT:
                    self.control_loop_work_overrun_count += 1
            self._publish_c1d_status_if_due()




    def signal_handler(self, sig, frame):
        print("Interrupt received, shutting down...")
        self.on_close()
        sys.exit(0)

    def on_close(self):
        if getattr(self, 'on_close_called', False):
            return
        self.on_close_called = True
        msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0, channel_5=0.0, channel_6=0.0, channel_7=0.0, channel_8=0.0, channel_9=0.0, channel_10=0.0)
        self.cb.cmd_publisher_.publish(msg)
        self.data_logger.close()


def main(args=None): 
    rclpy.init(args=args)
    controller = Controller()
    signal.signal(signal.SIGINT, controller.signal_handler)
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
    except Exception as e:
        print(f"Exception occurred: {e}")
    finally:
        controller.on_close()
        # Final cleanup
        try:
            controller.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
