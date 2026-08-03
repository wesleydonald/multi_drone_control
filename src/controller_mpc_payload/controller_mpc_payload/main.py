import rclpy
import signal
import sys
import numpy as np
import copy
import math
import os
import threading
from rclpy.node import Node
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import time
# from .acados import generate_ocp_controller, set_initial_guess, warm_start_from_previous_solution, set_trajectory_reference_aligned, update_ocp_parameters
from .acados import (
    generate_payload_ocp_controller,
    set_initial_guess,
    warm_start_from_previous_solution,
    set_payload_trajectory_reference_aligned,
    update_payload_ocp_parameters
)
from .trajectories import zeng_simplified_payload_trajectory, hover_trajectory, z_sin_trajectory, xyz_sine_trajectory, circle_trajectory, power_loop_trajectory, figure8_zsine_trajectory, fast_xyz_sine_trajectory
from utility_objects.visualization import TrajectoryVisualizer
from utility_objects.data_logger import DataLogger
from utility_objects.callback_manager import CallbackManager
from interfaces.msg import MotionCaptureState, ELRSCommand, Telemetry
from trajectory_msgs.msg import MultiDOFJointTrajectory
from scipy.linalg import cholesky
from .thrust_ratio_ukf import ThrustRatioUKF
from .thrust_ratio_vertical_ukf import VerticalThrustRatioUKF
from .thrust_ratio_full_model_ukf import FullModelThrustRatioUKF
# from vector_map import VectorMap


POSE_TIMEOUT_THRESHOLD = 0.25  # seconds
USE_MOTION_CAPTURE = True  # Set to False to use ORB-SLAM data instead
FREQUENCY_HZ = 30.0
DT = 1.0 / FREQUENCY_HZ

LOGGING_NAME = 'controller_mpc_payload'
    
class Controller(Node):
    def __init__(self):
        print('[approach_mpc] __init__ start (before acados codegen)', flush=True)
        super().__init__('controller')

        self.current_pose = None
        self.last_pose_update_time = None
        self.pendulum_state = np.zeros(4)
        self.last_pendulum_update_time = None

        # General Settings. drone_id selects which /drone_{id}/* namespace the state input
        # (motion_capture_state) and command output (ELRSCommand) live on -- set it to the
        # approach drone's id when this controller flies one member of a larger fleet.
        drone_id = int(self.declare_parameter('drone_id', 0).value)
        self.cb = CallbackManager(self, drone_id=drone_id)
        # DEBUG: print the RESOLVED topic names (after launch remaps) so it is obvious what
        # this node listens to for ARM and pose, and where it emits ELRSCommand. If
        # cmd_in is not the topic your ARM button publishes to, the remap did not take.
        self.get_logger().info(
            f"[approach] ARM/cmd_in='{self.cb.command_subscription_.topic_name}'  "
            f"pose_in='{self.cb.pose_subscription_.topic_name}'  "
            f"elrs_out='{self.cb.cmd_publisher_.topic_name}'  (armed starts False)")


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
        self.external_traj = None
        self.last_external_reference_time = None
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

        # self.ocp, self.sim_integrator = generate_ocp_controller()
        self.ocp, self.sim_integrator = generate_payload_ocp_controller()

        # Parameters: [Thrust ratio, drag ratio, angular velocity tau, centre rate, max rate, expo]
        # Thrust ratio is the drone's thrust-to-hover model gain: hover_throttle = 9.81/TR. It
        # MUST match the plant's actual thrust. The sim motorConstant now matches the real
        # airframe, so 24.0 is right for both (it was 45 when the sim was over-powered, and 22
        # before that -- too low a value makes the MPC command ~2x the throttle it needs and the
        # drone rockets up until feedback pulls it back). Launch-tunable via mpc_thrust_ratio.
        # This is also the seed the thrust-ratio UKF initializes from, and the centre of the
        # +/-thrust_ratio_feedback_max_fractional_change band the feedback is allowed to move in.
        mpc_thrust_ratio = float(self.declare_parameter('mpc_thrust_ratio', 24.0).value)
        self.est_params = np.array([mpc_thrust_ratio, 0.0, 0.12, 100.0, 100.0, 0.5, 0.531])

        # NEW UKF ACTIVATION THRUST RATIO PARAMETERS
        # When true the filtered kT estimate is slewed into est_params[0] (and therefore into the
        # OCP parameters every cycle). False = shadow mode: the UKF still runs and logs, but the
        # MPC keeps flying on the fixed mpc_thrust_ratio.
        self.enable_thrust_ratio_feedback = bool(
            self.declare_parameter(
                'enable_thrust_ratio_feedback',
                True,
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
            self.declare_parameter('enable_thrust_ratio_ukf', True).value
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
        self.thrust_ratio_estimator_last_update_monotonic = None
        self.thrust_ratio_full_model_start_state = None
        self.thrust_ratio_full_model_start_monotonic = None
        self.thrust_ratio_ukf_last_result = self._empty_thrust_ratio_ukf_result(
            'disabled' if not self.enable_thrust_ratio_ukf else 'waiting_for_takeoff'
        )

        # Logging
        log_headers = [
            'step', 'timestamp', 'u0', 'u1', 'u2', 'u3',
            'pose_x', 'pose_y', 'pose_z', 'pose_qw', 'pose_qx', 'pose_qy', 'pose_qz',
            'phi', 'theta', 'phi_dot', 'theta_dot',
            'payload_swing_error',
            'mpc_thrust_ratio',
            'mpc_thrust_ratio_next',
            'thrust_ratio_feedback_enabled',
            'thrust_ratio_feedback_applied',
            'thrust_ratio_feedback_status',
            'thrust_ratio_feedback_target',
            'ukf_enabled', 'ukf_backend', 'ukf_effective_rate_hz',
            'ukf_initialized', 'ukf_updated', 'ukf_status',
            'ukf_thrust_ratio_raw', 'ukf_thrust_ratio_filtered',
            'ukf_thrust_ratio_variance', 'ukf_innovation_norm',
            'ukf_normalized_innovation_squared', 'ukf_update_time_s',
            'ukf_update_count', 'ukf_control_delay_steps'
        ]
        self.data_logger = DataLogger(LOGGING_NAME, trajectory_name, log_headers)

        self.observed_state_history = []       
        self.control_history = []
        self.estimated_state_history = []

        self.pendulum_state = np.zeros(4)
        self.last_pendulum_update_time = None


    def external_reference_callback(self, msg: MultiDOFJointTrajectory):
        """Convert a rolling MultiDOFJointTrajectory into the existing 17-row trajectory format."""
        if len(msg.points) == 0:
            self.get_logger().warn('Received empty external reference trajectory.', throttle_duration_sec=1.0)
            return

        traj = np.zeros((17, len(msg.points)), dtype=float)

        for i, point in enumerate(msg.points):
            if len(point.transforms) == 0:
                self.get_logger().warn('External reference point has no transform.', throttle_duration_sec=1.0)
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

        required_samples = self.N * self.skip_steps + 1
        if traj.shape[1] <= required_samples:
            self.get_logger().warn(
                f'External reference too short: got {traj.shape[1]} samples, '
                f'need at least {required_samples + 1}.',
                throttle_duration_sec=1.0
            )
            return

        self.external_traj = traj
        self.last_external_reference_time = time.time()

    def get_reference_trajectory(self):
        """
        Select either the fixed precomputed trajectory or the newest rolling online reference.

        Returns:
            traj_for_mpc: np.ndarray or None
            ref_step: int
            using_external_reference: bool
        """
        if not self.use_external_reference:
            return self.traj, self.step_counter, False

        if self.external_traj is None or self.last_external_reference_time is None:
            self.get_logger().warn(
                f'Waiting for external reference on {self.external_reference_topic}...',
                throttle_duration_sec=1.0
            )
            return None, 0, True

        age = time.time() - self.last_external_reference_time
        if age > self.external_reference_timeout_s:
            self.get_logger().warn(
                f'External reference is stale: {age:.3f} s old.',
                throttle_duration_sec=1.0
            )
            return None, 0, True

        required_samples = self.N * self.skip_steps + 1
        if self.external_traj.shape[1] <= required_samples:
            self.get_logger().warn(
                f'External reference has {self.external_traj.shape[1]} samples, '
                f'but the MPC needs at least {required_samples + 1}.',
                throttle_duration_sec=1.0
            )
            return None, 0, True

        return self.external_traj, 0, True


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
        now = time.monotonic()
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
        self.thrust_ratio_estimator_last_update_monotonic = None
        self.thrust_ratio_full_model_start_state = None
        self.thrust_ratio_full_model_start_monotonic = None
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
            'timestamp_monotonic': time.monotonic(),
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

    def _resync_full_model_kt_interval(self, now_monotonic):
        if self.current_pose is None:
            self.thrust_ratio_full_model_start_state = None
        else:
            self.thrust_ratio_full_model_start_state = np.asarray(
                self.current_pose[:13], dtype=float
            ).copy()
        self.thrust_ratio_full_model_start_monotonic = float(now_monotonic)

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

        now_monotonic = time.monotonic()
        if not estimator.initialized:
            try:
                result = estimator.initialize(float(self.est_params[0]))
                self._resync_full_model_kt_interval(now_monotonic)
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
            or self.thrust_ratio_full_model_start_monotonic is None
        ):
            self._resync_full_model_kt_interval(now_monotonic)
            self.thrust_ratio_ukf_last_result = estimator.snapshot(
                'resynced_interval'
            )
            return self.thrust_ratio_ukf_last_result

        elapsed = (
            now_monotonic - self.thrust_ratio_full_model_start_monotonic
        )
        if elapsed < self.thrust_ratio_estimator_period_s:
            self.thrust_ratio_ukf_last_result = estimator.snapshot('rate_limited')
            return self.thrust_ratio_ukf_last_result

        if elapsed > self.thrust_ratio_full_model_max_dt_s:
            self._resync_full_model_kt_interval(now_monotonic)
            self.thrust_ratio_ukf_last_result = estimator.snapshot(
                'skipped_large_dt'
            )
            return self.thrust_ratio_ukf_last_result

        extra_delay_s = max(
            0, self.thrust_ratio_ukf_control_delay_steps - 1
        ) * DT
        interval_start = (
            self.thrust_ratio_full_model_start_monotonic - extra_delay_s
        )
        interval_end = now_monotonic - extra_delay_s
        input_samples = [
            sample for sample in self.thrust_ratio_ukf_input_history
            if interval_start < sample['timestamp_monotonic'] <= interval_end
        ]
        if not input_samples:
            self.thrust_ratio_ukf_last_result = estimator.snapshot(
                'waiting_for_delayed_input'
            )
            return self.thrust_ratio_ukf_last_result

        if len(input_samples) > self.thrust_ratio_full_model_max_substeps:
            self._resync_full_model_kt_interval(now_monotonic)
            self.thrust_ratio_ukf_last_result = estimator.snapshot(
                'skipped_too_many_substeps'
            )
            return self.thrust_ratio_ukf_last_result

        mean_throttle = float(np.mean([
            sample['throttle'] for sample in input_samples
        ]))
        if mean_throttle < self.thrust_ratio_ukf_min_throttle:
            self._resync_full_model_kt_interval(now_monotonic)
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
            self._resync_full_model_kt_interval(now_monotonic)

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

        now_monotonic = time.monotonic()
        if not estimator.initialized:
            try:
                result = estimator.initialize(self.current_pose[:13], float(self.est_params[0]))
                self.thrust_ratio_estimator_last_update_monotonic = now_monotonic
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

        assert self.thrust_ratio_estimator_last_update_monotonic is not None
        elapsed = now_monotonic - self.thrust_ratio_estimator_last_update_monotonic
        if elapsed < self.thrust_ratio_estimator_period_s:
            self.thrust_ratio_ukf_last_result = estimator.snapshot('rate_limited')
            return self.thrust_ratio_ukf_last_result

        if elapsed > self.thrust_ratio_vertical_max_dt_s:
            self.thrust_ratio_estimator_last_update_monotonic = now_monotonic
            self.thrust_ratio_ukf_last_result = estimator.snapshot('skipped_large_dt')
            return self.thrust_ratio_ukf_last_result

        # control_state already represents the command from the preceding MPC
        # interval. delay_steps=1 therefore requires no additional time shift.
        extra_delay_s = max(0, self.thrust_ratio_ukf_control_delay_steps - 1) * DT
        interval_start = self.thrust_ratio_estimator_last_update_monotonic - extra_delay_s
        interval_end = now_monotonic - extra_delay_s
        input_samples = [
            sample for sample in self.thrust_ratio_ukf_input_history
            if interval_start < sample['timestamp_monotonic'] <= interval_end
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
            self.thrust_ratio_estimator_last_update_monotonic = now_monotonic
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
            self.thrust_ratio_estimator_last_update_monotonic = now_monotonic
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
                self.thrust_ratio_ukf_start_time = time.time()
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
            and (time.time() - self.thrust_ratio_ukf_start_time)
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


        if self.shutdown_requested:
            self.cb.request_shutdown()
            return

        if self.armed and (time.time() - self.last_pose_update_time) > POSE_TIMEOUT_THRESHOLD:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0, channel_5=0.0, channel_6=0.0, channel_7=0.0, channel_8=0.0, channel_9=0.0, channel_10=0.0)
            self.cb.disarm(msg)
            return 
        
        if self.current_pose is None:
            self.get_logger().warn("Waiting for /motion_capture_state...")
            return

        if self.thrust_ratio_ukf_initial_z is None:
            self.thrust_ratio_ukf_initial_z = float(self.current_pose[2])

        if not self.armed:
            age = (time.time() - self.last_external_reference_time
                   if self.last_external_reference_time is not None else None)
            self.get_logger().info(
                "[approach] DISARMED — waiting for ARM on this node's command topic. "
                f"pose=ok ext_ref={'NONE' if self.external_traj is None else f'{age:.2f}s old'}",
                throttle_duration_sec=2.0)

        if self.armed and self.current_pose is not None:
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
            # Per-cycle approach diagnostics throttled to ~0.5 Hz (they were flooding the launch
            # stdout every control cycle). Post-weld this node no longer drives the drone.
            self.get_logger().info(
                f"[approach] current pos = {self.current_pose[0:3]}, "
                f"ref pos = {traj_for_mpc[0:3, ref_step]}, pendulum = {self.pendulum_state}",
                throttle_duration_sec=2.0,
            )
            # Capture the value actually supplied to this MPC solve. Active UKF
            # feedback is applied later in the loop and therefore affects the next solve.
            mpc_thrust_ratio_used = float(self.est_params[0])

            # set_trajectory_reference_aligned(self.ocp, self.traj, self.N, self.step_counter, self.skip_steps, self.est_params)
            set_payload_trajectory_reference_aligned(
                self.ocp,
                traj_for_mpc,
                self.N,
                ref_step,
                self.skip_steps,
                self.est_params
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
                estimated_state,   # 13
                control_state,     # 4
                payload_state      # 4
            ))

            assert len(estimated_state_with_control) == 21

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

            if initial_state.shape != (21,):
                raise ValueError(
                    f"Expected 21 initial states, got shape {initial_state.shape}"
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
            status = self.ocp.solve()
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
                self.get_logger().info(
                    f"[approach] r:{round(u[0],3)} p:{round(u[1],3)} t:{round(u[2],3)} "
                    f"y:{round(u[3],3)} TR:{round(self.est_params[0],2)}",
                    throttle_duration_sec=2.0)
            else:
                # Stay armed but don't send thrust commands until takeoff
                msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0, channel_5=1.0, channel_6=1.0, channel_7=1.0, channel_8=1.0, channel_9=1.0, channel_10=1.0)
                self.get_logger().info("[approach] Armed - waiting for TAKEOFF",
                                       throttle_duration_sec=2.0)
            
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
            
            log_row = [
                self.step_counter,
                time.time(),
                float(u[0]), float(u[1]), float(u[2]), float(u[3]),
                float(self.current_pose[0]), float(self.current_pose[1]), float(self.current_pose[2]),
                float(self.current_pose[3]), float(self.current_pose[4]), float(self.current_pose[5]), float(self.current_pose[6]),
                float(self.pendulum_state[0]), float(self.pendulum_state[1]),
                float(self.pendulum_state[2]), float(self.pendulum_state[3]),
                float(payload_swing_error),
                float(mpc_thrust_ratio_used),
                float(self.est_params[0]),
                bool(self.enable_thrust_ratio_feedback),
                bool(self.thrust_ratio_feedback_applied),
                str(self.thrust_ratio_feedback_status),
                float(self.thrust_ratio_feedback_target),
                bool(self.enable_thrust_ratio_ukf),
                str(self.thrust_ratio_estimator_backend),
                float(
                    self.thrust_ratio_estimator_rate_hz
                    if self.thrust_ratio_estimator_backend in (
                        'vertical_ukf', 'full_model_kt_ukf'
                    )
                    else FREQUENCY_HZ
                ),
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
                int(self.thrust_ratio_ukf_control_delay_steps)
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

        else:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0, channel_5=0.0, channel_6=0.0, channel_7=0.0, channel_8=0.0, channel_9=0.0, channel_10=0.0)
            self.cb.cmd_publisher_.publish(msg)
            self.step_counter = 0
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
    print('[approach_mpc] main() reached — process started', flush=True)
    rclpy.init(args=args)
    controller = Controller()
    print('[approach_mpc] Controller constructed OK — entering spin '
          '(acados compile done)', flush=True)
    signal.signal(signal.SIGINT, controller.signal_handler)
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
    except Exception as e:
        import traceback
        print(f"Exception occurred: {e}")
        traceback.print_exc()
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
