import rclpy
import signal
import sys
import os
import numpy as np
import math
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
import time
from .trajectories import hover_trajectory, z_sin_trajectory, xyz_sine_trajectory
from utility_objects.visualization import TrajectoryVisualizer
from utility_objects.data_logger import DataLogger
from utility_objects.callback_manager import CallbackManager
from interfaces.msg import ELRSCommand
from scipy.spatial.transform import Rotation as R

from skrl.utils.runner.torch import Runner
import torch
import gym as ogym
import yaml
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

# ---------------- Tiny finite-bounds classic-gym env ----------------
class _TinyGymEnv(ogym.Env):
    metadata = {}
    def __init__(self, obs_dim: int, act_dim: int):
        super().__init__()
        big = np.float32(1e6)
        self.observation_space = ogym.spaces.Box(
            low=-big * np.ones((obs_dim,), dtype=np.float32),
            high= big * np.ones((obs_dim,), dtype=np.float32),
            dtype=np.float32,
        )
        self.action_space = ogym.spaces.Box(
            low=-1.0 * np.ones((act_dim,), dtype=np.float32),
            high= 1.0 * np.ones((act_dim,), dtype=np.float32),
            dtype=np.float32,
        )
        self._obs = np.zeros((obs_dim,), dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        return self._obs.copy(), {}

    def step(self, action):
        return self._obs.copy(), 0.0, False, False, {}
    



POSE_TIMEOUT_THRESHOLD = 0.25  # seconds
FREQUENCY_HZ = 100.0
DT = 1.0 / FREQUENCY_HZ

LOGGING_NAME = 'controller_rl'

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # General Settings
        self.last_pose_update_time = time.time()
        self.cb = CallbackManager(self)

        self.traj, trajectory_name = z_sin_trajectory(DT)
        self.trajectory_visualizer = TrajectoryVisualizer(self, frame_id="map")
        self.trajectory_visualizer.publish_all_visualizations(self.traj,  pose_subsample=15, show_velocity=False,  velocity_scale=0.3, color_by_time=True )

        self.timer = self.create_timer(DT, self.control_loop)
        self.step_counter = 0
        self.steps = self.traj.shape[1] - 1

        self.armed = False
        self.takeoff_requested = False
        self.shutdown_requested = False

        # Logging
        log_headers = [
            'step', 'timestamp', 'u0', 'u1', 'u2', 'u3',
            'pose_x', 'pose_y', 'pose_z', 'pose_qw', 'pose_qx', 'pose_qy', 'pose_qz',
        ]
        self.data_logger = DataLogger(LOGGING_NAME, trajectory_name, log_headers)


        #RL Agent Loading
        ckpt_rel = "best_agent.pt"
        agent_yaml_rel = "agent.yaml"

        # Prefer local src copy of the package (useful when running from repo)
        src_pkg_dir = Path(__file__).resolve().parents[1]  # src/controller_rl
        candidates = [
            src_pkg_dir,
            src_pkg_dir / "run",
            Path.cwd(),
            Path(get_package_share_directory("controller_rl")),  # fallback to installed package
        ]

        ckpt_path = None
        agent_yaml_path = None
        used_candidate = None
        for d in candidates:
            c = Path(d) / ckpt_rel
            a = Path(d) / agent_yaml_rel
            if c.exists() and a.exists():
                ckpt_path = str(c)
                agent_yaml_path = str(a)
                used_candidate = str(d)
                break

        if ckpt_path is None or agent_yaml_path is None:
            raise FileNotFoundError(
                f"Couldn't find '{ckpt_rel}' and '{agent_yaml_rel}' in any of: {[str(x) for x in candidates]}"
            )

        self.get_logger().info(f"Loading skrl agent from (candidate: {used_candidate}):\n- ckpt: {ckpt_path}\n- cfg : {agent_yaml_path}")


        ckpt = torch.load(ckpt_path, map_location="cpu")
        pol = ckpt.get("policy", {}) or ckpt.get("agent", {}).get("policy", {})
        if not pol:
            raise RuntimeError("Couldn't find 'policy' state_dict in checkpoint")
        obs_dim = int(pol["net_container.0.weight"].shape[1])
        act_dim = int(pol["policy_layer.bias"].shape[0])

        from skrl.envs.wrappers.torch import wrap_env as wrap_env_torch
        env = _TinyGymEnv(obs_dim, act_dim)
        venv = wrap_env_torch(env, wrapper="gym")

        with open(agent_yaml_path, "r") as f:
            agent_cfg = yaml.safe_load(f)

        agent_cfg.setdefault("agent", {}).setdefault("experiment", {})
        agent_cfg["agent"]["experiment"]["write_interval"] = 0
        agent_cfg["agent"]["experiment"]["checkpoint_interval"] = 0

        self._runner = Runner(venv, agent_cfg)
        self._runner.agent.load(ckpt_path)
        self._runner.agent.set_running_mode("eval")

        self._agent_device = next(self._runner.agent.policy.parameters()).device
        self.get_logger().info(f"Agent device: {self._agent_device}")

        self._prev_actions = np.zeros(4, dtype=np.float32)  # keep in unit space unless you trained differently
        self.get_logger().info("SKRLController initialised.")




    def control_loop(self):

        if self.shutdown_requested:
            self.cb.request_shutdown()
            return

        if self.armed and (time.time() - self.last_pose_update_time) > POSE_TIMEOUT_THRESHOLD:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cb.disarm(msg)
            return 
        
        if self.armed and not self.takeoff_requested and self.current_pose is not None:
            msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cb.cmd_publisher_.publish(msg)

        elif self.armed and self.takeoff_requested and self.current_pose is not None:

            if self.step_counter > self.steps:
                msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
                self.cb.disarm(msg)
                self.cb.request_shutdown()
                return
            
            p = self.current_pose[0:3]
            wxyz = self.current_pose[3:7]
            v_world = self.current_pose[7:10]
            w_body = self.current_pose[10:13]

            quat_xyzw = np.array([wxyz[1], wxyz[2], wxyz[3], wxyz[0]], dtype=np.float32)
            Rwb = R.from_quat(quat_xyzw).as_matrix().astype(np.float32)
            v_body = Rwb.T @ v_world

            # Desired relative position in body frame

            setpoint = self.traj[:, self.step_counter][0:3]
            pos_err_world = setpoint - p
            pos_err_body = Rwb.T @ pos_err_world

            # Heading error (yaw)
            x, y, z, w = quat_xyzw
            curr_yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            desired_yaw = 0.0
            yaw_err = np.arctan2(np.sin(curr_yaw - desired_yaw), np.cos(curr_yaw - desired_yaw))
            heading_error = np.array([np.sin(yaw_err), np.cos(yaw_err)], dtype=np.float32)

            # Observation layout: [v_b(3), w_b(3), quat wxyz(4), pos_err_b(3), heading(2), prev_actions(4)] = 19
            obs = np.concatenate(
                [v_body, w_body, wxyz, pos_err_body, heading_error, self._prev_actions], dtype=np.float32
            )

            self._obs = np.round(obs, 3).astype(np.float32)
            obs_t = torch.from_numpy(self._obs).to(self._agent_device, dtype=torch.float32).unsqueeze(0)

            with torch.inference_mode():
                outputs = self._runner.agent.act(obs_t, timestep=0, timesteps=0)
                info = outputs[-1] if isinstance(outputs, (tuple, list)) else {}
                unit_action = info.get("mean_actions", outputs[0]).squeeze(0).detach().cpu().numpy().astype(np.float32)

            self._prev_actions = unit_action.copy()
            u = [float(unit_action[1]), float(unit_action[2]), float(unit_action[0]), float(unit_action[3])]
            msg = ELRSCommand(armed=True, channel_0=round(u[0], 3), channel_1=round(u[1], 3), channel_2=round(u[2], 3), channel_3=round(u[3], 3))
            self.cb.cmd_publisher_.publish(msg)


            print(f"Step: {self.step_counter}/{self.steps} | Cmd: {u}")

            log_row = [
                self.step_counter,
                time.time(),
                float(u[0]), float(u[1]), float(u[2]), float(u[3]),
                float(self.current_pose[0]), float(self.current_pose[1]), float(self.current_pose[2]),
                float(self.current_pose[3]), float(self.current_pose[4]), float(self.current_pose[5]), float(self.current_pose[6])
            ]
            self.data_logger.append_row(log_row)
            self.trajectory_visualizer.publish_transform_frame(self.current_pose, "drone_mocap")
            self.trajectory_visualizer.publish_actual_path(self.current_pose)
            if self.takeoff_requested:
                self.step_counter += 1

        else:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cb.cmd_publisher_.publish(msg)
            self.step_counter = 0



    def signal_handler(self, sig, frame):
        print("Interrupt received, shutting down...")
        self.on_close()
        sys.exit(0)

    def on_close(self):
        if getattr(self, 'on_close_called', False):
            return
        self.on_close_called = True
        msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
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
