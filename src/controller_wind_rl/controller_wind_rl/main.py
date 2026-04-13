import rclpy
import signal
import sys
import os
import matplotlib.pyplot as plt
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
from .wind_estimator import LSTM_wind_estimator, predict_cpu
from skrl.utils.runner.torch import Runner
import torch
import gym as ogym
import yaml
from ament_index_python.packages import get_package_share_directory
from collections import deque

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
    


## Frequency was 50 Hz in Betaflight, changed it here 

POSE_TIMEOUT_THRESHOLD = 0.25  # seconds
FREQUENCY_HZ = 100.0
DT = 1.0 / FREQUENCY_HZ

LOGGING_NAME = 'controller_wind_rl'

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # General Settings
        self.last_pose_update_time = time.time()
        self.cb = CallbackManager(self)

        self.traj, trajectory_name = xyz_sine_trajectory(DT)
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
        self.observation_history = deque()
        self.wind_estimate = deque()


        # RL Agent Loading
        # Prefer package source directory (src/) so we can load models during development.
        # Search upwards from a couple of sensible starting points (cwd and this file's dir)
        # for a directory that contains `src/controller_wind_rl` and use that when found.
        ckpt_rel = "ppo_model.pt"
        agent_yaml_rel = "skrl_ppo_cfg.yaml"
        wind_model_rel = "wind_model.pth"

        def find_src_pkg_dir(pkg_name: str):
            # allow override from environment
            env_key = f"{pkg_name.upper()}_SRC_DIR"
            env_val = os.environ.get(env_key)
            if env_val and os.path.isdir(env_val):
                return os.path.abspath(env_val)

            # candidates to start searching upwards
            candidates = [os.getcwd(), os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))]
            # second candidate attempts to move out of installed package build/install folders

            for start in candidates:
                cur = os.path.abspath(start)
                while True:
                    candidate = os.path.join(cur, 'src', pkg_name)
                    if os.path.isdir(candidate):
                        return os.path.abspath(candidate)
                    parent = os.path.dirname(cur)
                    if parent == cur:
                        break
                    cur = parent
            return None

        pkg_src_found = find_src_pkg_dir('controller_wind_rl')

        if pkg_src_found:
            run_dir = pkg_src_found
        else:
            # fallback to installed package share directory
            try:
                pkg_share = get_package_share_directory("controller_wind_rl")
            except Exception as e:
                raise RuntimeError("Could not find package 'controller_wind_rl' in install or source paths") from e
            run_dir = pkg_share

        ckpt_path = os.path.join(run_dir, ckpt_rel)
        agent_yaml_path = os.path.join(run_dir, agent_yaml_rel)
        wind_model_path = os.path.join(run_dir, wind_model_rel)

        # keep explicit src paths for additional checks/logging
        ckpt_path_src = os.path.join(pkg_src_found, ckpt_rel) if pkg_src_found else None
        agent_yaml_path_src = os.path.join(pkg_src_found, agent_yaml_rel) if pkg_src_found else None
        wind_model_path_src = os.path.join(pkg_src_found, wind_model_rel) if pkg_src_found else None

        if not os.path.exists(ckpt_path):
            raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")
        if not os.path.exists(agent_yaml_path):
            raise FileNotFoundError(f"Agent YAML not found: {agent_yaml_path}")

        self.get_logger().info(f"Loading skrl agent from:\n- ckpt: {ckpt_path}\n- cfg : {agent_yaml_path}")


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
        
        self.data_history =  {
            "pos_err_x": [],
            "pos_err_y": [],
            "pos_err_z": [],
            "throttle": [],
            "wind_x": [], 
            "wind_y": [],
            "wind_z": [],
            "roll": [],
            "pitch": [], 
            "yaw": [],
        }


        HIDDEN_DIM = 64
        INPUT_SIZE = 22
        self.model = LSTM_wind_estimator(hidden_dim=HIDDEN_DIM, input_size=INPUT_SIZE)
        # load wind model weights if available (prefer src/ path)
        try:
            if os.path.exists(wind_model_path):
                state = torch.load(wind_model_path, map_location="cpu")
                self.model.load_state_dict(state)
                self.get_logger().info(f"Loaded wind model from: {wind_model_path}")
            elif os.path.exists(wind_model_path_src):
                state = torch.load(wind_model_path_src, map_location="cpu")
                self.model.load_state_dict(state)
                self.get_logger().info(f"Loaded wind model from: {wind_model_path_src}")
            else:
                self.get_logger().warning(f"Wind model not found at {wind_model_path} or {wind_model_path_src}; starting with random weights")
        except Exception as e:
            self.get_logger().warning(f"Failed to load wind model: {e}")

    def log(self):
        print("Saving data")
        fig, obs_axes = plt.subplots(2, 2, figsize=(14, 12))
        obs_axes = obs_axes.flatten()
        obs_axes[0].plot(self.data_history["pos_err_x"], label="pos_err_x")
        obs_axes[0].plot(self.data_history["pos_err_y"], label="pos_err_y")
        obs_axes[0].plot(self.data_history["pos_err_z"], label="pos_err_z")
        obs_axes[0].set_title("positional error")
        obs_axes[0].legend()
        obs_axes[0].set_xlabel("timesteps")
        obs_axes[0].set_ylabel("distance(m)")
        
        obs_axes[1].plot(self.data_history["roll"], label="roll")
        obs_axes[1].plot(self.data_history["pitch"], label="pitch")
        obs_axes[1].plot(self.data_history["yaw"], label="yaw")
        obs_axes[1].set_title("Angular rates")
        obs_axes[1].legend()
        obs_axes[1].set_xlabel("timesteps")
        obs_axes[1].set_ylabel("angular rate (rad/s)")
        
        obs_axes[2].plot(self.data_history["throttle"], label="throttle")
        obs_axes[2].set_title("Throttle")
        obs_axes[2].set_xlabel("timesteps")
        obs_axes[2].set_ylabel("throttle")
        print(os.getcwd())
        fig.savefig(os.path.join(os.getcwd(), "src", "controller_wind_rl", "data_logger", f"data {time.time()}.png"))
        PATH = os.path.join(os.getcwd(), "src", "controller_wind_rl", "data_logger", f"data {time.time()}")
        print(f"saved to {PATH}")
        plt.close(fig)


    def control_loop(self):

        start_time = time.time()

        if self.shutdown_requested:
            self.log()
            self.cb.request_shutdown()
            return

        if self.armed and (time.time() - self.last_pose_update_time) > POSE_TIMEOUT_THRESHOLD:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cb.disarm(msg)
            return 
        
        if self.armed and self.current_pose is not None:


            if self.step_counter > self.steps:
                msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
                self.cb.disarm(msg)
                self.cb.request_shutdown()
                self.log()
                return
            
            p = self.current_pose[0:3]
            wxyz = self.current_pose[3:7]
            v_world = self.current_pose[7:10]
            w_body = self.current_pose[10:13]

            quat_xyzw = np.array([wxyz[1], wxyz[2], wxyz[3], wxyz[0]], dtype=np.float32)
            Rwb = R.from_quat(quat_xyzw).as_matrix().astype(np.float32)
            v_body = Rwb.T @ v_world

            # Desired relative position in body frame

            setpoint = [0.0, 0.0, 1.1]
            pos_err_world = setpoint - p
            pos_err_body = Rwb.T @ pos_err_world

            # Heading error (yaw)
            x, y, z, w = quat_xyzw
            curr_yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            desired_yaw = 0.0
            yaw_err = np.arctan2(np.sin(curr_yaw - desired_yaw), np.cos(curr_yaw - desired_yaw))
            heading_error = np.array([np.sin(yaw_err), np.cos(yaw_err)], dtype=np.float32)

            ####################################################################
            #                                                                  #
            #                 CHANGED HERE: Added Wind Estimator               #
            #                                                                  #
            ####################################################################
            
            if len(self.observation_history) >= 180: 
                ## Then you pass in wind estimates
                # wind = predict_cpu(self.observation_history, self.model)
                # ret_wind = (np.sum(np.array(self.wind_estimate)) + np.array(wind))/(len(self.wind_estimate) + 1)
                ret_wind = torch.tensor([0.0, 0.0, 0.0], dtype=torch.float32)
            else:
                ## Replace here with the perfect wind knowledge
                ret_wind = torch.tensor([0.0,0.0, 0.0], dtype=torch.float32)

            LEN_AVERAGE = 50
            if len(self.wind_estimate) >= LEN_AVERAGE:
                self.wind_estimate.popleft()
            self.wind_estimate.append(ret_wind)

            # Observation layout: [v_b(3), w_b(3), quat wxyz(4), pos_err_b(3), heading(2), prev_actions(4)] = 19
            obs = np.concatenate(
                [v_body, w_body, wxyz, pos_err_body, heading_error, self._prev_actions, ret_wind], dtype=np.float32
            )
            
            self._obs = np.round(obs, 3).astype(np.float32)

            print(f"self._obs: {self._obs}")
            obs_t = torch.from_numpy(self._obs).to(self._agent_device, dtype=torch.float32).unsqueeze(0)

            if len(self.observation_history) >= 180:
                self.observation_history.popleft()
            self.observation_history.append(obs)

            ####################################################################
            #                                                                  #
            #                 CHANGED HERE: Added Wind Estimator               #
            #                                                                  #
            ####################################################################

            with torch.inference_mode():
                outputs = self._runner.agent.act(obs_t, timestep=0, timesteps=0)
                info = outputs[-1] if isinstance(outputs, (tuple, list)) else {}
                unit_action = info.get("mean_actions", outputs[0]).squeeze(0).detach().cpu().numpy().astype(np.float32)

            ## How to visualise wind generated in world_wind.sdf?
            print("Logging data")
            self.data_history["pos_err_x"].append(pos_err_body[0])
            self.data_history["pos_err_y"].append(pos_err_body[1])
            self.data_history["pos_err_z"].append(pos_err_body[2])
            self.data_history["throttle"].append(unit_action[0])
            self.data_history["roll"].append(unit_action[1])
            self.data_history["pitch"].append(unit_action[2])
            self.data_history["yaw"].append(unit_action[3])

            if (self.takeoff_requested):
                self._prev_actions = unit_action.copy()
                u = [float(unit_action[1]), float(unit_action[2]), float(unit_action[0]), float(unit_action[3])]
                msg = ELRSCommand(armed=True, channel_0=round(u[0], 3), channel_1=round(u[1], 3), channel_2=round(u[2], 3), channel_3=round(u[3], 3))
            else:
                u = [0.0, 0.0, -1.0, 0.0]
                self._prev_actions = [0.0, 0.0, 0.0, 0.0]
                msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            
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

        # print (f"Control loop time: {time.time() - start_time:.4f} seconds")

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
        print("hello")
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
