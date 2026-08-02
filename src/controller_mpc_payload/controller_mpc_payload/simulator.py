from trajectories import hover_trajectory, z_sin_trajectory, xyz_sine_trajectory, circle_trajectory, power_loop_trajectory, figure8_zsine_trajectory, fast_xyz_sine_trajectory
from acados import generate_ocp_controller, set_initial_guess, warm_start_from_previous_solution, set_trajectory_reference_aligned, update_ocp_parameters
import numpy as np
import time
import copy
import math
import matplotlib.pyplot as plt

FREQUENCY_HZ = 30.0
DT = 1.0 / FREQUENCY_HZ
LOGGING_NAME = "mpc_controller"

class simulator():        
  def __init__(self):
    ## General settings
    self.traj, trajectory_name = circle_trajectory(DT)

    self.observed_state_history = []       
    self.control_history = []
    self.estimated_state_history = []
    
    self.current_pose = np.array([0.0, 0.0, 0.0,  ## Position
                         1.0, 0.0, 0.0, 0.0,      ## Quaternion
                         0.0, 0.0, 0.0,           ## Linear velocity
                         0.0, 0.0, 0.0])          ## Angular velocity
    
    self.step_counter = 0
    self.steps = self.traj.shape[1] - 1

    ## Define MPC Controller parameters
    self.N = 20
    self.skip_steps = 0
    self.first_solve = True
    self.ocp, self.sim_integrator = generate_ocp_controller()

      # Parameters: [Thrust ratio, drag ratio, angular velocity tau, centre rate, max rate, expo]
    self.est_params = np.array([24.0, 0.5, 0.07, 50.0, 50.0, 0.5])

      # Logging
    log_headers = [
        'step', 'timestamp', 'u0', 'u1', 'u2', 'u3',
        'pose_x', 'pose_y', 'pose_z', 'pose_qw', 'pose_qx', 'pose_qy', 'pose_qz',
    ]


  
  def control_loop(self):            
    if self.step_counter + self.N * self.skip_steps > self.steps:
       return False
    
    set_trajectory_reference_aligned(self.ocp, self.traj, self.N, self.step_counter, self.skip_steps, self.est_params)
    estimated_state = copy.deepcopy(self.current_pose[:13])


    if len(self.control_history) == 0:
        estimated_state_with_control = np.concatenate((estimated_state, np.array([0.0, 0.0, 0.0, 0.0]))) 
    else:
        estimated_state_with_control = np.concatenate((estimated_state, np.array(self.control_history[-1][0:4]))) 

    relaxation_factor = 0.025
    relaxed_lbx = estimated_state_with_control * (1 - relaxation_factor)
    relaxed_ubx = estimated_state_with_control * (1 + relaxation_factor)
    self.ocp.set(0, "lbx", relaxed_lbx)
    self.ocp.set(0, "ubx", relaxed_ubx)

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
    x_og = self.ocp.get(0, "x")
    u = x[-4:]
    u_rate = self.ocp.get(0, "u")

    ## Use the sim integrator to propagate the state forward
    sim_state = np.concatenate((self.current_pose[:13], u if len(self.control_history) == 0 else np.array(self.control_history[-1][0:4])))
    self.sim_integrator.set("x", sim_state)
    self.sim_integrator.set("u", u_rate)
    sim_p = np.concatenate([self.est_params, np.array([1.0, 0.0, 0.0, 0.0])])
    self.sim_integrator.set("p", sim_p)
    status_sim = self.sim_integrator.solve()
    if status_sim != 0:
        raise Exception(f"Simulation integrator failed with status {status_sim}.")
    x_next = self.sim_integrator.get("x")
    self.current_pose = copy.deepcopy(x_next)

    # Extract MPC trajectory for visualization
    mpc_trajectory = np.zeros((13, self.N))
    for i in range(self.N):
        x_i = self.ocp.get(i, "x")
        mpc_trajectory[:, i] = x_i[:13]
    
    # Replace the first state with the current measured pose to eliminate offset
    mpc_trajectory[:, 0] = self.current_pose[:13]

    self.step_counter += 1
    
    ## Use the dynamics model to figure out the change 
    log_row = [
        self.step_counter,
        time.time(),
        float(u[0]), float(u[1]), float(u[2]), float(u[3]),
        float(self.current_pose[0]), float(self.current_pose[1]), float(self.current_pose[2]),
        float(self.current_pose[3]), float(self.current_pose[4]), float(self.current_pose[5]), float(self.current_pose[6])
    ]

    self.control_history.append( np.concatenate( (u, u_rate) ).tolist() )
    self.observed_state_history.append( self.current_pose[:13].tolist() )
    self.estimated_state_history.append( estimated_state[:13].tolist() )
    return True

if __name__ == '__main__':
  sim = simulator()
  # print(sim.traj[:3, :].shape)
  while sim.control_loop() is True:
    continue
  
  ## Plot the simulation results
  fig, obs_axes = plt.subplots(4, 2, figsize=(18, 15))
  obs_axes = obs_axes.flatten()

  distance_err = []
  x_err = []
  y_err = []
  z_err = []
  for t, pose_t in enumerate(sim.observed_state_history):
    # distance_err.append(math.sqrt((pose_t[0]-sim.traj[0, t])**2+(pose_t[1]-sim.traj[1,t])**2+(pose_t[2]-sim.traj[2,t])**2))
    x_err.append(pose_t[0]-sim.traj[0,t])
    y_err.append(pose_t[1]-sim.traj[1,t])
    z_err.append(pose_t[2]-sim.traj[2,t])
  
  runtime = len(sim.observed_state_history) 
  timesteps = DT * np.arange(runtime)

  # 1. Distance error
  obs_axes[0].plot(timesteps, x_err, label='x_err')
  obs_axes[0].plot(timesteps, y_err, label='y_err')
  obs_axes[0].plot(timesteps, z_err, label='z_err')
  # obs_axes[0].plot(timesteps, distance_err)
  obs_axes[0].legend()
  obs_axes[0].set_title("Distance between the quadcopter and target")
  obs_axes[0].set_xlabel("Time (s)")
  obs_axes[0].set_ylabel("Distance")
  obs_axes[0].grid(alpha=0.3)

  # 2. Trajectory mapping
  obs_axes[1].plot(timesteps, sim.traj[0,:runtime], label="x")
  obs_axes[1].plot(timesteps, sim.traj[1,:runtime], label="y")
  obs_axes[1].plot(timesteps, sim.traj[2,:runtime], label="z")
  obs_axes[1].legend()
  obs_axes[1].set_title("Trajectory Position")
  obs_axes[1].set_xlabel("Time (s)")
  obs_axes[1].set_ylabel("Position")
  obs_axes[1].grid(alpha=0.3)

  # 3. Control outputs
  obs_axes[2].plot(timesteps, np.array(sim.control_history)[:runtime, 0], label="roll")
  obs_axes[2].plot(timesteps, np.array(sim.control_history)[:runtime, 1], label="pitch")
  obs_axes[2].plot(timesteps, np.array(sim.control_history)[:runtime, 2], label="thrust")
  obs_axes[2].plot(timesteps, np.array(sim.control_history)[:runtime, 3], label="yaw")

  obs_axes[2].legend()
  obs_axes[2].set_title("Control")
  obs_axes[2].set_xlabel("Time (s)")
  obs_axes[2].set_ylabel("Throttle")
  obs_axes[2].grid(alpha=0.3)
  plt.show()
  print(f"DONE: {sim.step_counter} STEPS TAKEN")