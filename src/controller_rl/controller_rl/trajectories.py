import numpy as np
from scipy.spatial.transform import Rotation as R

def takeoff_trajectory(dt):
    steps_takeoff = 4 * 100
    steps_hover = 4 * 100

    time_space_takeoff = np.linspace(0, steps_takeoff * dt, steps_takeoff)
    x_traj_takeoff = np.zeros_like(time_space_takeoff)
    y_traj_takeoff = np.zeros_like(time_space_takeoff)
    z_traj_takeoff = np.linspace(0.0, 1.0, steps_takeoff)

    time_space_hover = np.linspace(0, steps_hover * dt, steps_hover)
    x_traj_hover = np.zeros_like(time_space_hover)
    y_traj_hover = np.zeros_like(time_space_hover)
    z_traj_hover = np.ones_like(time_space_hover) * 1.0

    x_traj = np.concatenate((x_traj_takeoff, x_traj_hover))
    y_traj = np.concatenate((y_traj_takeoff, y_traj_hover))
    z_traj = np.concatenate((z_traj_takeoff, z_traj_hover))
    time_space_total = np.concatenate((time_space_takeoff, time_space_hover))

    roll_traj = np.zeros_like(time_space_total)
    pitch_traj = np.zeros_like(time_space_total)
    yaw_traj = np.zeros_like(time_space_total)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space_total)
    ay_traj = np.zeros_like(time_space_total)
    az_traj = np.zeros_like(time_space_total)
    u1 = np.zeros_like(time_space_total)
    u2 = np.zeros_like(time_space_total)
    u3 = np.zeros_like(time_space_total)
    u4 = np.zeros_like(time_space_total)
    traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                     vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])
    return traj, "takeoff"
    

def land_trajectory(dt, init_pose):
    steps_move_back = 3 * 100 
    steps_descend = 3 * 100 
    time_space_move_back = np.linspace(0, steps_move_back * dt, steps_move_back)
    time_space_descend = np.linspace(0, steps_descend * dt, steps_descend)

    time_space_total = np.concatenate((time_space_move_back, time_space_descend))

    x_traj_move_back = np.linspace(init_pose[0], 0, steps_move_back)  # Move back 1 meter
    x_traj_descend = np.linspace(0, 0, steps_descend)  # Move back 1 meter
    x_traj = np.concatenate((x_traj_move_back, x_traj_descend))

    y_traj_move_back = np.linspace(init_pose[1], 0, steps_move_back)  # Move back 1 meter
    y_traj_descend = np.linspace(0, 0, steps_descend)  # Move back 1 meter
    y_traj = np.concatenate((y_traj_move_back, y_traj_descend))

    z_traj_move_back = np.linspace(init_pose[2], 1.0, steps_move_back)  # Move back 1 meter
    z_traj_descend  = np.linspace(1.0, 0.1, steps_descend)  # Move back 1 meter
    z_traj = np.concatenate((z_traj_move_back, z_traj_descend))

    roll_traj = np.zeros_like(time_space_total)
    pitch_traj = np.zeros_like(time_space_total)
    yaw_traj = np.zeros_like(time_space_total)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space_total)
    ay_traj = np.zeros_like(time_space_total)
    az_traj = np.zeros_like(time_space_total)
    u1 = np.zeros_like(time_space_total)
    u2 = np.zeros_like(time_space_total)
    u3 = np.zeros_like(time_space_total)
    u4 = np.zeros_like(time_space_total)
    traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                     vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])
    return traj, "land"



def hover_trajectory(dt):
    take_off_traj, _ = takeoff_trajectory(dt)

    steps = 5 * 100  # 10 seconds of hover at 30 Hz
    time_space = np.linspace(0, steps * dt, steps)
    x_traj = np.zeros_like(time_space) #np.zeros_like(time_space)
    y_traj = np.zeros_like(time_space) #np.zeros_like(time_space) # 0.0 + 0.5 * np.sin(1.0 * np.pi * time_space / 5.0)
    z_traj = 1.5 * np.ones_like(time_space)#1.2 * np.ones_like(time_space) #1.5 + 0.5 * np.sin(1.5 * np.pi * time_space / 5.0)    #

    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space)
    ay_traj = np.zeros_like(time_space)
    az_traj = np.zeros_like(time_space)
    u1 = np.zeros_like(time_space)
    u2 = np.zeros_like(time_space)
    u3 = np.zeros_like(time_space)
    u4 = np.zeros_like(time_space)
    hover_traj =  np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                 vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])

    land_traj, _ = land_trajectory(dt, take_off_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

    traj = np.concatenate((take_off_traj,hover_traj, land_traj,zeros), axis=1)
    return traj, "hover"




def z_sin_trajectory(dt):
    trajectory_name = "ZSINE"
    take_off_traj, _ = takeoff_trajectory(dt)

    # Phase 1: Hover for 20 seconds
    steps_hover = 20 * 100  # 20 seconds of hover at 30 Hz
    time_space_hover = np.linspace(0, steps_hover * dt, steps_hover)
    x_traj_hover = np.zeros_like(time_space_hover)
    y_traj_hover = np.zeros_like(time_space_hover)
    z_traj_hover = 1.5 * np.ones_like(time_space_hover)  # Constant hover at 1.5m
    
    # Phase 2: Variable frequency sinusoidal motion for 40 seconds
    steps_sin = 40 * 100  # 40 seconds of sinusoidal motion at 30 Hz
    time_space_sin = np.linspace(0, steps_sin * dt, steps_sin)
    
    # Frequency increases linearly from 1.0 to 3.0 Hz over 40 seconds
    freq_start = 0.1  # Hz
    freq_end = 0.5    # Hz
    frequencies = np.linspace(freq_start, freq_end, steps_sin)
    
    # Calculate the instantaneous phase by integrating frequency
    # phase(t) = 2π * ∫frequency(τ)dτ from 0 to t
    phase = 2 * np.pi * np.cumsum(frequencies) * dt
    
    x_traj_sin = np.zeros_like(time_space_sin)
    y_traj_sin = np.zeros_like(time_space_sin)
    z_traj_sin = 1.5 + 1.0 * np.sin(phase)  # 1m amplitude around 1.5m center
    
    # Combine both phases
    x_traj = np.concatenate((x_traj_hover, x_traj_sin))
    y_traj = np.concatenate((y_traj_hover, y_traj_sin))
    z_traj = np.concatenate((z_traj_hover, z_traj_sin))
    time_space = np.concatenate((time_space_hover, time_space_sin))

    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space)
    ay_traj = np.zeros_like(time_space)
    az_traj = np.zeros_like(time_space)
    u1 = np.zeros_like(time_space)
    u2 = np.zeros_like(time_space)
    u3 = np.zeros_like(time_space)
    u4 = np.zeros_like(time_space)
    hover_traj =  np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                 vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])

    land_traj, _ = land_trajectory(dt, hover_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

    traj = np.concatenate((take_off_traj,hover_traj, land_traj,zeros), axis=1)
    return traj, "z_sin"




def xyz_sine_trajectory(dt):
    take_off_traj, _ = takeoff_trajectory(dt)

    steps = 40 * 100  # 10 seconds of hover at 30 Hz
    time_space = np.linspace(0, steps * dt, steps)
    x_traj = 0.0 + 1.0 * np.sin(0.5 * np.pi * time_space / 5.0) #np.zeros_like(time_space)
    y_traj = 0.0 + 1.0 * np.sin(0.25 * np.pi * time_space / 5.0) #np.zeros_like(time_space) # 0.0 + 0.5 * np.sin(1.0 * np.pi * time_space / 5.0)
    z_traj = 1.5 + 0.5 * np.sin(1.0 * np.pi * time_space / 5.0)#1.2 * np.ones_like(time_space) #1.5 + 0.5 * np.sin(1.5 * np.pi * time_space / 5.0)    #

    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space)
    ay_traj = np.zeros_like(time_space)
    az_traj = np.zeros_like(time_space)
    u1 = np.zeros_like(time_space)
    u2 = np.zeros_like(time_space)
    u3 = np.zeros_like(time_space)
    u4 = np.zeros_like(time_space)
    hover_traj =  np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                 vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])

    land_traj, _ = land_trajectory(dt, take_off_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

    traj = np.concatenate((take_off_traj,hover_traj, land_traj,zeros), axis=1)
    return traj, "xyz_sine"
