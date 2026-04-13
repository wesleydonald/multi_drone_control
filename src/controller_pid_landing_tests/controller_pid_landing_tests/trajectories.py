import numpy as np
from scipy.spatial.transform import Rotation as R

def takeoff_trajectory(dt):
    steps_takeoff = 4 * 120 
    steps_hover = 4 * 120

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
    steps_move_back = 3 * 120 
    steps_descend = 3 * 120 
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

    steps = 5 * 120  # 10 seconds of hover at 30 Hz
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
    steps_hover = 20 * 120  # 20 seconds of hover at 30 Hz
    time_space_hover = np.linspace(0, steps_hover * dt, steps_hover)
    x_traj_hover = np.zeros_like(time_space_hover)
    y_traj_hover = np.zeros_like(time_space_hover)
    z_traj_hover = 1.5 * np.ones_like(time_space_hover)  # Constant hover at 1.5m
    
    # Phase 2: Variable frequency sinusoidal motion for 40 seconds
    steps_sin = 40 * 120  # 40 seconds of sinusoidal motion at 30 Hz
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

    steps = 40 * 120  # 10 seconds of hover at 30 Hz
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

def takeoff_fly_land_trajectory(dt):
    takeoff_time = 3.0  
    takeoff_steps = int(takeoff_time / dt)  
    
    fly_time = 5.0  
    fly_steps = int(fly_time / dt)  
    
    land_time = 5.0
    land_steps = int(land_time / dt)  
    
    hover_time = 5.0 
    hover_steps = int(hover_time / dt)
    
    takeoff_t = np.linspace(0, takeoff_time, takeoff_steps)
    fly_t = np.linspace(0, fly_time, fly_steps)
    land_t = np.linspace(0, land_time, land_steps)
    hover_t = np.linspace(0, hover_time, hover_steps)
    
    total_t = np.concatenate((takeoff_t, takeoff_time + fly_t, takeoff_time + fly_time + land_t, takeoff_time + fly_time + land_time + hover_t))
    
    takeoff_x = np.zeros_like(takeoff_t)  
    takeoff_y = np.zeros_like(takeoff_t)  
    takeoff_z = np.linspace(0.0, 1.0, takeoff_steps)  
    
    fly_x = np.linspace(0.0, 1.0, fly_steps)  
    fly_y = np.linspace(0.0, 1.0, fly_steps)  
    fly_z = np.ones_like(fly_t) * 1.0  
    
    land_x = np.ones_like(land_t) * 1.0  
    land_y = np.ones_like(land_t) * 1.0  
    land_z = np.linspace(1.0, 0.01, land_steps)  
    
    hover_x = np.ones_like(hover_t) * 1.0
    hover_y = np.ones_like(hover_t) * 1.0 
    hover_z = np.ones_like(hover_t) * 0.01
    
    total_x = np.concatenate((takeoff_x, fly_x, land_x, hover_x))
    total_y = np.concatenate((takeoff_y, fly_y, land_y, hover_y))
    total_z = np.concatenate((takeoff_z, fly_z, land_z, hover_z))
    
    roll = np.zeros_like(total_t)
    pitch = np.zeros_like(total_t)
    yaw = np.zeros_like(total_t)
    rpy = np.vstack((roll, pitch, yaw)).T  
    quaternions = R.from_euler('xyz', rpy).as_quat()  
    qx = quaternions[:, 0]
    qy = quaternions[:, 1]
    qz = quaternions[:, 2]
    qw = quaternions[:, 3]
    
    vx = np.gradient(total_x, dt)
    vy = np.gradient(total_y, dt)
    vz = np.gradient(total_z, dt)
    
    ax = np.zeros_like(total_t)
    ay = np.zeros_like(total_t)
    az = np.zeros_like(total_t)
    u1 = np.zeros_like(total_t)
    u2 = np.zeros_like(total_t)
    u3 = np.zeros_like(total_t)
    u4 = np.zeros_like(total_t)
    
    traj = np.array([
        total_x, total_y, total_z,
        qw, qx, qy, qz,
        vx, vy, vz,
        ax, ay, az,
        u1, u2, u3, u4
    ])
    
    return traj, "takeoff_fly_land"  