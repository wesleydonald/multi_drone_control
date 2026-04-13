import numpy as np
from scipy.spatial.transform import Rotation as R




def takeoff_trajectory(dt):
    steps = 1 * 30  # 5 seconds of takeoff at 30 Hz
    time_space = np.linspace(0, steps * dt, steps)
    x_traj = np.zeros_like(time_space)
    y_traj = np.zeros_like(time_space)
    z_traj = np.linspace(0.0,1.0,steps)

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
    return np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                     vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj])
    

def land_trajectory(dt, init_pose):
    steps_move_back = 3 * 30  # 5 seconds of takeoff at 30 Hz
    steps_descend = 3 * 30  # 5 seconds of takeoff at 30 Hz
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
    z_traj_descend  = np.linspace(1.0, 0.0, steps_descend)  # Move back 1 meter
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
    return np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                     vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj])


def move_to_start_of_main_trajectory(dt, init_pose, final_pose):
    steps = 3 * 30  # 3 seconds at 30 Hz
    time_space = np.linspace(0, steps * dt, steps)

    x_traj = np.linspace(init_pose[0], final_pose[0], steps)
    y_traj = np.linspace(init_pose[1], final_pose[1], steps)
    z_traj = np.linspace(init_pose[2], final_pose[2], steps)
    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space)
    ay_traj = np.zeros_like(time_space)
    az_traj = np.gradient(vz_traj, dt)
    return np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                     vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj])





def hover_trajectory(dt):
        take_off_traj = takeoff_trajectory(dt)

        steps = 30 * 30  # 10 seconds of hover at 30 Hz
        time_space = np.linspace(0, steps * dt, steps)
        x_traj = np.zeros_like(time_space)
        y_traj = np.zeros_like(time_space)
        z_traj = 1*np.ones_like(time_space)

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
        hover_traj =  np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                     vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj])

        land_traj = land_trajectory(dt, hover_traj[:, -1])
        zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

        return np.concatenate((take_off_traj, hover_traj, land_traj, zeros), axis=1)



def hover_and_rotate(dt):
            take_off_traj = takeoff_trajectory(dt)
            
            # Phase 1: Initial hover
            steps_hover = 3 * 30  # 3 seconds of hover at 30 Hz
            time_space_hover = np.linspace(0, steps_hover * dt, steps_hover)
            x_traj_hover = np.zeros_like(time_space_hover)
            y_traj_hover = np.zeros_like(time_space_hover)
            z_traj_hover = np.ones_like(time_space_hover)
            roll_traj_hover = np.zeros_like(time_space_hover)
            pitch_traj_hover = np.zeros_like(time_space_hover)
            yaw_traj_hover = np.zeros_like(time_space_hover)

            # Phase 2: Tilt to diamond orientation (45 degrees around X and Y axes)
            steps_tilt = 10 * 30  # 5 seconds to tilt
            time_space_tilt = np.linspace(0, steps_tilt * dt, steps_tilt)
            x_traj_tilt = np.zeros_like(time_space_tilt)
            y_traj_tilt = np.zeros_like(time_space_tilt)
            z_traj_tilt = np.ones_like(time_space_tilt)
            
            # Smooth transition to 45 degree tilt on both X and Y axes (diamond orientation)
            target_tilt = np.pi / 4  # 45 degrees in radians
            roll_traj_tilt = target_tilt * (time_space_tilt / time_space_tilt[-1])  # Smooth ramp to 45°
            pitch_traj_tilt = target_tilt * (time_space_tilt / time_space_tilt[-1])  # Smooth ramp to 45°
            yaw_traj_tilt = np.zeros_like(time_space_tilt)

            # Phase 3: Spin around world Z-axis while maintaining diamond orientation
            steps_spin = 20 * 30  # 10 seconds of spinning
            time_space_spin = np.linspace(0, steps_spin * dt, steps_spin)
            x_traj_spin = np.zeros_like(time_space_spin)
            y_traj_spin = np.zeros_like(time_space_spin)
            z_traj_spin = np.ones_like(time_space_spin)
            
            # Maintain diamond orientation while spinning around Z-axis
            roll_traj_spin = np.full_like(time_space_spin, target_tilt)  # Keep 45° roll
            pitch_traj_spin = np.full_like(time_space_spin, target_tilt)  # Keep 45° pitch
            spin_rate = 1.0  # rad/s around Z-axis
            yaw_traj_spin = spin_rate * time_space_spin  # Continuous rotation around Z

            # Phase 4: Return to level orientation
            steps_level = 10 * 30  # 5 seconds to level out
            time_space_level = np.linspace(0, steps_level * dt, steps_level)
            x_traj_level = np.zeros_like(time_space_level)
            y_traj_level = np.zeros_like(time_space_level)
            z_traj_level = np.ones_like(time_space_level)
            
            # Smooth transition back to level
            final_yaw = yaw_traj_spin[-1]
            roll_traj_level = target_tilt * (1 - time_space_level / time_space_level[-1])  # Smooth ramp to 0°
            pitch_traj_level = target_tilt * (1 - time_space_level / time_space_level[-1])  # Smooth ramp to 0°
            yaw_traj_level = np.full_like(time_space_level, final_yaw)  # Hold final yaw

            # Phase 5: Final hover
            steps_final = 3 * 30  # 3 seconds
            time_space_final = np.linspace(0, steps_final * dt, steps_final)
            x_traj_final = np.zeros_like(time_space_final)
            y_traj_final = np.zeros_like(time_space_final)
            z_traj_final = np.ones_like(time_space_final)
            roll_traj_final = np.zeros_like(time_space_final)
            pitch_traj_final = np.zeros_like(time_space_final)
            yaw_traj_final = np.full_like(time_space_final, final_yaw)  # Hold final yaw

            # Concatenate all phases
            x_traj = np.concatenate((x_traj_hover, x_traj_tilt, x_traj_spin, 
                                     x_traj_level, x_traj_final))
            y_traj = np.concatenate((y_traj_hover, y_traj_tilt, y_traj_spin, 
                                     y_traj_level, y_traj_final))
            z_traj = np.concatenate((z_traj_hover, z_traj_tilt, z_traj_spin, 
                                     z_traj_level, z_traj_final))
            roll_traj = np.concatenate((roll_traj_hover, roll_traj_tilt, roll_traj_spin, 
                                        roll_traj_level, roll_traj_final))
            pitch_traj = np.concatenate((pitch_traj_hover, pitch_traj_tilt, pitch_traj_spin, 
                                         pitch_traj_level, pitch_traj_final))
            yaw_traj = np.concatenate((yaw_traj_hover, yaw_traj_tilt, yaw_traj_spin, 
                                       yaw_traj_level, yaw_traj_final))

            # Convert Euler angles to quaternions
            rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
            quaternions = R.from_euler('xyz', rpy_traj).as_quat()
            qx_traj = quaternions[:, 0]
            qy_traj = quaternions[:, 1]
            qz_traj = quaternions[:, 2]
            qw_traj = quaternions[:, 3]

            # Calculate velocities
            vx_traj = np.gradient(x_traj, dt)
            vy_traj = np.gradient(y_traj, dt)
            vz_traj = np.gradient(z_traj, dt)
            ax_traj = np.zeros_like(x_traj)
            ay_traj = np.zeros_like(y_traj)
            az_traj = np.zeros_like(z_traj)

            hover_rotate_traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                                          vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj])

            land_traj = land_trajectory(dt, hover_rotate_traj[:, -1])
            zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

            return np.concatenate((take_off_traj, hover_rotate_traj, land_traj, zeros), axis=1)


def circle_trajectory(dt):
    take_off_traj = takeoff_trajectory(dt)
    move_to_start = move_to_start_of_main_trajectory(dt, take_off_traj[:, -1], np.array([1.5, 0.0, 1.5]))

    radius = 0.5
    height = 1.5
    angular_velocity_start = 0.1  # m/s
    angular_velocity_end = 0.2  # rad/s
    duration = 30
    steps = int(duration / dt)

    time_space = np.linspace(0, duration, steps)
    angular_velocity = np.linspace(angular_velocity_start, angular_velocity_end, steps)

    theta = np.cumsum(angular_velocity * dt)  
    x_traj = radius * np.cos(theta)
    y_traj = radius * np.sin(theta)
    z_traj = np.full_like(x_traj, height)

    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]

    vx_traj = -radius * angular_velocity * np.sin(theta)
    vy_traj = radius * angular_velocity * np.cos(theta)
    vz_traj = np.zeros_like(vx_traj)
    ax_traj = np.zeros_like(vx_traj)
    ay_traj = np.zeros_like(vy_traj)
    az_traj = np.zeros_like(vx_traj)

    circular_traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                              vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj])

    land_traj = land_trajectory(dt, circular_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

    return np.concatenate((take_off_traj, move_to_start, circular_traj, land_traj, zeros), axis=1)






def power_loop_trajectory(dt):
    take_off_traj = takeoff_trajectory(dt)
    move_to_start = move_to_start_of_main_trajectory(dt, take_off_traj[:, -1], np.array([0.0, 0.0, 0.8]))

    # Define parameters for the power loop
    radius = 1.2
    origin = np.array([0.0, 0.0, 2.0])
    angular_speed = 3.0  # rad/s
    duration = 2 * np.pi / angular_speed  # Time to complete one loop
    steps = int(duration / dt)

    # Time array for the power loop
    time_space = np.linspace(0, duration, steps)

    # Circular trajectory in the XZ plane (loop)
    theta = angular_speed * time_space
    x_traj = radius * np.sin(theta) + origin[0]
    y_traj = np.full_like(x_traj, origin[1])  # Y remains constant
    z_traj = -radius * np.cos(theta) + origin[2]  # Start from origin - radius in Z

    # Orientation (quaternions) changes to pitch back during the loop
    roll_traj = np.zeros_like(time_space)
    pitch_traj = -theta  # Pitch back as the drone loops
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]

    # Velocities and accelerations
    vx_traj = radius * angular_speed * np.cos(theta)
    vy_traj = np.zeros_like(vx_traj)
    vz_traj = -radius * angular_speed * np.sin(theta)
    ax_traj = -radius * angular_speed**2 * np.sin(theta)
    ay_traj = np.zeros_like(vx_traj)
    az_traj = np.zeros_like(vx_traj)

    power_loop_traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                                vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj])

    land_traj = land_trajectory(dt, power_loop_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 2 * 30))

    return np.concatenate((take_off_traj, move_to_start, power_loop_traj, land_traj, zeros), axis=1)


def hover_and_yaw(dt):
    take_off_traj = takeoff_trajectory(dt)
    
    # Phase 1: Initial hover
    hover_duration = 10.0  # 3 seconds of initial hover
    hover_steps = int(hover_duration / dt)
    hover_time_space = np.linspace(0, hover_duration, hover_steps)
    
    x_traj_hover = np.zeros_like(hover_time_space)
    y_traj_hover = np.zeros_like(hover_time_space)
    z_traj_hover = np.ones_like(hover_time_space)  # Hover at 1 meter altitude
    
    roll_traj_hover = np.zeros_like(hover_time_space)
    pitch_traj_hover = np.zeros_like(hover_time_space)
    yaw_traj_hover = np.zeros_like(hover_time_space)
    
    # Phase 2: Full yaw rotation in 8 seconds
    yaw_duration = 20.0  # 8 seconds for full rotation
    yaw_steps = int(yaw_duration / dt)
    yaw_time_space = np.linspace(0, yaw_duration, yaw_steps)
    
    x_traj_yaw = np.zeros_like(yaw_time_space)
    y_traj_yaw = np.zeros_like(yaw_time_space)
    z_traj_yaw = np.ones_like(yaw_time_space)  # Maintain altitude
    
    roll_traj_yaw = np.zeros_like(yaw_time_space)
    pitch_traj_yaw = np.zeros_like(yaw_time_space)
    yaw_traj_yaw = np.zeros_like(yaw_time_space)
    # Full 360 degree rotation (2π radians) over 8 seconds
    #yaw_traj_yaw= 2 * np.pi * (yaw_time_space / yaw_duration)
    yaw_traj_yaw= 2 * np.pi * (yaw_time_space / yaw_duration)
    
    # Phase 3: Final hover
    final_hover_duration = 10.0  # 3 seconds of final hover
    final_hover_steps = int(final_hover_duration / dt)
    final_hover_time_space = np.linspace(0, final_hover_duration, final_hover_steps)
    
    x_traj_final = np.zeros_like(final_hover_time_space)
    y_traj_final = np.zeros_like(final_hover_time_space)
    z_traj_final = np.ones_like(final_hover_time_space)
    
    roll_traj_final = np.zeros_like(final_hover_time_space)
    pitch_traj_final = np.zeros_like(final_hover_time_space)
    yaw_traj_final = np.full_like(final_hover_time_space, 2 * np.pi)  # Hold final yaw position
    
    # Concatenate all phases
    x_traj = np.concatenate((x_traj_hover, x_traj_yaw, x_traj_final))
    y_traj = np.concatenate((y_traj_hover, y_traj_yaw, y_traj_final))
    z_traj = np.concatenate((z_traj_hover, z_traj_yaw, z_traj_final))
    roll_traj = np.concatenate((roll_traj_hover, roll_traj_yaw, roll_traj_final))
    pitch_traj = np.concatenate((pitch_traj_hover, pitch_traj_yaw, pitch_traj_final))
    yaw_traj = np.concatenate((yaw_traj_hover, yaw_traj_yaw, yaw_traj_final))
    
    # Convert Euler angles to quaternions
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    
    # Calculate velocities
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    
    # Angular velocities (desired rates)
    ax_traj = np.zeros_like(x_traj)
    ay_traj = np.zeros_like(y_traj)
    az_traj = np.zeros_like(z_traj)
    
    hover_yaw_traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                               vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj])
    
    land_traj = land_trajectory(dt, hover_yaw_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))
    
    return np.concatenate((take_off_traj, hover_yaw_traj, land_traj, zeros), axis=1)


def sine_wave_trajectory(dt):
    take_off_traj = takeoff_trajectory(dt)
    
    # Define sine wave parameters
    amplitude = 2.0  # 1 meter amplitude
    wavelength = 1.0  # 4 meters per complete wave
    height = 1.5  # Fixed altitude
    x_start = -1.0  # Start position (to center the sine wave)
    x_end = 1.0    # End position
    duration = 15.0  # Duration to complete the sine wave motion
    
    # Move to start position of sine wave
    move_to_start = move_to_start_of_main_trajectory(dt, take_off_traj[:, -1], np.array([x_start, 0.0, height]))
    
    # Create sine wave trajectory
    steps = int(duration / dt)
    time_space = np.linspace(0, duration, steps)
    
    # X trajectory: linear motion from start to end
    x_traj = np.linspace(x_start, x_end, steps)
    
    # Y trajectory: sine wave based on x position
    k = 0.1 * np.pi / wavelength  # Wave number
    y_traj = amplitude * np.sin(k * x_traj)
    
    # Z trajectory: constant height
    z_traj = np.full_like(x_traj, height)
    
    # Orientation: keep level
    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    
    # Calculate velocities
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    
    # Angular velocities (desired rates)
    ax_traj = np.zeros_like(time_space)
    ay_traj = np.zeros_like(time_space)
    az_traj = np.zeros_like(time_space)
    
    sine_wave_traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                               vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj])
    
    land_traj = land_trajectory(dt, sine_wave_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))
    
    return np.concatenate((take_off_traj, move_to_start, sine_wave_traj, land_traj, zeros), axis=1)


def zsine_trajectory(dt):
    """
    Trajectory that performs a slow vertical sine wave motion.
    The drone hovers at a fixed horizontal position while moving up and down
    in a sine wave pattern at a slow frequency.
    """
    take_off_traj = takeoff_trajectory(dt)
    
    # Define vertical sine wave parameters
    base_height = 1.0      # Base altitude (center of sine wave)
    amplitude = 0.5       # Vertical amplitude (±0.8 meters)
    frequency = 0.05       # Slow frequency (Hz) - about 6.7 second period
    duration = 40.0        # Duration of the sine wave motion
    
    # Move to start position
    move_to_start = move_to_start_of_main_trajectory(dt, take_off_traj[:, -1], 
                                                   np.array([0.0, 0.0, base_height]))
    
    # Create vertical sine wave trajectory
    steps = int(duration / dt)
    time_space = np.linspace(0, duration, steps)
    
    # Fixed horizontal position
    x_traj = np.zeros_like(time_space)
    y_traj = np.zeros_like(time_space)
    
    # Vertical sine wave motion
    z_traj = base_height + amplitude * np.sin(2 * np.pi * frequency * time_space)
    
    # Keep orientation level throughout
    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    
    # Convert Euler angles to quaternions
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    
    # Calculate velocities
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    
    # Angular velocities (set to zero for stable hovering)
    ax_traj = np.zeros_like(time_space)
    ay_traj = np.zeros_like(time_space)
    az_traj = np.zeros_like(time_space)
    
    zsine_traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                           vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj])
    
    land_traj = land_trajectory(dt, zsine_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))
    
    return np.concatenate((take_off_traj, move_to_start, zsine_traj, land_traj, zeros), axis=1)


def four_roll_rotations_trajectory(dt):
    """
    Trajectory that hovers, then performs 4 separate 90-degree roll rotations 
    with 3-second hover periods in between each rotation.
    """
    take_off_traj = takeoff_trajectory(dt)
    
    # Parameters
    hover_duration = 3.0  # 3 seconds hover between rotations
    roll_duration = 3.0   # 2 seconds per 90-degree roll rotation
    height = 1.5          # Fixed altitude throughout
    
    hover_steps = int(hover_duration / dt)
    roll_steps = int(roll_duration / dt)
    
    # Initialize lists to store trajectory segments
    x_segments = []
    y_segments = []
    z_segments = []
    roll_segments = []
    pitch_segments = []
    yaw_segments = []
    
    # Initial hover
    hover_time = np.linspace(0, hover_duration, hover_steps)
    x_segments.append(np.zeros_like(hover_time))
    y_segments.append(np.zeros_like(hover_time))
    z_segments.append(np.full_like(hover_time, height))
    roll_segments.append(np.zeros_like(hover_time))
    pitch_segments.append(np.zeros_like(hover_time))
    yaw_segments.append(np.zeros_like(hover_time))
    
    # Perform 4 roll rotations with hovers in between
    current_roll = 0.0
    for i in range(4):
        # Roll rotation (90 degrees = π/2 radians)
        roll_time = np.linspace(0, roll_duration, roll_steps)
        target_roll = current_roll + np.pi/2
        
        x_segments.append(np.zeros_like(roll_time))
        y_segments.append(np.zeros_like(roll_time))
        z_segments.append(np.full_like(roll_time, height))
        roll_segments.append(np.linspace(current_roll, target_roll, roll_steps))
        pitch_segments.append(np.zeros_like(roll_time))
        yaw_segments.append(np.zeros_like(roll_time))
        
        current_roll = target_roll
        
        # Hover after rotation (except after the last rotation)
        if i < 3:  # Only add hover between rotations, not after the last one
            hover_time = np.linspace(0, hover_duration, hover_steps)
            x_segments.append(np.zeros_like(hover_time))
            y_segments.append(np.zeros_like(hover_time))
            z_segments.append(np.full_like(hover_time, height))
            roll_segments.append(np.full_like(hover_time, current_roll))
            pitch_segments.append(np.zeros_like(hover_time))
            yaw_segments.append(np.zeros_like(hover_time))
    
    # Final hover to stabilize
    final_hover_time = np.linspace(0, hover_duration, hover_steps)
    x_segments.append(np.zeros_like(final_hover_time))
    y_segments.append(np.zeros_like(final_hover_time))
    z_segments.append(np.full_like(final_hover_time, height))
    roll_segments.append(np.full_like(final_hover_time, current_roll))
    pitch_segments.append(np.zeros_like(final_hover_time))
    yaw_segments.append(np.zeros_like(final_hover_time))
    
    # Concatenate all segments
    x_traj = np.concatenate(x_segments)
    y_traj = np.concatenate(y_segments)
    z_traj = np.concatenate(z_segments)
    roll_traj = np.concatenate(roll_segments)
    pitch_traj = np.concatenate(pitch_segments)
    yaw_traj = np.concatenate(yaw_segments)
    
    # Convert Euler angles to quaternions
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    
    # Calculate velocities
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    
    # Angular velocities (desired rates) - set to zero for now
    ax_traj = np.zeros_like(x_traj)
    ay_traj = np.zeros_like(y_traj)
    az_traj = np.zeros_like(z_traj)
    
    four_roll_traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                               vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj])
    
    land_traj = land_trajectory(dt, four_roll_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))
    
    return np.concatenate((take_off_traj, four_roll_traj, land_traj, zeros), axis=1)