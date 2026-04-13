import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ============================================================================
# USER CONFIGURATION
# ============================================================================

# Camera view settings (in degrees)
CAMERA_ELEVATION = 45  # Pitch: angle above the horizontal plane (-90 to 90)
CAMERA_AZIMUTH = 45    # Yaw: rotation around the vertical axis (0 to 360)
CAMERA_ROLL = 0        # Roll: rotation around the viewing axis

# Time duration settings (in seconds)
START_TIME = 35.0       # Start time for plotting (seconds)
END_TIME = 70.0        # End time for plotting (seconds)

# Log file paths
log_files = [
    '/home/mitchell/Documents/PhD/drone_cage_control/logs/ExperimentDataSets/Cinewhoop/circle_video/log.csv',
    '/home/mitchell/Documents/PhD/drone_cage_control/logs/ExperimentDataSets/Freestyle/circle_1/log.csv',
    '/home/mitchell/Documents/PhD/drone_cage_control/logs/ExperimentDataSets/TinyWhoop/circle_1/log.csv',
    '/home/mitchell/Documents/PhD/drone_cage_control/logs/ExperimentDataSets/TinyTrainer/circle_1/log.csv',
]



# Labels and colors
labels = ['Cinewhoop', 'Freestyle', 'Tinywhoop', 'Tinytrainer']
colors = ['#e97d00', '#008e00', '#0049bd', '#911515']

# Plot settings
FIGURE_SIZE = (14, 10)
LINE_WIDTH = 2
POINT_SIZE = 100
AXIS_LABEL_SIZE = 14
TITLE_SIZE = 16
LEGEND_SIZE = 12
GRID_ALPHA = 0.3

# ============================================================================
# MAIN PLOTTING CODE
# ============================================================================

fig = plt.figure(figsize=FIGURE_SIZE)
ax = fig.add_subplot(111, projection='3d')

# Track global bounds for axis scaling
all_x, all_y, all_z = [], [], []

# Flag to plot desired trajectory only once
desired_plotted = False

# Read and plot data from each log file
for idx, (log_file, label) in enumerate(zip(log_files, labels)):
    try:
        # Read the CSV file
        df = pd.read_csv(log_file)
        
        # Calculate time from timestamp
        time = df['timestamp'] - df['timestamp'].iloc[0]
        
        # Filter data based on time duration
        mask = (time >= START_TIME) & (time <= END_TIME)
        time_filtered = time[mask]
        
        if len(time_filtered) == 0:
            print(f"Warning: No data in time range for {label}")
            continue
        
        # Extract actual pose trajectory
        x_actual = df['ukf_pose_x'][mask].values
        y_actual = df['ukf_pose_y'][mask].values
        z_actual = df['ukf_pose_z'][mask].values
        
        # Extract desired trajectory
        x_desired = df['traj_x_ref'][mask].values
        y_desired = df['traj_y_ref'][mask].values
        z_desired = df['traj_z_ref'][mask].values
        
        color = colors[idx]
        
        # Plot actual trajectory (solid line)
        ax.plot(x_actual, y_actual, z_actual, 
                color=color, linewidth=LINE_WIDTH, 
                label=f'{label}', linestyle='-')
        
        # Plot desired trajectory only once (black dashed line)
        if not desired_plotted:
            ax.plot(x_desired, y_desired, z_desired, 
                    color='black', linewidth=LINE_WIDTH*0.7, 
                    label='Desired', linestyle='--', alpha=0.8)
            desired_plotted = True
        
        # Collect data for axis bounds
        all_x.extend(x_actual)
        all_x.extend(x_desired)
        all_y.extend(y_actual)
        all_y.extend(y_desired)
        all_z.extend(z_actual)
        all_z.extend(z_desired)
        
        print(f"Plotted {label}: {len(time_filtered)} points from t={time_filtered.iloc[0]:.2f}s to t={time_filtered.iloc[-1]:.2f}s")
        
    except FileNotFoundError:
        print(f"Warning: File not found: {log_file}")
    except KeyError as e:
        print(f"Warning: Missing column in {log_file}: {e}")
    except Exception as e:
        print(f"Error processing {log_file}: {e}")

# Set equal aspect ratio for all axes
if all_x and all_y and all_z:
    max_range = np.array([max(all_x) - min(all_x), 
                          max(all_y) - min(all_y), 
                          max(all_z) - min(all_z)]).max() / 2.0
    
    mid_x = (max(all_x) + min(all_x)) * 0.5
    mid_y = (max(all_y) + min(all_y)) * 0.5
    mid_z = (max(all_z) + min(all_z)) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

# Set camera view angles
ax.view_init(elev=CAMERA_ELEVATION, azim=CAMERA_AZIMUTH, roll=CAMERA_ROLL)

# Labels and title
ax.set_xlabel('X Position (m)', fontsize=AXIS_LABEL_SIZE)
ax.set_ylabel('Y Position (m)', fontsize=AXIS_LABEL_SIZE)
ax.set_zlabel('Z Position (m)', fontsize=AXIS_LABEL_SIZE)
ax.set_title(f'3D Pose Trajectories (t={START_TIME:.1f}s to {END_TIME:.1f}s)\n'
             f'Camera: Elev={CAMERA_ELEVATION}°, Azim={CAMERA_AZIMUTH}°, Roll={CAMERA_ROLL}°',
             fontsize=TITLE_SIZE, fontweight='bold')

# Legend
ax.legend(fontsize=LEGEND_SIZE, loc='upper left', framealpha=0.9)
ax.grid(True, alpha=GRID_ALPHA)

# Save figure
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
output_path = os.path.join(script_dir, '3d_trajectory_plot.pdf')
fig.savefig(output_path, format='pdf', bbox_inches='tight', dpi=300)
print(f"\nSaved figure to {output_path}")

plt.show()
