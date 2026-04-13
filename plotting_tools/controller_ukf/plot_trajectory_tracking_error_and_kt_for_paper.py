


import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, FFMpegWriter
from mpl_toolkits.mplot3d import Axes3D

# Plot settings
FIGURE_SIZE = (1, 3)
MAIN_TITLE_SIZE = 15
SUBPLOT_TITLE_SIZE = 16
AXIS_LABEL_SIZE = 15
TICK_LABEL_SIZE = 15
LEGEND_SIZE = 16
LINE_WIDTH = 2
GRID_ALPHA = 0.3

# Define the color palette
colors = ['#e97d00', '#008e00', '#0049bd', '#911515', '#000000', '#97c6d1', '#b697ff']

# 3D Plot configuration
CAMERA_ELEVATION = 25  # Pitch: angle above the horizontal plane (-90 to 90)
CAMERA_AZIMUTH = 45    # Yaw: rotation around the vertical axis (0 to 360)
CAMERA_ROLL = 0        # Roll: rotation around the viewing axis
START_TIME_3D = 35.0   # Start time for 3D plotting (seconds)
END_TIME_3D = 70.0     # End time for 3D plotting (seconds)

# Define the log file paths

DURATION = 70
OUTPUT_PREFIX = 'ukf_circle' 
log_files = [
    '/home/mitchell/Documents/PhD/drone_cage_control/logs/ExperimentDataSets/Cinewhoop/circle_video/log.csv',
    '/home/mitchell/Documents/PhD/drone_cage_control/logs/ExperimentDataSets/Freestyle/circle_3/log.csv',
    '/home/mitchell/Documents/PhD/drone_cage_control/logs/ExperimentDataSets/TinyWhoop/circle_3/log.csv',
    '/home/mitchell/Documents/PhD/drone_cage_control/logs/ExperimentDataSets/TinyTrainer/circle_2_video/log.csv',
]


# Labels for the legend
labels = [ 'Cinewhoop', 'Freestyle', 'Tinywhoop', 'Tinytrainer']

# Create figure with 3 subplots (1 3D plot + 2 2D plots)
fig = plt.figure(figsize=(FIGURE_SIZE[0] * 10, FIGURE_SIZE[1] * 4))
gs_main = fig.add_gridspec(2, 1, hspace=0.2, height_ratios=[1.6, 1])

# Create nested gridspec for 3D plot (centered and smaller width)
gs_3d = gs_main[0].subgridspec(1, 3, wspace=0.1, width_ratios=[0.1, 0.8, 0.1])
ax_3d = fig.add_subplot(gs_3d[1], projection='3d')

# Create nested gridspec for 2D plots (centered and narrower)
gs_2d_container = gs_main[1].subgridspec(1, 3, wspace=0.1, width_ratios=[0.1, 0.8, 0.1])
gs_2d = gs_2d_container[1].subgridspec(2, 1, hspace=1.0)
axes = [fig.add_subplot(gs_2d[0]), fig.add_subplot(gs_2d[1])]

# ============================================================================
# PLOT 3D TRAJECTORIES
# ============================================================================

# Track global bounds for axis scaling
all_x, all_y, all_z = [], [], []

# Flag to plot desired trajectory only once
desired_plotted = False

print("Plotting 3D trajectories...")
for idx, (log_file, label) in enumerate(zip(log_files, labels)):
    try:
        # Read the CSV file
        df = pd.read_csv(log_file)
        
        # Calculate time from timestamp
        time = df['timestamp'] - df['timestamp'].iloc[0]
        
        # Filter data based on time duration for 3D plot
        mask = (time >= START_TIME_3D) & (time <= END_TIME_3D)
        time_filtered = time[mask]
        
        if len(time_filtered) == 0:
            print(f"Warning: No 3D data in time range for {label}")
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
        ax_3d.plot(x_actual, y_actual, z_actual, 
                   color=color, linewidth=LINE_WIDTH, 
                   label=f'{label}', linestyle='-')
        
        # Plot desired trajectory only once (black dashed line)
        if not desired_plotted:
            ax_3d.plot(x_desired, y_desired, z_desired, 
                       color='black', linewidth=LINE_WIDTH*0.7, 
                       label='Reference Trajectory', linestyle='--', alpha=0.8)
            desired_plotted = True
        
        # Collect data for axis bounds
        all_x.extend(x_actual)
        all_x.extend(x_desired)
        all_y.extend(y_actual)
        all_y.extend(y_desired)
        all_z.extend(z_actual)
        all_z.extend(z_desired)
        
        print(f"  Plotted {label} 3D: {len(time_filtered)} points from t={time_filtered.iloc[0]:.2f}s to t={time_filtered.iloc[-1]:.2f}s")
        
    except FileNotFoundError:
        print(f"Warning: File not found: {log_file}")
    except KeyError as e:
        print(f"Warning: Missing column in {log_file}: {e}")
    except Exception as e:
        print(f"Error processing {log_file} for 3D plot: {e}")

# Set equal aspect ratio for all axes
if all_x and all_y and all_z:
    max_range = np.array([max(all_x) - min(all_x), 
                          max(all_y) - min(all_y), 
                          max(all_z) - min(all_z)]).max() / 2.0
    
    mid_x = (max(all_x) + min(all_x)) * 0.5
    mid_y = (max(all_y) + min(all_y)) * 0.5
    mid_z = (max(all_z) + min(all_z)) * 0.5
    
    ax_3d.set_xlim(mid_x - max_range, mid_x + max_range)
    ax_3d.set_ylim(mid_y - max_range, mid_y + max_range)
    ax_3d.set_zlim(mid_z - max_range, mid_z + max_range)

# Set camera view angles
ax_3d.view_init(elev=CAMERA_ELEVATION, azim=CAMERA_AZIMUTH, roll=CAMERA_ROLL)

# Labels and title for 3D plot
ax_3d.set_xlabel('X Position (m)', fontsize=AXIS_LABEL_SIZE, labelpad=15)
ax_3d.set_ylabel('Y Position (m)', fontsize=AXIS_LABEL_SIZE, labelpad=15)
ax_3d.set_zlabel('Z Position (m)', fontsize=AXIS_LABEL_SIZE, labelpad=15)
ax_3d.set_title(f'(a) 3D Pose Trajectories', fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold', y=0.98)
ax_3d.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
ax_3d.grid(True, alpha=GRID_ALPHA)

# ============================================================================
# PLOT 2D TRACKING ERROR AND THRUST ESTIMATES
# ============================================================================

# Create separate figure for legend
fig_legend = plt.figure(figsize=(10, 0.5))

# Store handles and labels for legend
legend_handles = []
legend_labels = []

print("Plotting tracking error and thrust estimates...")
# Read and plot data from each log file
for idx, (log_file, label) in enumerate(zip(log_files, labels)):
    try:
        # Read the CSV file
        df = pd.read_csv(log_file)
        
        # Calculate time from timestamp
        time = df['timestamp'] - df['timestamp'].iloc[0]
        
        # Filter data based on DURATION
        mask = time <= DURATION
        time = time[mask]
        
        # Extract data
        x_actual = df['ukf_pose_x'][mask].values
        y_actual = df['ukf_pose_y'][mask].values
        z_actual = df['ukf_pose_z'][mask].values
        
        x_desired = df['traj_x_ref'][mask].values
        y_desired = df['traj_y_ref'][mask].values
        z_desired = df['traj_z_ref'][mask].values
        
        # Calculate 3D Euclidean tracking error
        tracking_error_3d = np.sqrt(
            (x_actual - x_desired)**2 + 
            (y_actual - y_desired)**2 + 
            (z_actual - z_desired)**2
        )
        
        thrust_estimate = df['est_param_thrust_ratio'][mask]
        
        color = colors[idx]
        
        # Plot 1: 3D tracking error
        line1, = axes[0].plot(time, tracking_error_3d, color=color, linewidth=LINE_WIDTH, label=label)
        
        # Plot 2: Estimated thrust value
        axes[1].plot(time, thrust_estimate, color=color, linewidth=LINE_WIDTH, label=label)
        
        # Store for legend
        legend_handles.append(line1)
        legend_labels.append(label)
        
    except FileNotFoundError:
        print(f"Warning: File not found: {log_file}")
    except Exception as e:
        print(f"Error processing {log_file}: {e}")

# Add desired trajectory to legend
from matplotlib.lines import Line2D
desired_line = Line2D([0], [0], color='black', linewidth=LINE_WIDTH*0.7, linestyle='--', alpha=0.8)
legend_handles.append(desired_line)
legend_labels.append('Reference trajectory')

# Configure axis 0: 3D Tracking Error
axes[0].set_ylabel('Error (m)', fontsize=AXIS_LABEL_SIZE)
axes[0].grid(True, alpha=GRID_ALPHA)
axes[0].set_title('(b) 3D Position Tracking Error', fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
axes[0].set_xlim(0, DURATION)
axes[0].set_xticks(np.arange(0, DURATION + 10, 20))
axes[0].tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)

# Configure axis 1: Thrust Estimate
axes[1].set_ylabel(r'Throttle gain $\hat{k}_t$', fontsize=AXIS_LABEL_SIZE)
axes[1].set_xlabel('Time (s)', fontsize=AXIS_LABEL_SIZE)
axes[1].grid(True, alpha=GRID_ALPHA)
axes[1].set_title(r'(c) Estimated Throttle Gain $\hat{k}_t$', fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
axes[1].set_xlim(0, DURATION)
axes[1].set_xticks(np.arange(0, DURATION + 10, 20))
axes[1].tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)

# Create legend in separate figure
all_handles = legend_handles
all_labels = legend_labels

fig_legend.legend(all_handles, all_labels, 
                  loc='center', ncol=3, frameon=True, fontsize=LEGEND_SIZE, borderaxespad=0)
fig_legend.tight_layout()

# Adjust main figure layout
fig.tight_layout()

# Save figures as PDFs
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
fig.savefig(os.path.join(script_dir, 'trajectory_tracking_comparison.pdf'), format='pdf', bbox_inches='tight')
fig_legend.savefig(os.path.join(script_dir, 'trajectory_tracking_comparison_legend.pdf'), format='pdf', bbox_inches='tight')
print(f"Saved figures to {script_dir}")

# Show both figures
plt.show()