import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Plot settings
FIGURE_SIZE = (1, 3)
MAIN_TITLE_SIZE = 24
SUBPLOT_TITLE_SIZE = 26
AXIS_LABEL_SIZE = 24
TICK_LABEL_SIZE = 24
LEGEND_SIZE = 22
LINE_WIDTH = 2
GRID_ALPHA = 0.3

# Define the color palette
colors = ['#e97d00', '#008e00', '#0049bd', '#911515', '#000000', '#97c6d1', '#b697ff']

# Define the log file paths
log_file_with_comp = '/home/mitchell/Documents/PhD/drone_cage_control/logs/controller_angle_ukf/32_with_delay_comp/log.csv'
log_file_without_comp = '/home/mitchell/Documents/PhD/drone_cage_control/logs/controller_angle_ukf/32_without_delay_comp/log.csv'



#log_file_with_comp = '/home/mitchell/Documents/PhD/drone_cage_control/logs/ExperimentDataSets/CinewhoopCircleTimedelay1?/log.csv'
#log_file_without_comp = '/home/mitchell/Documents/PhD/drone_cage_control/logs/ExperimentDataSets/CinewhoopCircleTimedelay2?/log.csv'
#log_file_with_comp = '/home/mitchell/Documents/PhD/drone_cage_control/logs/ExperimentDataSets/CinewhoopCircleTimedelay3?/log.csv'
#log_file_without_comp = '/home/mitchell/Documents/PhD/drone_cage_control/logs/ExperimentDataSets/CinewhoopCircleTimedelay4?/log.csv'

# Create figure with 4 subplots (x, y, z, throttle)
fig, axes = plt.subplots(4, 1, figsize=(FIGURE_SIZE[0] * 10, FIGURE_SIZE[1] * 4))
fig.subplots_adjust(hspace=0.3)

# Create separate figure for legend
fig_legend = plt.figure(figsize=(10, 0.5))

# Store handles and labels for legend
legend_handles = []
legend_labels = []

try:
    # Read both CSV files
    df_with = pd.read_csv(log_file_with_comp)
    df_without = pd.read_csv(log_file_without_comp)
    
    print(f"Data loaded successfully:")
    print(f"With comp: {len(df_with)} rows")
    print(f"Without comp: {len(df_without)} rows")
    print(f"Available columns: {df_with.columns.tolist()}")
    
    # Calculate time from timestamp for both (use entire timespan)
    time_with = df_with['timestamp'] - df_with['timestamp'].iloc[0]
    time_without = df_without['timestamp'] - df_without['timestamp'].iloc[0]
    
    # Extract data with delay compensation
    x_pos_with = df_with['ukf_pose_x']
    y_pos_with = df_with['ukf_pose_y']
    z_pos_with = df_with['ukf_pose_z']
    x_des_with = df_with['traj_x_ref']
    y_des_with = df_with['traj_y_ref']
    z_des_with = df_with['traj_z_ref']
    throttle_with = df_with['est_param_thrust_ratio']
    
    # Extract data without delay compensation
    x_pos_without = df_without['ukf_pose_x']
    y_pos_without = df_without['ukf_pose_y']
    z_pos_without = df_without['ukf_pose_z']
    x_des_without = df_without['traj_x_ref']
    y_des_without = df_without['traj_y_ref']
    z_des_without = df_without['traj_z_ref']
    throttle_without = df_without['est_param_thrust_ratio']
    
    # Plot 0: X position tracking
    line_x_with, = axes[0].plot(time_with, x_pos_with, color=colors[0], linewidth=LINE_WIDTH, label='X Actual (With Comp)')
    line_x_des_with, = axes[0].plot(time_with, x_des_with, color=colors[0], linewidth=LINE_WIDTH, linestyle='--', label='X Desired (With Comp)')
    line_x_without, = axes[0].plot(time_without, x_pos_without, color=colors[1], linewidth=LINE_WIDTH, label='X Actual (Without Comp)')
    line_x_des_without, = axes[0].plot(time_without, x_des_without, color=colors[1], linewidth=LINE_WIDTH, linestyle='--', label='X Desired (Without Comp)')
    
    # Plot 1: Y position tracking
    line_y_with, = axes[1].plot(time_with, y_pos_with, color=colors[2], linewidth=LINE_WIDTH, label='Y Actual (With Comp)')
    line_y_des_with, = axes[1].plot(time_with, y_des_with, color=colors[2], linewidth=LINE_WIDTH, linestyle='--', label='Y Desired (With Comp)')
    line_y_without, = axes[1].plot(time_without, y_pos_without, color=colors[3], linewidth=LINE_WIDTH, label='Y Actual (Without Comp)')
    line_y_des_without, = axes[1].plot(time_without, y_des_without, color=colors[3], linewidth=LINE_WIDTH, linestyle='--', label='Y Desired (Without Comp)')
    
    # Plot 2: Z position tracking
    line_z_with, = axes[2].plot(time_with, z_pos_with, color=colors[4], linewidth=LINE_WIDTH, label='Z Actual (With Comp)')
    line_z_des_with, = axes[2].plot(time_with, z_des_with, color=colors[4], linewidth=LINE_WIDTH, linestyle='--', label='Z Desired (With Comp)')
    line_z_without, = axes[2].plot(time_without, z_pos_without, color=colors[5], linewidth=LINE_WIDTH, label='Z Actual (Without Comp)')
    line_z_des_without, = axes[2].plot(time_without, z_des_without, color=colors[5], linewidth=LINE_WIDTH, linestyle='--', label='Z Desired (Without Comp)')
    
    # Plot 3: Throttle estimate
    line_throttle_with, = axes[3].plot(time_with, throttle_with, color=colors[0], linewidth=LINE_WIDTH, label='Throttle (With Comp)')
    line_throttle_without, = axes[3].plot(time_without, throttle_without, color=colors[1], linewidth=LINE_WIDTH, label='Throttle (Without Comp)')
    
    # Store for legend - reorder for 4 columns, 4 rows
    # Desired layout (row-first):
    # Row 1: X Actual (With), X Desired (With), X Actual (Without), X Desired (Without)
    # Row 2: Y Actual (With), Y Desired (With), Y Actual (Without), Y Desired (Without)
    # Row 3: Z Actual (With), Z Desired (With), Z Actual (Without), Z Desired (Without)
    # Row 4: Throttle (With), [empty], Throttle (Without), [empty]
    # matplotlib fills columns first, so reorder to achieve this
    legend_handles = [
        line_x_with, line_y_with, line_z_with, line_throttle_with,
        line_x_des_with, line_y_des_with, line_z_des_with,
        line_x_without, line_y_without, line_z_without, line_throttle_without,
        line_x_des_without, line_y_des_without, line_z_des_without
    ]
    legend_labels = [
        'X Actual (With Comp)', 'Y Actual (With Comp)', 'Z Actual (With Comp)', 'Throttle (With Comp)',
        'X Desired (With Comp)', 'Y Desired (With Comp)', 'Z Desired (With Comp)',
        'X Actual (Without Comp)', 'Y Actual (Without Comp)', 'Z Actual (Without Comp)', 'Throttle (Without Comp)',
        'X Desired (Without Comp)', 'Y Desired (Without Comp)', 'Z Desired (Without Comp)'
    ]
    
except FileNotFoundError as e:
    print(f"Warning: File not found: {e}")
except Exception as e:
    print(f"Error processing files: {e}")

# Configure axis 0: X Position
axes[0].set_ylabel('X Position (m)', fontsize=AXIS_LABEL_SIZE)
axes[0].set_xlabel('Time (s)', fontsize=AXIS_LABEL_SIZE)
axes[0].grid(True, alpha=GRID_ALPHA)
axes[0].set_title('(a) X Position Tracking', fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
axes[0].tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)

# Configure axis 1: Y Position
axes[1].set_ylabel('Y Position (m)', fontsize=AXIS_LABEL_SIZE)
axes[1].set_xlabel('Time (s)', fontsize=AXIS_LABEL_SIZE)
axes[1].grid(True, alpha=GRID_ALPHA)
axes[1].set_title('(b) Y Position Tracking', fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
axes[1].tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)

# Configure axis 2: Z Position
axes[2].set_ylabel('Z Position (m)', fontsize=AXIS_LABEL_SIZE)
axes[2].set_xlabel('Time (s)', fontsize=AXIS_LABEL_SIZE)
axes[2].grid(True, alpha=GRID_ALPHA)
axes[2].set_title('(c) Z Position Tracking', fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
axes[2].tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)

# Configure axis 3: Throttle Estimate
axes[3].set_ylabel('Throttle Estimate', fontsize=AXIS_LABEL_SIZE)
axes[3].set_xlabel('Time (s)', fontsize=AXIS_LABEL_SIZE)
axes[3].grid(True, alpha=GRID_ALPHA)
axes[3].set_title('(d) Throttle Estimate', fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
axes[3].tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)

# Create legend in separate figure with 4 columns
fig_legend.legend(legend_handles, legend_labels, 
                  loc='center', ncol=4, frameon=True, fontsize=LEGEND_SIZE, borderaxespad=0)
fig_legend.tight_layout()

# Adjust main figure layout
fig.tight_layout()

# Save figures as PDFs
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
fig.savefig(os.path.join(script_dir, 'delay_compensation_comparison.pdf'), format='pdf', bbox_inches='tight')
fig_legend.savefig(os.path.join(script_dir, 'delay_compensation_comparison_legend.pdf'), format='pdf', bbox_inches='tight')
print(f"Saved figures to {script_dir}")

# Show both figures
plt.show()
