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
log_file_with_comp = '/home/mitchell/Documents/PhD/drone_cage_control/logs/ExperimentDataSets/Cinewhoop/xyz_xy_adapt_4/log.csv'
log_file_without_comp = '/home/mitchell/Documents/PhD/drone_cage_control/logs/ExperimentDataSets/Cinewhoop/xyz_no_xy_adapt_3/log.csv'

# Create figure with 1 subplot for total tracking error
fig, ax = plt.subplots(1, 1, figsize=(FIGURE_SIZE[0] * 10, FIGURE_SIZE[1]))

# Create separate figure for legend
fig_legend = plt.figure(figsize=(10, 0.5))

# Store handles and labels for legend
legend_handles = []
legend_labels = []

# Function to calculate RMSE
def calculate_rmse(errors):
    """Calculate Root Mean Square Error"""
    return np.sqrt(np.mean(errors**2))

# Dictionary to store RMSE values
rmse_results = {
    'with_adaptation': {'x': 0, 'y': 0, 'z': 0},
    'without_adaptation': {'x': 0, 'y': 0, 'z': 0}
}

try:
    # Read the CSV file with adaptation
    df_with = pd.read_csv(log_file_with_comp)
    
    # Calculate time from timestamp
    time_with = df_with['timestamp'] - df_with['timestamp'].iloc[0]
    
    # Filter data from 13 seconds onwards for error calculation
    mask_with = time_with >= 13
    
    # Extract position data
    x_pos_with = df_with['ukf_pose_x']
    y_pos_with = df_with['ukf_pose_y']
    z_pos_with = df_with['ukf_pose_z']
    
    x_ref_with = df_with['traj_x_ref']
    y_ref_with = df_with['traj_y_ref']
    z_ref_with = df_with['traj_z_ref']
    
    # Calculate tracking errors
    x_error_with = x_pos_with - x_ref_with
    y_error_with = y_pos_with - y_ref_with
    z_error_with = z_pos_with - z_ref_with
    
    # Calculate 3D Euclidean distance error
    total_error_with = np.sqrt(x_error_with**2 + y_error_with**2 + z_error_with**2)
    
    # Calculate RMSE for with adaptation (from 13 seconds onwards)
    rmse_results['with_adaptation']['x'] = calculate_rmse(x_error_with[mask_with])
    rmse_results['with_adaptation']['y'] = calculate_rmse(y_error_with[mask_with])
    rmse_results['with_adaptation']['z'] = calculate_rmse(z_error_with[mask_with])
    
    # Plot total tracking error with adaptation
    line_with, = ax.plot(time_with, total_error_with, color=colors[0], linewidth=LINE_WIDTH, label='With Adaptation')
except FileNotFoundError:
    print(f"Warning: File not found: {log_file_with_comp}")
except Exception as e:
    print(f"Error processing {log_file_with_comp}: {e}")

try:
    # Read the CSV file without adaptation
    df_without = pd.read_csv(log_file_without_comp)
    
    # Calculate time from timestamp
    time_without = df_without['timestamp'] - df_without['timestamp'].iloc[0]
    
    # Filter data from 13 seconds onwards for error calculation
    mask_without = time_without >= 13
    
    # Extract position data
    x_pos_without = df_without['ukf_pose_x']
    y_pos_without = df_without['ukf_pose_y']
    z_pos_without = df_without['ukf_pose_z']
    
    x_ref_without = df_without['traj_x_ref']
    y_ref_without = df_without['traj_y_ref']
    z_ref_without = df_without['traj_z_ref']
    
    # Calculate tracking errors
    x_error_without = x_pos_without - x_ref_without
    y_error_without = y_pos_without - y_ref_without
    z_error_without = z_pos_without - z_ref_without
    
    # Calculate 3D Euclidean distance error
    total_error_without = np.sqrt(x_error_without**2 + y_error_without**2 + z_error_without**2)
    
    # Calculate RMSE for without adaptation (from 13 seconds onwards)
    rmse_results['without_adaptation']['x'] = calculate_rmse(x_error_without[mask_without])
    rmse_results['without_adaptation']['y'] = calculate_rmse(y_error_without[mask_without])
    rmse_results['without_adaptation']['z'] = calculate_rmse(z_error_without[mask_without])
    
    # Plot total tracking error without adaptation
    line_without, = ax.plot(time_without, total_error_without, color=colors[1], linewidth=LINE_WIDTH, label='Without Adaptation')
    
    # Store for legend
    legend_handles = [line_with, line_without]
    legend_labels = ['With Adaptation', 'Without Adaptation']
    
except FileNotFoundError:
    print(f"Warning: File not found: {log_file_without_comp}")
except Exception as e:
    print(f"Error processing {log_file_without_comp}: {e}")

# Configure axis: 3D Tracking Error
ax.set_ylabel('Tracking Error (m)', fontsize=AXIS_LABEL_SIZE)
ax.set_xlabel('Time (s)', fontsize=AXIS_LABEL_SIZE)
ax.grid(True, alpha=GRID_ALPHA)
ax.set_title('3D Position Tracking Error', fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
ax.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)

# Create legend in separate figure
fig_legend.legend(legend_handles, legend_labels, 
                  loc='center', ncol=2, frameon=True, fontsize=LEGEND_SIZE, borderaxespad=0)
fig_legend.tight_layout()

# Adjust main figure layout
fig.tight_layout()

# Print RMSE results
print("\n" + "="*60)
print("RMSE COMPARISON RESULTS")
print("="*60)
print("\nWith Adaptation:")
print(f"  X-axis RMSE: {rmse_results['with_adaptation']['x']:.4f} m")
print(f"  Y-axis RMSE: {rmse_results['with_adaptation']['y']:.4f} m")
print(f"  Z-axis RMSE: {rmse_results['with_adaptation']['z']:.4f} m")
print(f"  Total RMSE:  {np.sqrt(rmse_results['with_adaptation']['x']**2 + rmse_results['with_adaptation']['y']**2 + rmse_results['with_adaptation']['z']**2):.4f} m")

print("\nWithout Adaptation:")
print(f"  X-axis RMSE: {rmse_results['without_adaptation']['x']:.4f} m")
print(f"  Y-axis RMSE: {rmse_results['without_adaptation']['y']:.4f} m")
print(f"  Z-axis RMSE: {rmse_results['without_adaptation']['z']:.4f} m")
print(f"  Total RMSE:  {np.sqrt(rmse_results['without_adaptation']['x']**2 + rmse_results['without_adaptation']['y']**2 + rmse_results['without_adaptation']['z']**2):.4f} m")

print("\nImprovement (%):")
print(f"  X-axis: {(rmse_results['without_adaptation']['x'] - rmse_results['with_adaptation']['x']) / rmse_results['without_adaptation']['x'] * 100:.2f}%")
print(f"  Y-axis: {(rmse_results['without_adaptation']['y'] - rmse_results['with_adaptation']['y']) / rmse_results['without_adaptation']['y'] * 100:.2f}%")
print(f"  Z-axis: {(rmse_results['without_adaptation']['z'] - rmse_results['with_adaptation']['z']) / rmse_results['without_adaptation']['z'] * 100:.2f}%")
print("="*60 + "\n")

# Save figures as PDFs
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
fig.savefig(os.path.join(script_dir, 'tracking_error_comparison.pdf'), format='pdf', bbox_inches='tight')
fig_legend.savefig(os.path.join(script_dir, 'tracking_error_comparison_legend.pdf'), format='pdf', bbox_inches='tight')
print(f"Saved figures to {script_dir}")

# Show both figures
plt.show()
