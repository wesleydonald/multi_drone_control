import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
df = pd.read_csv('/home/mitchell/Documents/PhD/drone_cage_control/logs/controller_angle_ukf/LogsToKeep/hover_inside_house.csv')

# Replace these with your actual column names if different
axis_columns = ['MPC_setup_time', 'MPC_solve_time', 'Visualisation_time', 'UKF_update_time']

# Use a sample index or timestamp for x-axis
if 'timestamp' in df.columns:
    x = df['timestamp']
    xlabel = 'Timestamp'
elif 'sample' in df.columns:
    x = df['sample']
    xlabel = 'Sample'
else:
    x = df.index
    xlabel = 'Sample Index'

plt.figure(figsize=(10, 6))
for axis in axis_columns:
    plt.plot(x, df[axis], label=axis)

plt.xlabel(xlabel)
plt.ylabel('Time Taken (ms)')
plt.title('Controller Run Time on 4 Axes')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()