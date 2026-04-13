import pandas as pd
import matplotlib.pyplot as plt

# Load data
df = pd.read_csv('/home/mitchell/Documents/PhD/drone_cage_control/logs/controller_angle_ukf/LogsToKeep/hover_inside_house.csv')

# Replace 'timestamp' with your actual column name
df['second'] = df['timestamp'].astype(int)

# Count samples per second
hz_per_second = df.groupby('second').size()

# Plot
plt.figure(figsize=(10,5))
plt.plot(hz_per_second.index, hz_per_second.values, marker='o')
plt.xlabel('Time (s)')
plt.ylabel('Samples per second (Hz)')
plt.title('Controller Frequency Over Time')
plt.grid(True)
plt.show()

# Print average Hz
print(f"Average Hz: {hz_per_second.mean():.2f}")