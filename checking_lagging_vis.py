import pandas as pd
import matplotlib.pyplot as plt

# Load your CSV file
df = pd.read_csv('src/TRAILBot/timestamp_diffs_oct2.csv')

# ✅ Plot raw timestamps
plt.figure()
plt.plot(df['lidar_time'], label='LiDAR Time')
plt.plot(df['camera_time'], label='Camera Time')
plt.plot(df['tf_time'], label='TF Time')
plt.title('Timestamps')
plt.xlabel('Message Index')
plt.ylabel('ROS Time (s)')
plt.legend()
plt.grid()

# Save raw timestamp plot
plt.savefig('timestamp_plot.png')
print('Saved: timestamp_plot.png')

# ✅ Plot timestamp differences
plt.figure()
plt.plot(df['lidar-camera'], label='LiDAR - Camera')
plt.plot(df['lidar-tf'], label='LiDAR - TF')
plt.plot(df['camera-tf'], label='Camera - TF')
plt.title('Timestamp Differences')
plt.xlabel('Message Index')
plt.ylabel('Time Difference (s)')
plt.legend()
plt.grid()

# Save difference plot
plt.savefig('timestamp_diff_plot.png')
print('Saved: timestamp_diff_plot.png')

