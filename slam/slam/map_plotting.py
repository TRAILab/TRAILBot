import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import pandas as pd
import yaml
import subprocess
import threading
import os

# For converting quaternions to Euler angles:
# Install: pip install tf-transformations
from tf_transformations import euler_from_quaternion

class MapListener(Node):
    def __init__(self):
        super().__init__('map_plotting')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',            # <-- Subscribe to the /map topic
            self.map_callback,
            10
        )

        # Lists to store data
        self.timestamps = []

        # Occupancy fractions
        self.fraction_occupied = []
        self.fraction_free = []
        self.fraction_unknown = []

        # Map origin position/orientation
        self.pos_x = []
        self.pos_y = []
        self.pos_z = []
        self.rolls = []
        self.pitchs = []
        self.yaws = []

    def map_callback(self, msg):
        """
        Each OccupancyGrid message:
          - header (timestamp, frame_id)
          - info (map_load_time, resolution, width, height, origin)
          - data (1D array of int8: -1=unknown, 0=free, 100=occupied, etc.)

        We'll calculate the fraction of occupied, free, unknown,
        and record the map's origin (position + orientation).
        """

        # Convert ROS time to floating-point seconds
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.timestamps.append(timestamp)

        # --- 1) Occupancy fractions ---
        data = msg.data
        total_cells = len(data)
        if total_cells > 0:
            occupied_count = sum(1 for cell in data if cell >= 50)
            free_count = sum(1 for cell in data if cell == 0)
            unknown_count = sum(1 for cell in data if cell == -1)

            frac_occupied = occupied_count / total_cells
            frac_free = free_count / total_cells
            frac_unknown = unknown_count / total_cells
        else:
            frac_occupied = 0
            frac_free = 0
            frac_unknown = 0

        self.fraction_occupied.append(frac_occupied)
        self.fraction_free.append(frac_free)
        self.fraction_unknown.append(frac_unknown)

        # --- 2) Map origin: position & orientation ---
        origin_pose = msg.info.origin
        # Position
        self.pos_x.append(origin_pose.position.x)
        self.pos_y.append(origin_pose.position.y)
        self.pos_z.append(origin_pose.position.z)

        # Orientation (quaternion -> euler)
        qx = origin_pose.orientation.x
        qy = origin_pose.orientation.y
        qz = origin_pose.orientation.z
        qw = origin_pose.orientation.w

        roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])
        self.rolls.append(roll)
        self.pitchs.append(pitch)
        self.yaws.append(yaw)

    def save_data(self):
        """Combine all data into a DataFrame and return it."""
        df = pd.DataFrame({
            'timestamp': self.timestamps,
            'fraction_occupied': self.fraction_occupied,
            'fraction_free': self.fraction_free,
            'fraction_unknown': self.fraction_unknown,
            'pos_x': self.pos_x,
            'pos_y': self.pos_y,
            'pos_z': self.pos_z,
            'roll': self.rolls,
            'pitch': self.pitchs,
            'yaw': self.yaws,
        })
        return df

def run_shell_command(command):
    """Runs a shell command (like ros2 bag play) and waits until it finishes."""
    with open(os.devnull, 'w') as nullfile:
        process = subprocess.Popen(command, shell=True, stdout=nullfile, stderr=subprocess.STDOUT)
        process.communicate()

def main(args=None):
    rclpy.init(args=args)
    node = MapListener()

    
    # Example: modify this command to point to your specific ROS bag file
    command = "ros2 bag play /home/trailbot/bags/frameshift2/"
    

    # Run bag file in a separate thread
    bag_thread = threading.Thread(target=run_shell_command, args=(command,))
    bag_thread.start()

    try:
        # Keep spinning until the bag finishes OR we shut down
        while bag_thread.is_alive() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        # Ensure we shut down the ROS node in all cases
        rclpy.shutdown()

    # Once here, the bag thread is done or ROS was shut down
    data = node.save_data()

    # Save data to a CSV for later analysis
    data.to_csv('map_data.csv', index=False)
    print(data)

    # -------------------
    # Plot the data
    # -------------------
    plt.figure(figsize=(14, 10))

    # --- (A) Occupancy fractions ---
    plt.subplot(3, 1, 1)
    plt.plot(data['timestamp'], data['fraction_occupied'], label='Occupied Fraction')
    plt.plot(data['timestamp'], data['fraction_free'], label='Free Fraction')
    plt.plot(data['timestamp'], data['fraction_unknown'], label='Unknown Fraction')
    plt.title('Map Occupancy Fractions over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Fraction of Cells')
    plt.legend()

    # --- (B) Origin Position (x, y, z) ---
    plt.subplot(3, 1, 2)
    plt.plot(data['timestamp'], data['pos_x'], label='pos_x')
    plt.plot(data['timestamp'], data['pos_y'], label='pos_y')
    plt.plot(data['timestamp'], data['pos_z'], label='pos_z')
    plt.title('Map Origin Position over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.legend()

    # --- (C) Origin Orientation (roll, pitch, yaw) ---
    plt.subplot(3, 1, 3)
    plt.plot(data['timestamp'], data['roll'], label='roll')
    plt.plot(data['timestamp'], data['pitch'], label='pitch')
    plt.plot(data['timestamp'], data['yaw'], label='yaw')
    plt.title('Map Origin Orientation over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')
    plt.legend()

    plt.tight_layout()
    plt.savefig("map_data_plot.png")
    print("Plot saved as map_data_plot.png")

    # If you have a valid display, show the plot
    plt.show()

if __name__ == '__main__':
    print("Starting map plotting node...")
    main()
