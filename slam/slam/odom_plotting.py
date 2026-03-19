import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import matplotlib.pyplot as plt
from tf_transformations import euler_from_quaternion
import pandas as pd
import yaml
import subprocess
import threading
import os

class TFListener(Node):
    def __init__(self):
        super().__init__('odom_plotting')
        print("INIT")
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )
        self.translation_data = []
        self.orientation_data = []
        self.timestamps = []

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                # Extract translation
                translation = transform.transform.translation
                x, y, z = translation.x, translation.y, translation.z

                # Extract rotation as quaternion
                rotation = transform.transform.rotation
                quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)

                # Convert quaternion to Euler angles
                roll, pitch, yaw = euler_from_quaternion(quaternion)

                # Get timestamp
                timestamp = (
                    transform.header.stamp.sec
                    + transform.header.stamp.nanosec * 1e-9
                )

                # Append data
                self.translation_data.append((x, y, z))
                self.orientation_data.append((roll, pitch, yaw))
                self.timestamps.append(timestamp)

    def save_data(self):
        # Combine data into a DataFrame for easy manipulation
        df = pd.DataFrame({
            'timestamp': self.timestamps,
            'x': [t[0] for t in self.translation_data],
            'y': [t[1] for t in self.translation_data],
            'z': [t[2] for t in self.translation_data],
            'roll': [o[0] for o in self.orientation_data],
            'pitch': [o[1] for o in self.orientation_data],
            'yaw': [o[2] for o in self.orientation_data],
        })
        return df

def run_shell_command(command):
    """Runs a shell command (like ros2 bag play) and waits until it finishes."""
    with open(os.devnull, 'w') as nullfile:
        process = subprocess.Popen(command, shell=True, stdout=nullfile, stderr=subprocess.STDOUT)
        process.communicate()

def main(args=None):
    rclpy.init(args=args)
    node = TFListener()

    # Start ros2 bag in a separate thread
    command = "ros2 bag play /home/trailbot/bags/frameshift2/"
    bag_thread = threading.Thread(target=run_shell_command, args=(command,))
    bag_thread.start()

    try:
        # Spin periodically until the bag is done or the node shuts down
        while bag_thread.is_alive() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        # Ensure we shut down the ROS node in all cases
        rclpy.shutdown()

    # Once we're here, the bag thread has finished OR ROS was shutdown
    print("here")

    # Save and plot data
    data = node.save_data()
    #data.to_csv('output.csv', index=False)
    print(data)
    plt.figure(figsize=(10, 8))

    # Translation plots
    plt.subplot(2, 1, 1)
    plt.plot(data['timestamp'], data['x'], label='x')
    plt.plot(data['timestamp'], data['y'], label='y')
    plt.plot(data['timestamp'], data['z'], label='z')
    plt.title('Translation over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Translation (m)')
    plt.legend()

    # Orientation plots
    plt.subplot(2, 1, 2)
    plt.plot(data['timestamp'], data['roll'], label='Roll')
    plt.plot(data['timestamp'], data['pitch'], label='Pitch')
    plt.plot(data['timestamp'], data['yaw'], label='Yaw')
    plt.title('Orientation over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Orientation (rad)')
    plt.legend()

    plt.tight_layout()
    # Save the figure first
    plt.savefig("map_odom.png")
    print("Plot saved")

    # If you have a valid display, show the plot window:
    plt.show()

if __name__ == '__main__':
    print("\n\nDEBUG MODE ON\n\n")
    main()
