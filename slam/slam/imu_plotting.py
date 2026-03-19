import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import pandas as pd
import yaml
import subprocess
import threading
import os
from tf_transformations import euler_from_quaternion

class ImuListener(Node):
    def __init__(self):
        super().__init__('imu_plotting')
        self.subscription = self.create_subscription(
            Imu,
            '/ouster/imu',   # <-- Subscribe to the /ouster/imu topic
            self.imu_callback,
            10
        )
        self.timestamps = []
        self.linear_accels = []
        self.angular_vels = []
        self.euler_orientations = []

    def imu_callback(self, msg):
        # Extract linear acceleration
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z
        
        # Extract angular velocity
        ang_x = msg.angular_velocity.x
        ang_y = msg.angular_velocity.y
        ang_z = msg.angular_velocity.z

        # Extract orientation (quaternion) and convert to Euler angles
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])

        # Convert ROS time to floating-point seconds
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Save data
        self.timestamps.append(timestamp)
        self.linear_accels.append((acc_x, acc_y, acc_z))
        self.angular_vels.append((ang_x, ang_y, ang_z))
        self.euler_orientations.append((roll, pitch, yaw))

    def save_data(self):
        """Combine all data into a DataFrame and return it."""
        df = pd.DataFrame({
            'timestamp': self.timestamps,
            'acc_x': [acc[0] for acc in self.linear_accels],
            'acc_y': [acc[1] for acc in self.linear_accels],
            'acc_z': [acc[2] for acc in self.linear_accels],
            'ang_x': [ang[0] for ang in self.angular_vels],
            'ang_y': [ang[1] for ang in self.angular_vels],
            'ang_z': [ang[2] for ang in self.angular_vels],
            'roll': [e[0] for e in self.euler_orientations],
            'pitch': [e[1] for e in self.euler_orientations],
            'yaw': [e[2] for e in self.euler_orientations],
        })
        return df

def run_shell_command(command):
    """Runs a shell command (like ros2 bag play) and waits until it finishes."""
    with open(os.devnull, 'w') as nullfile:
        process = subprocess.Popen(command, shell=True, stdout=nullfile, stderr=subprocess.STDOUT)
        process.communicate()

def main(args=None):
    rclpy.init(args=args)
    node = ImuListener()

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

    # Once we're here, the bag thread is done, or ROS was shut down
    data = node.save_data()
    data.to_csv('imu_data.csv', index=False)
    print(data)

    # Now plot the data
    plt.figure(figsize=(12, 10))

    # --- 1) Linear Accelerations ---
    plt.subplot(3, 1, 1)
    plt.plot(data['timestamp'], data['acc_x'], label='acc_x')
    plt.plot(data['timestamp'], data['acc_y'], label='acc_y')
    plt.plot(data['timestamp'], data['acc_z'], label='acc_z')
    plt.title('Linear Acceleration over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s^2)')
    plt.legend()

    # --- 2) Angular Velocities ---
    plt.subplot(3, 1, 2)
    plt.plot(data['timestamp'], data['ang_x'], label='ang_x')
    plt.plot(data['timestamp'], data['ang_y'], label='ang_y')
    plt.plot(data['timestamp'], data['ang_z'], label='ang_z')
    plt.title('Angular Velocity over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend()

    # --- 3) Orientation (Euler) ---
    plt.subplot(3, 1, 3)
    plt.plot(data['timestamp'], data['roll'], label='roll')
    plt.plot(data['timestamp'], data['pitch'], label='pitch')
    plt.plot(data['timestamp'], data['yaw'], label='yaw')
    plt.title('Orientation (Euler angles) over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')
    plt.legend()

    plt.tight_layout()
    plt.savefig("ouster_imu_plot.png")
    print("Plot saved as ouster_imu_plot.png")

    # If you have a valid display, show the plot
    plt.show()

if __name__ == '__main__':
    print("Starting IMU plotting node...")
    main()
