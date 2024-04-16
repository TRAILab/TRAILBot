import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloud2Listener(Node):
    def __init__(self):
        super().__init__('pointcloud2_listener')
        self.subscription = self.create_subscription(
            PointCloud2,
            # 'velodyne_points',  # Change this to the topic you're subscribing to
            '/ouster/points',
            self.listener_callback,
            10)
        self.subscription
        self.msgCount = 0

    def listener_callback(self, msg):
        # Deserialize PointCloud2 data into a generator of (x, y, z) points
        timestamp = msg.header.stamp
        self.msgCount = self.msgCount+1
        print("Received PointCloud2 message at time", timestamp)
        point_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        points = []
        for point in point_gen:
            x, y, z = point
            points.append([x, y, z])

        print(f"Received {len(points)} 3D points:")
        filename = f"points_{self.msgCount}.txt"
        with open(filename, "w") as output_file:
            # output_file.write(f"Timestamp: {timestamp}\n")
            for idx, point in enumerate(points):
                print(f"{idx + 1}: {point}")
                output_file.write(f"{point[0]} {point[1]} {point[2]}\n")
        print("Total message:", self.msgCount)

def main(args=None):
    rclpy.init(args=args)

    pointcloud2_listener = PointCloud2Listener()

    rclpy.spin(pointcloud2_listener)

    pointcloud2_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
