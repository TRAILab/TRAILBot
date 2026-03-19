import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class SimplePathPublisher(Node):
    def __init__(self):
        super().__init__('simple_path_publisher')
        self.publisher_ = self.create_publisher(Path, '/path', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for i in range(5):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(i)
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.publisher_.publish(path)
        self.get_logger().info(f"Published path with {len(path.poses)} points.")

def main(args=None):
    rclpy.init(args=args)
    node = SimplePathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
