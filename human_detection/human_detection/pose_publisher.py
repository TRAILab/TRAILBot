import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from tf2_ros.transform_broadcaster import TransformBroadcaster
from rclpy.duration import Duration

class HuskyPosePublisher(Node):
    def __init__(self):
        super().__init__('husky_pose_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(PoseStamped, '/husky_pose', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.target_frame = 'map'
        self.source_frame = 'base_link'

        self.get_logger().info("Husky pose publisher initialized (map -> base_link).")

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                now,
                timeout=Duration(seconds=0.5)
            )

            pose = PoseStamped()
            pose.header.stamp = trans.header.stamp
            pose.header.frame_id = self.target_frame
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation

            self.pose_pub.publish(pose)

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = HuskyPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
