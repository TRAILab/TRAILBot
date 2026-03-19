#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf_transformations
from tf2_ros import TransformException
from rclpy.duration import Duration

class RobotPosePublisher(Node):
    def __init__(self):
        super().__init__('robot_pose_publisher')

        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose_world', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer to publish pose periodically
        self.create_timer(0.2, self.publish_robot_pose)  # 5 Hz

    def publish_robot_pose(self):
        try:
            # Lookup the transform from world to base_link
            # trans = self.tf_buffer.lookup_transform('world', 'base_link', rclpy.time.Time())
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link', # remember to change it either to base_link or base_footprint
                rclpy.time.Time(),  # time=0 means latest available
                timeout=Duration(seconds=0.1)
            )
            
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = trans.transform.translation.x
            pose_msg.pose.position.y = trans.transform.translation.y
            pose_msg.pose.position.z = trans.transform.translation.z
            pose_msg.pose.orientation = trans.transform.rotation
            self.get_logger().info(f'Published robot pose: x={pose_msg.pose.position.x:.2f}, y={pose_msg.pose.position.y:.2f}')

            self.pose_pub.publish(pose_msg)
        except TransformException as e:
            self.get_logger().warn(f'Could not get transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

