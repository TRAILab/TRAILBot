import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


#class to draw in RVIZ
class CircleMarkerNode(Node):
    def __init__(self):
        super().__init__('circle_marker_node')

        self.publisher_ = self.create_publisher(Marker, 'circle_marker', 10)
        self.subscription = self.create_subscription(
            PoseStamped,
            'target_location',
            self.goal_pose_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def goal_pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        marker = self.create_circle_marker(x, y)
        self.publisher_.publish(marker)

    def create_circle_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'velodyne'  #  frame ID of the coordinate system
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0  # Assuming the circle is in the XY plane
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0  # Diameter of the circle
        marker.scale.y = 1.0  # Diameter of the circle
        marker.scale.z = 0.01  # Height of the cylinder (thickness of the circle)
        marker.color = ColorRGBA()
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5  # Semi-transparent

        return marker

def main(args=None):
    rclpy.init(args=args)
    node = CircleMarkerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
