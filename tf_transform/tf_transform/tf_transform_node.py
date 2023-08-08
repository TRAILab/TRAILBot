import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg as gm

class TFTransformNode(Node):

    def __init__(self):
        super().__init__('tf_transform')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def transform_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            
            translation = trans.transform.translation
            rotation = trans.transform.rotation
            
            # Adjust the z component of translation
            desired_height = 0.0  # Replace with the desired height
            translation.z = desired_height
            
            new_trans = gm.TransformStamped()
            new_trans.header.stamp = self.get_clock().now().to_msg()
            new_trans.header.frame_id = "map"
            new_trans.child_frame_id = "map_adjusted"
            new_trans.transform.translation = translation
            new_trans.transform.rotation = rotation
            
            self.tf_broadcaster.sendTransform(new_trans)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TFTransformNode()
    try:
        while rclpy.ok():
            node.transform_callback()
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

