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
                # Look up the transformations between "map" and "base_link"
            trans_map = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            trans_base = self.tf_buffer.lookup_transform("base_link", "base_link", rclpy.time.Time())  # Identity transformation
            
            translation_map = trans_map.transform.translation
            rotation_map = trans_map.transform.rotation
            
            translation_base = trans_base.transform.translation
            
            # Set the desired translation by keeping the x and y components from "map"
            # and using the z component from "base_link"
            desired_translation = gm.Vector3(x=translation_map.x, y=translation_map.y, z=translation_base.z)
            
            new_trans = gm.TransformStamped()
            new_trans.header.stamp = self.get_clock().now().to_msg()
            new_trans.header.frame_id = "map"
            new_trans.child_frame_id = "map_adjusted"
            new_trans.transform.translation = desired_translation
            new_trans.transform.rotation = rotation_map
            
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

