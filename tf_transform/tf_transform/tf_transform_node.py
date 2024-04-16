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
            # Look up the transformations between "map" and "velodyne"
            trans_map = self.tf_buffer.lookup_transform("map", "os_sensor", rclpy.time.Time())
            #the base link is an identity transformation, hence the values are all 0 (not being computed), but am leaving for understanding
            trans_base = self.tf_buffer.lookup_transform("os_sensor", "os_sensor", rclpy.time.Time())  # Identity transformation
            
            translation_map = trans_map.transform.translation
            #rotation_map = trans_map.transform.rotation
            
            translation_base = trans_base.transform.translation
            
            # trans_map is a transformation taking points in the base frame and outputs points in the map frame
            #The translation component of the transform is thus the robots position relative to the map origin (0,0,0)
            #The translation is being defined in the map frame, so setting z = 0 creates a pseduo robot frame at the corresponding xy pos. 
            #The added +0.30 is to allow the pointcloud converter some adjustment room for angles 
            #desired_translation = gm.Vector3(x=translation_map.x, y=translation_map.y, z=(translation_base.z + 0.30))
            desired_translation = gm.Vector3(x=translation_map.x, y=translation_map.y, z= 0.10) #same thing as above 

            #If you want the translation to be at the original location of the map but follow the velodyne's z height then this is the corresponding translation:
            #desired_translation = gm.Vector3(x=translation_base.x, y=translation_base.y, z=translation_map)
            
            new_trans = gm.TransformStamped()
            new_trans.header.stamp = self.get_clock().now().to_msg()
            new_trans.header.frame_id = "map"
            new_trans.child_frame_id = "robot_adjusted"
            new_trans.transform.translation = desired_translation
            #new_trans.transform.rotation = rotation_map
            
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

