import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg as gm
from rclpy.parameter import Parameter

class MapToOdomBroadcaster(Node):
    def __init__(self):
        super().__init__(
        'world_tf_publisher',
        parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, False)
        ]
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # Broadcast at a fixed rate (10 Hz in this example).
        self.timer_period = 0.1  
        self.timer = self.create_timer(self.timer_period, self.broadcast_transform)
    
    def broadcast_transform(self):
        # Create the TransformStamped message
        t = gm.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'   # parent frame
        t.child_frame_id = 'odom'   # child frame

        # If you just want an identity transform, set translation to 0
        # and rotation to a unit quaternion (w=1, x=y=z=0).
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('✅ Initial transform set for map -> odom')

def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import tf2_ros
# import tf_transformations
# from geometry_msgs.msg import TransformStamped
# import time
# from rclpy.parameter import Parameter

# class WorldTFPublisher(Node):
#     def __init__(self):
#         super().__init__(
#             'world_tf_publisher',
#             parameter_overrides=[
#                 Parameter('use_sim_time', Parameter.Type.BOOL, True)
#             ]
#         )

#         # TF buffer to store transform data
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         # Broadcaster to publish the world -> odom transform
#         self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

#         # Flag to check whether the initial origin transform has been set
#         self.origin_set = False
#         self.initial_transform = None

#         # Timer to periodically attempt to get the initial transform
#         self.create_timer(0.5, self.timer_callback)

#     def timer_callback(self):
#         if not self.origin_set:
#             try:
#                 # Get the current transform from odom to base_link
#                 # trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
#                 trans = self.tf_buffer.lookup_transform(
#                 'odom',
#                 'base_link',
#                 rclpy.time.Time(), #self.get_clock().now().to_msg(), 
#                 timeout=rclpy.duration.Duration(seconds=0.5)
# )

#                 # Invert the transform (base_link -> odom) to define world -> odom
#                 t = trans.transform.translation
#                 q = trans.transform.rotation

#                 # Convert to transformation matrix and compute its inverse
#                 T = tf_transformations.concatenate_matrices(
#                     tf_transformations.translation_matrix([t.x, t.y, t.z]),
#                     tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
#                 )
#                 T_inv = tf_transformations.inverse_matrix(T)

#                 # Extract translation and rotation from the inverted matrix
#                 trans_vec = tf_transformations.translation_from_matrix(T_inv)
#                 rot_quat = tf_transformations.quaternion_from_matrix(T_inv)

#                 # Prepare the TransformStamped message for world -> odom
#                 self.initial_transform = TransformStamped()
#                 self.initial_transform.header.frame_id = 'world'
#                 self.initial_transform.child_frame_id = 'odom'
#                 self.initial_transform.transform.translation.x = trans_vec[0]
#                 self.initial_transform.transform.translation.y = trans_vec[1]
#                 self.initial_transform.transform.translation.z = trans_vec[2]
#                 self.initial_transform.transform.rotation.x = rot_quat[0]
#                 self.initial_transform.transform.rotation.y = rot_quat[1]
#                 self.initial_transform.transform.rotation.z = rot_quat[2]
#                 self.initial_transform.transform.rotation.w = rot_quat[3]

#                 self.origin_set = True
#                 self.get_logger().info('✅ Initial transform set for world -> odom')

#             except Exception as e:
#                 self.get_logger().warn(f'Waiting for transform: {str(e)}')
#         else:
#             # Update timestamp and publish the static transform
#             self.initial_transform.header.stamp = self.get_clock().now().to_msg()
#             self.tf_broadcaster.sendTransform(self.initial_transform)

# def main(args=None):
#     rclpy.init(args=args)
#     node = WorldTFPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# !/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.parameter import Parameter
# import tf2_ros
# import tf_transformations
# from geometry_msgs.msg import TransformStamped
# from rclpy.duration import Duration
# from tf2_ros import TransformException

# class WorldTFPublisher(Node):
#     def __init__(self):
#         # Initialize the node and enable simulation time
#         super().__init__(
#             'world_tf_publisher',
#             parameter_overrides=[
#                 Parameter('use_sim_time', Parameter.Type.BOOL, True)
#             ]
#         )

#         # TF buffer and listener for looking up transforms
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         # Broadcaster to publish world -> odom transform
#         self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

#         # Flags for tracking initial transform setup
#         self.origin_set = False
#         self.initial_transform = None

#         # Timer to periodically try to compute and publish the transform
#         self.create_timer(0.5, self.timer_callback)

#     def timer_callback(self):
#         if not self.origin_set:
#             try:
#                 # Use the current simulated time to look up transform
#                 now = self.get_clock().now().to_msg()

#                 # Lookup the transform from odom to base_link
#                 trans = self.tf_buffer.lookup_transform(
#                     'odom',
#                     'base_link',
#                     now,
#                     timeout=Duration(seconds=0.5)
#                 )
#                 trans = self.tf_buffer.lookup_transform(
#                     'odom',
#                     'base_link',
#                     rclpy.time.Time(), #self.get_clock().now().to_msg(),  # <-- 替代 rclpy.time.Time()
#                     timeout=rclpy.duration.Duration(seconds=1.0)
#                     )

#                 # Extract translation and rotation
#                 t = trans.transform.translation
#                 q = trans.transform.rotation

#                 # Build transformation matrix and compute its inverse
#                 T = tf_transformations.concatenate_matrices(
#                     tf_transformations.translation_matrix([t.x, t.y, t.z]),
#                     tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
#                 )
#                 T_inv = tf_transformations.inverse_matrix(T)

#                 # Extract inverted translation and rotation
#                 trans_vec = tf_transformations.translation_from_matrix(T_inv)
#                 rot_quat = tf_transformations.quaternion_from_matrix(T_inv)

#                 # Create the TransformStamped message for world -> odom
#                 self.initial_transform = TransformStamped()
#                 self.initial_transform.header.frame_id = 'world'
#                 self.initial_transform.child_frame_id = 'odom'
#                 self.initial_transform.transform.translation.x = trans_vec[0]
#                 self.initial_transform.transform.translation.y = trans_vec[1]
#                 self.initial_transform.transform.translation.z = trans_vec[2]
#                 self.initial_transform.transform.rotation.x = rot_quat[0]
#                 self.initial_transform.transform.rotation.y = rot_quat[1]
#                 self.initial_transform.transform.rotation.z = rot_quat[2]
#                 self.initial_transform.transform.rotation.w = rot_quat[3]

#                 self.origin_set = True
#                 self.get_logger().info('✅ Initial transform set for world -> odom')

#             except TransformException as e:
#                 self.get_logger().warn(f'Waiting for transform: {str(e)}')
#         else:
#             # Update timestamp and broadcast the transform
#             self.initial_transform.header.stamp = self.get_clock().now().to_msg()
#             self.tf_broadcaster.sendTransform(self.initial_transform)

# def main(args=None):
#     rclpy.init(args=args)
#     node = WorldTFPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
