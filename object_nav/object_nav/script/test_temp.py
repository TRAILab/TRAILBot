# # import rclpy
# # from rclpy.node import Node
# # from geometry_msgs.msg import PoseStamped
# # from nav_msgs.msg import Path
# # import numpy as np

# # class TestPathPublisher(Node):
# #     def __init__(self):
# #         super().__init__('test_path_publisher')

# #         self.pose_sub = self.create_subscription(
# #             PoseStamped,
# #             '/tracked_pose',
# #             self.pose_callback,
# #             10
# #         )
# #         self.path_pub = self.create_publisher(Path, '/path', 10)

# #         self.sent = False
# #         self.get_logger().info("Waiting for robot pose to publish test path...")

# #     def pose_callback(self, msg):
# #         if self.sent:
# #             return

# #         # Get current position
# #         x = msg.pose.position.x
# #         y = msg.pose.position.y
# #         z = msg.pose.position.z
# #         self.get_logger().info(f"Received robot pose: x={x:.2f}, y={y:.2f}")

# #         # Assume facing +x (you can modify this to read yaw from orientation)
# #         num_points = 10
# #         step = 0.1  # 10 cm step
# #         path_msg = Path()
# #         path_msg.header.frame_id = 'map'
# #         path_msg.header.stamp = self.get_clock().now().to_msg()

# #         for i in range(num_points + 1):
# #             pose = PoseStamped()
# #             pose.header.frame_id = 'map'
# #             pose.header.stamp = self.get_clock().now().to_msg()
# #             pose.pose.position.x = x - i * step
# #             pose.pose.position.y = y
# #             pose.pose.position.z = z
# #             pose.pose.orientation.w = 1.0  # No rotation
# #             path_msg.poses.append(pose)

# #         self.path_pub.publish(path_msg)
# #         self.get_logger().info(f"Published test path with {len(path_msg.poses)} poses.")
# #         self.sent = True


# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = TestPathPublisher()
# #     rclpy.spin(node)
# #     node.destroy_node()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()


# # import rclpy
# # from rclpy.node import Node
# # from geometry_msgs.msg import PoseStamped
# # from nav2_simple_commander.robot_navigator import BasicNavigator

# # class GoalPoseSender(Node):
# #     def __init__(self):
# #         super().__init__('goal_pose_sender')

# #         self.navigator = BasicNavigator()
# #         self.navigator.lifecycleStartup()  # ⚠️ 使用 Cartographer 时不要用 waitUntilNav2Active()

# #         # 构造目标点
# #         goal = PoseStamped()
# #         goal.header.frame_id = 'map'
# #         goal.header.stamp = self.get_clock().now().to_msg()
# #         goal.pose.position.x = -0.50
# #         goal.pose.position.y = 0.0
# #         goal.pose.orientation.w = 1.0

# #         # 发布 goal
# #         self.get_logger().info("Sending goal to Nav2...")
# #         self.navigator.goToPose(goal)

# #         # 等待执行完成
# #         while not self.navigator.isTaskComplete():
# #             rclpy.spin_once(self, timeout_sec=0.1)

# #         result = self.navigator.getResult()
# #         self.get_logger().info(f"Navigation result: {result}")

# #         # 自动退出
# #         rclpy.shutdown()

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = GoalPoseSender()
# #     rclpy.spin(node)

# # if __name__ == '__main__':
# #     main()


# # import rclpy
# # from rclpy.node import Node
# # from geometry_msgs.msg import PoseStamped
# # from nav_msgs.msg import Path
# # import numpy as np
# # from scipy.spatial.transform import Rotation as R
# # import math

# # class RightForwardPathPublisher(Node):
# #     def __init__(self):
# #         super().__init__('right_forward_path_publisher')

# #         self.pose_sub = self.create_subscription(
# #             PoseStamped,
# #             '/tracked_pose',
# #             self.pose_callback,
# #             10
# #         )
# #         self.path_pub = self.create_publisher(Path, '/path', 10)

# #         self.sent = False
# #         self.get_logger().info("Waiting for tracked_pose to publish trajectory...")

# #     def pose_callback(self, msg: PoseStamped):
# #         if self.sent:
# #             return

# #         # Get position and yaw from orientation
# #         x = msg.pose.position.x
# #         y = msg.pose.position.y
# #         quat = msg.pose.orientation
# #         rotation = R.from_quat([quat.x, quat.y, quat.z, quat.w])
# #         yaw = rotation.as_euler('xyz')[2]
# #         self.get_logger().info(f"Robot pose: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.2f}°")

# #         # Compute direction: 45 degrees to the right of current heading
# #         right_forward_yaw = yaw - math.pi / 4  # Turn 45° right
# #         direction = np.array([math.cos(right_forward_yaw), math.sin(right_forward_yaw)])

# #         # Build path of 1m in that direction
# #         num_points = 10
# #         step_length = 0.1  # 10 cm per step
# #         path_msg = Path()
# #         path_msg.header.frame_id = 'map'
# #         path_msg.header.stamp = self.get_clock().now().to_msg()

# #         for i in range(num_points + 1):
# #             point = np.array([x, y]) + i * step_length * direction
# #             pose = PoseStamped()
# #             pose.header.frame_id = 'map'
# #             pose.header.stamp = self.get_clock().now().to_msg()
# #             pose.pose.position.x = float(point[0])
# #             pose.pose.position.y = float(point[1])
# #             pose.pose.position.z = 0.0
# #             pose.pose.orientation = msg.pose.orientation  # Keep original orientation
# #             path_msg.poses.append(pose)

# #         self.path_pub.publish(path_msg)
# #         self.get_logger().info(f"Published right-forward path with {len(path_msg.poses)} points.")
# #         self.sent = True


# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = RightForwardPathPublisher()
# #     rclpy.spin(node)
# #     node.destroy_node()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator
# import numpy as np
# import math
# from scipy.spatial.transform import Rotation as R

# class FollowRightForwardPath(Node):
#     def __init__(self):
#         super().__init__('follow_right_forward_path')
#         self.navigator = BasicNavigator()
#         self.navigator.lifecycleStartup()  # ✅ For Cartographer

#         self.pose_sub = self.create_subscription(
#             PoseStamped,
#             '/husky_pose',
#             self.pose_callback,
#             10
#         )

#         self.sent = False
#         self.get_logger().info("Waiting for /tracked_pose to send path...")

#     def pose_callback(self, msg: PoseStamped):
#         if self.sent:
#             return

#         # Get position and orientation
#         x = msg.pose.position.x 
#         y = msg.pose.position.y - 1.5
#         quat = msg.pose.orientation
#         rotation = R.from_quat([quat.x, quat.y, quat.z, quat.w])
#         yaw = rotation.as_euler('xyz')[2]
#         self.get_logger().info(f"Current pose: x={x:.2f}, y={y:.2f}, roll = {math.degrees(rotation.as_euler('xyz')[0]):.2f}°, pitch = {math.degrees(rotation.as_euler('xyz')[1]):.2f}°, yaw={math.degrees(yaw):.2f}°")

#         # Compute +45° right-forward direction
#         forward_angle = yaw + math.pi / 4  # turn right 45°
#         direction = np.array([math.cos(forward_angle), math.sin(forward_angle)])

#         # Generate 1-meter path along that direction
#         step = 0.1
#         num_steps = int(1.0 / step)
#         poses = []

#         for i in range(num_steps + 1):
#             pt = np.array([x, y]) + i * step * direction
#             pose = PoseStamped()
#             pose.header.frame_id = 'map'
#             pose.header.stamp = self.get_clock().now().to_msg()
#             pose.pose.position.x = pt[0]
#             pose.pose.position.y = pt[1]
#             pose.pose.position.z = 0.0

#             # Keep original orientation
#             pose.pose.orientation = msg.pose.orientation
#             poses.append(pose)

#         # Send path to Nav2 via goThroughPoses
#         self.get_logger().info(f"Sending path with {len(poses)} poses to Nav2... {poses[5:]}")
#         self.navigator.goThroughPoses(poses[5:])


#         while not self.navigator.isTaskComplete():
#             rclpy.spin_once(self, timeout_sec=0.1)

#         result = self.navigator.getResult()
#         self.get_logger().info(f"Navigation complete with result: {result}")
#         self.sent = True

# def main(args=None):
#     rclpy.init(args=args)
#     node = FollowRightForwardPath()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
import json
import os

# 路径
path_file = "/home/trailbot/Documents/data_process_Kitti/results/04/planed_path/global_path_raw.json"

# 确保文件存在
if not os.path.exists(path_file):
    print(f"❌ File not found: {path_file}")
    exit(1)

# 读取 JSON
with open(path_file, 'r') as f:
    try:
        data = json.load(f)
    except json.JSONDecodeError as e:
        print(f"❌ Failed to parse JSON: {e}")
        exit(1)

# 修改 trigger
data['trigger'] = True # False True

# 写回文件
with open(path_file, 'w') as f:
    json.dump(data, f, indent=4)

print(f"✅ Trigger has been set to {data['trigger']}.")
