#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class MinimalActionClient(Node):

    def __init__(self):
        super().__init__('minimal_action_client')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    async def send_goal(self, goal_pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        future_goal_handle = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        goal_handle = await future_goal_handle
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        future_result = goal_handle.get_result_async()
        result = await future_result
        if result.result is not None:
            self.get_logger().info('Goal reached')
        else:
            self.get_logger().info('Goal failed to reach')

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Received feedback: {0}'.format(feedback_msg.feedback.navigation_time))

def main(args=None):
    rclpy.init(args=args)

    action_client = MinimalActionClient()

    goal_pose = PoseStamped()
    goal_pose.header.stamp = action_client.get_clock().now().to_msg()
    goal_pose.header.frame_id = "map"
    goal_pose.pose.position.x = 1.0
    goal_pose.pose.position.y = 2.0
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    rclpy.spin_until_future_complete(action_client, action_client.send_goal(goal_pose))

    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()