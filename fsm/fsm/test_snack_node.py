#! usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('test_snack_node')
        self.snack_status_publisher = self.create_publisher(String, 'snack_dispensed', 10)
        self.state_subscription = self.create_subscription(
            String,
            '/trailbot_state',
            self.listener_callback_goal_status,
            10
        )

    def listener_callback_goal_status(self, msg):
        if msg.data == 'WaitState':
            true_msg = String()
            true_msg.data = 'True'
            self.snack_status_publisher.publish(true_msg)
        else:
            false_msg = String()
            false_msg.data = 'False'
            self.snack_status_publisher.publish(false_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# #! usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String

# class MinimalPublisher(Node):
#     def __init__(self):
#         super().__init__('test_snack_node')
#         self.snack_status_publisher = self.create_publisher(String, 'snack_dispensed', 10)
#         self.state_subscription = self.create_subscription(
#             String,
#             '/trailbot_state',
#             self.listener_callback_goal_status,
#             10
#         )
#         self.timer = None

#     def listener_callback_goal_status(self, msg):
#         if msg.data == 'WaitState':
#             self.get_logger().info('Condition is true. Waiting 10 seconds...')
#             if self.timer is not None and self.timer.is_valid():
#                 self.timer.cancel()
#             self.timer = self.create_timer(10.0, self.publish_true)
#         else:
#             self.get_logger().info('Condition is false.')
#             self.publish_false()

#     def publish_true(self):
#         msg = String()
#         msg.data = 'True'
#         self.snack_status_publisher.publish(msg)
#         self.get_logger().info('Published "True"')

#     def publish_false(self):
#         msg = String()
#         msg.data = 'False'
#         self.snack_status_publisher.publish(msg)
#         self.get_logger().info('Published "False"')

# def main(args=None):
#     rclpy.init(args=args)
#     node = MinimalPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
