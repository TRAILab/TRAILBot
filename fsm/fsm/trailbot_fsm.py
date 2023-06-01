#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from nav2_msgs.action import NavigateToPose
# from nav2_msgs.action import NavigateToPose_FeedbackMessage


class FSM(Node):
    def __init__(self):
        super().__init__('trailbot_fsm')
        self.distance_pub = self.create_publisher(Float32, 'goal_distance_remaining', 10)
        self.feedback_sub = self.create_subscription(NavigateToPose.FeedbackMessage, 
                                                    'navigate_to_pose/_action/feedback', 
                                                    self.feedback_callback, 10)
        # self.feedback_sub = self.create_subscription(NavigateToPose.Feedback, 
        #                                              'navigate_to_pose/_action/feedback',
        #                                              self.listener_callback, 10)

    # def listener_callback(self, msg):
        # self.get_logger().info("data: %s" % msg.data)        
    
    # def getFeedback(self, msg):
    #     self.feedback = msg.feedback


    def feedback_callback(self, msg):
        
        self.get_logger().info('hello')
        distance_msg = Float32()
        distance_msg.data = msg.feedback.distance_remaining
        self.distance_pub.publish(distance_msg)
        self.get_logger().info('Published distance remaining: %f' % msg.feedback.distance_remaining)

def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()