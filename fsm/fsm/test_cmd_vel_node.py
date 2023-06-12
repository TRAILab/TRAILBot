#! usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('test_cmd_vel_node')
        self.goal_status_publisher = self.create_publisher(String, 'goal_status', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback_cmd_vel,
            10)

        self.state_subscription = self.create_subscription(
            String,
            '/trailbot_state',
            self.listener_callback_goal_status,
            10)

        self.last_cmd_vel = None
        self.last_goal_status = None

    def listener_callback_cmd_vel(self, msg):
        self.last_cmd_vel = msg
        self.check_condition_and_publish()

    def listener_callback_goal_status(self, goal_msg):
        self.last_goal_status = goal_msg.data
        self.check_condition_and_publish()
    
    def check_condition_and_publish(self):
        if self.last_cmd_vel is not None and self.last_goal_status is not None:
            
            if self.last_cmd_vel.linear.x <= 0.0 and self.last_cmd_vel.angular.z <= 0.2 and self.last_goal_status == 'ApproachState':
                goal_reached_msg = String()
                goal_reached_msg.data = 'goal_reached'
                self.goal_status_publisher.publish(goal_reached_msg)
        
                stop_msg = Twist()
                stop_msg.linear.x = 0.0
                stop_msg.angular.z = 0.0
                self.cmd_vel_publisher.publish(stop_msg)
            
            elif self.last_goal_status == 'WaitState':
                goal_reached_msg = String()
                goal_reached_msg.data = 'goal_reached'
                self.goal_status_publisher.publish(goal_reached_msg)
                
                stop_msg = Twist()
                stop_msg.linear.x = 0.0
                stop_msg.angular.z = 0.0
                self.cmd_vel_publisher.publish(stop_msg)
            else:
                goal_not_reached_msg = String()
                goal_not_reached_msg.data = 'goal_not_reached'
                self.goal_status_publisher.publish(goal_not_reached_msg)
                

            
        

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()