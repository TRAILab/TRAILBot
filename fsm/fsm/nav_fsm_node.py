#! usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from math import sqrt
from rclpy.duration import Duration

class Navigator_Command(BasicNavigator):
    def __init__(self):
        super().__init__('navigator_node')

        # Publish nav goal status
        self.goal_status_publisher = self.create_publisher(String, 'goal_status', 10)
        # self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose2', 10)

        # Subscribe to the FSM robot state
        self.fsm_state_subscription = self.create_subscription(
            String,
            'trailbot_state',
            self.listener_callback_fsm_state,
            10)
        
        # Subscribe to the FSM external goal pose
        self.fsm_pose_subscription = self.create_subscription(
            PoseStamped,
            'goal_pose_fsm',
            self.listener_callback_fsm_goal,
            10
        )
        
        self.curr_fsm_state = None
        self.curr_fsm_goal = None
        # self.offset = 1000
        self.goal_exists = False
        self.nav_called = False
        self.cancel_task = True

        self.new_goal = False
        self.is_running = False

    def publish_nav_success(self):
        goal_status_msg = String()
        goal_status_msg.data = 'goal_reached'
        self.goal_status_publisher.publish(goal_status_msg)

    def listener_callback_fsm_state(self, msg):
        self.curr_fsm_state = msg.data
        self.get_logger().info('Curr FSM: "%s"' % msg.data, throttle_duration_sec=1)
        self.stop_nav()

    def listener_callback_fsm_goal(self, goal_msg):
        if self.curr_fsm_state == 'ApproachState':
            self.cancel_task = False
            if goal_msg != self.curr_fsm_goal: 
                self.curr_fsm_goal = goal_msg
                self.get_logger().info('New goal')
                self.new_goal = True
            elif not self.is_running: 
                self.curr_fsm_goal = goal_msg
                self.get_logger().info('Restarting')
                self.new_goal = True
        else:
            self.new_goal = False
    
    def stop_nav(self):
        self.cancel_task = False
        if self.curr_fsm_state != 'ApproachState':
            if self.result_future is not None:
                # self.get_logger().info('Currently running nav', throttle_duration_sec=1)
                if not self.isTaskComplete():
                    self.cancel_task = True
                    # self.get_logger().info('Cancel task', throttle_duration_sec=1)
                self.curr_fsm_goal = None
            # else:
                # self.get_logger().info('Not running nav', throttle_duration_sec=1)
            if self.is_running:
                self.get_logger().info('Failed to catch', throttle_duration_sec=1)
                self.cancel_task = True


    # def send_goal(self):
    #     if self.curr_fsm_state == 'ApproachState':
    #         self.goToPose(self.curr_fsm_goal)


        # if self.curr_fsm_state == 'ApproachState':
        #     self.cancel_task = False
        #     if self.goal_exists:
        #         self.nav_called = False
        #         # self.get_logger().info('Updating goal')
        #         # self.goToPose(self.curr_fsm_goal)
        #         # self.goal_publisher.publish(self.curr_fsm_goal)
        #     else:
        #         self.goal_exists = True
        #         self.nav_called = False
        #         # self.get_logger().info('Starting new goal')
        #         # self.goToPose(self.curr_fsm_goal)
        #         # self.goal_publisher.publish(self.curr_fsm_goal)

        # else:
        #     if self.result_future is not None:
        #         self.get_logger().info('Currently running nav', throttle_duration_sec=1)
        #         if not self.isTaskComplete():
        #             # self.cancelTask()
        #             self.cancel_task = True
        #             self.curr_fsm_goal = None
        #             # self.get_logger().info('Cancel task', throttle_duration_sec=1)

        

        # elif self.result_future is not None:
        #     if self.isTaskComplete():
        #         self.get_logger().info('Goal reached', throttle_duration_sec=1)
        #         goal_reached_msg = String()
        #         goal_reached_msg.data = 'goal_reached'
        #         self.goal_status_publisher.publish(goal_reached_msg)
            
        #     else:
        #         self.get_logger().info('Goal not reached', throttle_duration_sec=1)
        #         goal_not_reached_msg = String()
        #         goal_not_reached_msg.data = 'goal_not_reached'
        #         self.goal_status_publisher.publish(goal_not_reached_msg)

    def run(self):
        # while rclpy.ok():
        rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Navigator_Command()
    # rclpy.spin(minimal_publisher)
    minimal_publisher.run()
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()