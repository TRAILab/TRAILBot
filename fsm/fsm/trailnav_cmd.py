#! usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray, Detection3D
from nav2_simple_commander.robot_navigator import BasicNavigator
from math import sqrt
from rclpy.duration import Duration

class Navigator_Command(BasicNavigator):
    def __init__(self):
        super().__init__('navigator_node')

        # Publish nav goal status
        self.goal_status_publisher = self.create_publisher(Bool, 'goal_reached', 10)
        self.target_status_publisher = self.create_publisher(Bool, 'customer_is_close', 10)

        # Subscribe to the FSM robot state
        self.fsm_state_subscription = self.create_subscription(
            String,
            'trailbot_state',
            self.listener_callback_fsm_state,
            10)
        
        # Subscribe to the trail detection pose
        self.fsm_state_subscription = self.create_subscription(
            PoseStamped,
            'target_location',
            self.listener_callback_trail_pose,
            10)

        # Subscribe to the tracking detection pose array
        self.fsm_state_subscription = self.create_subscription(
            Detection3DArray,
            'detection_location',
            self.listener_callback_tracking,
            10)

        self.curr_fsm_state = None
        self.cancel_task = True
        self.new_goal = False
        self.is_running = False

        self.curr_track_id = None
        self.curr_track_location = None
        self.no_target_count = 0

    def publish_nav_success(self):
        goal_status_msg = String()
        goal_status_msg.data = 'goal_reached'
        self.goal_status_publisher.publish(goal_status_msg)

    def publish_no_target(self):
        goal_status_msg = String()
        goal_status_msg.data = 'no_target'
        self.goal_status_publisher.publish(goal_status_msg)
    
    def publish_target_close(self):
        target_status_msg = Bool()
        target_status_msg.data = True
        self.target_status_publisher.publish(target_status_msg)

    def listener_callback_fsm_state(self, msg):
        self.curr_fsm_state = msg.data
        self.get_logger().info('Curr FSM: "%s"' % msg.data, throttle_duration_sec=1)
        self.stop_nav()

    def listener_callback_tracking(self, msg):
        if len(msg.detections):
            self.parse_tracks(msg.detections)
            self.no_target_count = 0
        else:
            self.no_target_count += 1
        if self.no_target_count > 1:
            self.publish_no_target()
        self.update_nav()

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

    def parse_tracks(new_tracks):
        pass

    def run(self):
        rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Navigator_Command()
    minimal_publisher.run()
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()