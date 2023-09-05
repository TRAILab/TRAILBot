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
        super().__init__('trailnav_cmd')

        # Publish nav goal status
        self.goal_status_publisher = self.create_publisher(String, 'goal_reached', 10)
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
            self.listener_callback_trail_point,
            10)

        # Subscribe to the human detection pose array
        self.human_detection_subscription = self.create_subscription(
            Detection3DArray,
            'detection_location',
            self.listener_callback_tracking,
            10)

        self.curr_fsm_state = None

        # Nav stuff
        self.curr_fsm_goal = None
        self.cancel_task = True
        self.new_goal = False
        self.is_running = False
        self.check_new_tnav_dist = False
        nav_update_thresh = 0.5 # distance between goals to update the current goal
        self.nav_update_thresh2 = nav_update_thresh**2

        # Track centerline
        self.check_new_trail_dist = False
        centerline_update_max_thresh = 2 # max distance allowed between consecutive new centerline points
        self.centerline_update_max_thresh2 = centerline_update_max_thresh**2
        self.curr_trail_point = None

        # Target tracking
        self.track_init_thresh = 2.0 # move towards target instead of trail when within this distance
        self.curr_track_id = -1
        self.curr_track_location = None
        self.no_target_count = 0
        self.no_target_thresh = 2

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
        self.update_nav()

    def listener_callback_tracking(self, msg):
        if len(msg.detections):
            self.parse_tracks(msg.detections)
            self.no_target_count = 0
        else:
            self.no_target_count += 1
        if self.no_target_count >= self.no_target_thresh:
            self.publish_no_target()

    def listener_callback_trail_point(self, msg):
        self.parse_trail_point(msg)

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
    
    def update_nav(self):
        self.cancel_task = False
        self.new_goal = False
        if self.curr_fsm_state == "ApproachState":
            if self.curr_fsm_goal != self.curr_track_location:
                self.curr_fsm_goal = self.curr_track_location
                self.get_logger().info('New tracked target position')
                self.new_goal = True
        
        elif self.curr_fsm_state == "TravelState":
            if self.curr_fsm_goal != self.curr_trail_point:
                self.curr_fsm_goal = self.curr_trail_point
                self.get_logger().info('New trail goal', throttle_duration_sec=1)
                self.new_goal = True

        else: 
            if self.result_future is not None:
                if not self.isTaskComplete():
                    self.cancel_task = True
                self.curr_fsm_goal = None
            if self.is_running:
                self.get_logger().info('Failed to catch', throttle_duration_sec=1)
                self.cancel_task = True
                self.curr_fsm_goal = None

    def parse_tracks(self, new_tracks):
        temp_id = None
        temp_loc = None
        temp_dist = None
        for track in new_tracks:
            if track.id == self.curr_track_id and self.curr_fsm_state != "ApproachState":
                self.curr_track_location = track.bbox.center.pose
                break
            
            if temp_id is not None:
                dist = track.bbox.center.position.dot(track.bbox.center.position)
                if dist < temp_dist:
                    temp_id = track.id
                    temp_loc = track.bbox.center.pose
                    temp_dist = dist
            else:
                temp_id = track.id
                temp_loc = track.bbox.center.pose
                temp_dist = dist
            
            self.curr_track_id = temp_id
            self.curr_track_location = temp_loc
            
        if self.curr_fsm_state == "TravelState" and temp_dist < self.track_init_thresh:
            self.publish_target_close()

    def parse_trail_point(self, new_point):
        if self.curr_trail_point is not None and self.check_new_trail_points:
            dist = self.curr_trail_point.pose.position.dot(new_point.pose.position)
            if dist < self.centerline_update_max_thresh2:
                self.curr_trail_point = new_point
        else:
            self.curr_trail_point = new_point

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