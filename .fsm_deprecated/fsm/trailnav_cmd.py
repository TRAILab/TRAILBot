#! usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool


from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Pose
from vision_msgs.msg import Detection3DArray, Detection3D
from nav2_simple_commander.robot_navigator import BasicNavigator
from math import sqrt
from rclpy.duration import Duration

from numpy import array
from rclpy.clock import Clock

import tf2_ros

class Navigator_Command(BasicNavigator):
    def __init__(self):
        super().__init__('trailnav_cmd')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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
            'trail_location',
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
        self.check_new_nav_dist = False
        nav_update_thresh = 0.5 # distance between goals to update the current goal
        self.nav_update_thresh2 = nav_update_thresh**2

        # Track centerline
        self.check_new_trail_dist = False
        centerline_update_max_thresh = 2 # max distance allowed between consecutive new centerline points
        self.centerline_update_max_thresh2 = centerline_update_max_thresh**2
        self.curr_trail_point = PoseStamped()

        # Target tracking
        self.track_init_thresh = 2.0 # move towards target instead of trail when within this distance
        # self.track_init_thresh = 100000.0
        self.curr_track_id = -1
        self.curr_track_location = None
        self.no_target_count = 0
        self.no_target_thresh = 2

        self.array = array([0,0,0]) # dummy numpy array

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
            self.parse_tracks(msg)
        else:
            self.no_target_count += 1
            if self.curr_fsm_state == 'ApproachState':
                self.get_logger().info('Track msg with no targets, {} in a row'.format(self.no_target_count), throttle_duration_sec=1)
        if self.no_target_count >= self.no_target_thresh:
            self.publish_no_target()
            if self.curr_fsm_state == 'ApproachState':
                self.get_logger().info('ApproachState dropped after {} empty track messages'.format(self.no_target_thresh), throttle_duration_sec=1)

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
        
        elif self.curr_fsm_state == "SearchState":
            if self.curr_fsm_goal != self.curr_trail_point and self.curr_trail_point.header.frame_id != "":
                self.curr_fsm_goal = self.curr_trail_point
                self.get_logger().info('New trail goal', throttle_duration_sec=1)
                self.new_goal = True

        else: 
            if self.result_future is not None:
                if not self.isTaskComplete():
                    self.cancel_task = True
                    self.get_logger().info('Canceling nav', throttle_duration_sec=1)
                self.curr_fsm_goal = None
            if self.is_running:
                self.get_logger().info('Failed to catch', throttle_duration_sec=1)
                self.cancel_task = True
                self.curr_fsm_goal = None

    def parse_tracks(self, track_msgs):
        new_tracks = track_msgs.detections
        temp_id = None
        temp_loc = PoseStamped()
        temp_dist = None
        new_track = True
        for track in new_tracks:
            # If in Approach state, keep current track, else track closest person
            if track.id == self.curr_track_id and self.curr_fsm_state == "ApproachState":
                self.no_target_count = 0
                temp_loc.header = track_msgs.header
                temp_loc.pose = track.bbox.center
                self.curr_track_location = self.tf_buffer.transform(temp_loc, 'map')
                new_track = False
                self.get_logger().info('New target track position', throttle_duration_sec=1)
                self.get_logger().info('Target location in velo x:{} y:{} z:{}'.format(temp_loc.pose.position.x, temp_loc.pose.position.y, temp_loc.pose.position.z))
                self.get_logger().info('Target location in map x:{} y:{} z:{}'.format(self.curr_track_location.pose.position.x, self.curr_track_location.pose.position.y, self.curr_track_location.pose.position.z))
                break
            
            # self.get_logger().info('Not current track', throttle_duration_sec=1)
            self.array[0] = track.bbox.center.position.x
            self.array[1] = track.bbox.center.position.y
            self.array[2] = track.bbox.center.position.z
            dist = self.array.dot(self.array)
            if temp_id is not None:
                if dist < temp_dist:
                    temp_id = track.id
                    temp_loc.header = track_msgs.header
                    temp_loc.pose = track.bbox.center
                    temp_dist = dist
            else:
                temp_id = track.id
                temp_loc.header = track_msgs.header
                temp_loc.pose = track.bbox.center
                temp_dist = dist
        
        if new_track and self.curr_fsm_state != 'ApproachState':
            self.curr_track_id = temp_id
            self.curr_track_location = self.tf_buffer.transform(temp_loc, 'map')
            self.get_logger().info('New Track ID: {} |  Dist. {}'.format(self.curr_track_id, sqrt(dist)), throttle_duration_sec=1)

        elif new_track and self.curr_fsm_state == 'ApproachState':
            self.no_target_count += 1
            self.get_logger().info('Target ID {} not in frame'.format(self.curr_track_id), throttle_duration_sec=1)

        if self.curr_fsm_state == "SearchState" and sqrt(temp_dist) < self.track_init_thresh:
            self.no_target_count = 0
            self.get_logger().info('Target ID {} @ {} m is close enough to approach'.format(temp_id, sqrt(dist)), throttle_duration_sec=1)
            self.get_logger().info('Target location in velo x:{} y:{} z:{}'.format(temp_loc.pose.position.x, temp_loc.pose.position.y, temp_loc.pose.position.z), throttle_duration_sec=1)
            self.get_logger().info('Target location in map x:{} y:{} z:{}'.format(self.curr_track_location.pose.position.x, self.curr_track_location.pose.position.y, self.curr_track_location.pose.position.z), throttle_duration_sec=1)
            self.publish_target_close()

    def parse_trail_point(self, new_point):
        new_point_map = self.tf_buffer.transform(new_point, 'map')
        if self.curr_trail_point is not None and self.check_new_trail_dist:
            dist = self.curr_trail_point.pose.position.dot(new_point_map.pose.position)
            if dist < self.centerline_update_max_thresh2:
                self.curr_trail_point = new_point_map
                self.get_logger().info('Updated trail point', throttle_duration_sec=1)
            else:
                self.get_logger().info('New point is too far at {}'.format(sqrt(dist)), throttle_duration_sec=1)                

        else:
            self.curr_trail_point = new_point_map
            self.get_logger().info('Updated trail point', throttle_duration_sec=1)

    def run(self):
        rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Navigator_Command()
    while rclpy.ok():
        minimal_publisher.run()
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()