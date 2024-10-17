#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
# from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import PoseStamped
import tf2_ros
import numpy as np

from simple_node import Node
from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from fsm.robot_navigator import BasicNavigator
# from fsm.trailbot_states import SearchState, ApproachState, QueryState
from fsm.trailbot_states import SearchState, ApproachState, QueryState, StandbyState

import time

class FSM(Node):
  def __init__(self):
    super().__init__("trailbot_state_machine")
    self.nav = BasicNavigator()
    
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self) 
    
    # publish trailbot state
    self.state_publisher_ = self.create_publisher(String, "trailbot_state", 10)

    # subscribe to goal pose topic
    self.goal_subscriber_ = self.create_subscription(PoseStamped, "person_target", self.target_callback, 10)
    
    # # subscribe to trail pose topic
    self.trail_subscriber_ = self.create_subscription(PoseStamped, "trail_location", self.trail_callback, 10)

    # subscribe to dispenser client
    self.dispenser_subscriber_ = self.create_subscription(Bool, "query_complete", self.client_callback, 10)

    # self.timer = self.create_timer(1.0, self.trail_callback()) # seconds


    # create state machine (yasmin) and blackboard (dict)
    self.sm = StateMachine(outcomes=["finished"])

    self.blackboard = {"target_found":False,
                        "target_location":None,
                        "dispensed":False,
                        "new_trail_pose":False,
                        "trail_pose":None} 
    
    self.trail_update_dist = -1

    # add states
    self.sm.add_state("SEARCH", SearchState(self.state_publisher_, self.nav, self.get_logger()),
                      transitions={"target_found": "APPROACH", "target_not_found": "SEARCH"})
    self.sm.add_state("APPROACH", ApproachState(self.state_publisher_, self.nav, self.get_logger()),
                      transitions={"arrived": "QUERY", "not_arrived": "APPROACH"})
    self.sm.add_state("QUERY", QueryState(self.state_publisher_, self.get_logger()),
                      transitions={"snack_dispensed": "STANDBY", "snack_not_dispensed": "QUERY"})
    self.sm.add_state("STANDBY", StandbyState(self.state_publisher_, self.get_logger()),
                      transitions={"time_elapsed": "SEARCH", "time_left": "STANDBY"})

    # run state machine
    self.sm.execute(self.blackboard)
    
    
  def target_callback(self, msg):
    try:
      if msg.header.frame_id == "base_link":
        self.blackboard["target_location"] = None
        self.blackboard["target_found"] = False
        return
      new_target_point = self.tf_buffer.transform(msg, 'map')
      self.blackboard["target_location"] = new_target_point
      self.blackboard["target_found"] = True
      # self.blackboard["target_found"] = False

    except tf2_ros.TransformException as ex:
      # self.get_logger().info('Keep old trail location', throttle_duration_sec=1)
      self.get_logger().info('Could not transform os_lidar to map: {0}'.format(ex))
      return


  def trail_callback(self, msg):
    try:
      new_trail_point = self.tf_buffer.transform(msg, 'map')
      self.get_logger().info('New trail location', throttle_duration_sec=1)
      if self.blackboard['trail_pose'] is not None:
        old_trail_point = self.blackboard['trail_pose']
        delta = np.array([old_trail_point.pose.position.x-old_trail_point.pose.position.x, new_trail_point.pose.position.y-new_trail_point.pose.position.y])
        dist2 = delta.dot(delta)
        if dist2 > self.trail_update_dist:
          self.get_logger().info('Update trail location', throttle_duration_sec=1)
          self.blackboard["new_trail_pose"] = True
          self.blackboard["trail_pose"] = new_trail_point
          # self.blackboard["trail_out"] = msg
        else:
          self.get_logger().info('Keep old trail location', throttle_duration_sec=1)
      else:
          self.get_logger().info('First trail location', throttle_duration_sec=1)
          self.blackboard["new_trail_pose"] = True
          self.blackboard["trail_pose"] = new_trail_point
          # self.blackboard["trail_out"] = msg
    except tf2_ros.TransformException as ex:
      # self.get_logger().info('Keep old trail location', throttle_duration_sec=1)
      self.get_logger().info('Could not transform os_lidar to map: {0}'.format(ex))
      return
    

  def client_callback(self, msg):
    if msg.data:
      self.blackboard["dispensed"] = True
      

def main(args=None):
  rclpy.init(args=args)
  fsm = FSM()
  rclpy.spin(fsm)
  fsm.nav.lifecycleShutdown()
  fsm.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
