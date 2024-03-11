#!/usr/bin/env python3
import datetime
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

from simple_node import Node
from yasmin import State 
from yasmin import StateMachine
from robot_navigator import BasicNavigator, TaskResult

def get_blackboard(blackboard):
  print(f"Current Blackboard Contents")
  for key, val in blackboard.items():
    print(f"{key}: {val}")

class SearchState(State):
  def __init__(self, state_publisher):
    super().__init__(outcomes=["target_found", "target_not_found"])
    self.state_publisher_ = state_publisher

  def execute(self, blackboard):
    state_msg = String()
    current_time = datetime.datetime.now().strftime('%H:%M:%S')
    state_msg.data = f"[{current_time}] {self.__class__.__name__}"
    self.state_publisher_.publish(state_msg)

    if blackboard.get("target_found", False):
      return "target_found"
    return "target_not_found"

class ApproachState(State):
  def __init__(self, state_publisher, basic_navigator):
    super().__init__(outcomes=["arrived", "not_arrived"])
    self.state_publisher_ = state_publisher
    self.navigator = basic_navigator

  def execute(self, blackboard):
    state_msg = String()
    current_time = datetime.datetime.now().strftime('%H:%M:%S')
    state_msg.data = f"[{current_time}] {self.__class__.__name__}"
    self.state_publisher_.publish(state_msg)

    target_location = blackboard.get("target_location")   # get goal pose
    self.navigator.goToPose(target_location)              # navigate to goal pose

    # check if robot reached goal
    while not self.navigator.isTaskComplete():
      feedback = self.navigator.getFeedback()
      print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] {feedback.distance_remaining=}")
      current_time = datetime.datetime.now().strftime('%H:%M:%S')
      state_msg.data = f"[{current_time}] {self.__class__.__name__}"
      self.state_publisher_.publish(state_msg)

    if self.navigator.getResult() == TaskResult.SUCCEEDED:
      blackboard["arrived"] = True
      get_blackboard(blackboard)
      return "arrived"
    return "not_arrived"

class QueryState(State):
  def __init__(self, state_publisher):
    super().__init__(outcomes=["snack_dispensed", "snack_not_dispensed"])
    self.state_publisher_ = state_publisher

  def execute(self, blackboard):
    state_msg = String()
    current_time = datetime.datetime.now().strftime('%H:%M:%S')
    state_msg.data = f"[{current_time}] {self.__class__.__name__}"
    self.state_publisher_.publish(state_msg)

    if blackboard.get("dispensed", False):
      blackboard["target_found"] = False
      blackboard["dispensed"] = False
      return "snack_dispensed"
    return "snack_not_dispensed"
    

class FSM(Node):
  def __init__(self):
    super().__init__("trailbot_state_machine")
    self.nav = BasicNavigator() 
    
    # publish trailbot state
    self.state_publisher_ = self.create_publisher(String, "trailbot_state", 10)

    # subscribe to goal pose topic
    self.goal_subscriber_ = self.create_subscription(PoseStamped, "target_pose", self.target_callback, 10)

    # subscribe to dispenser client
    self.dispenser_subscriber_ = self.create_subscription(Bool, "client_state", self.client_callback, 10)

    # create state machine (yasmin) and blackboard (dict)
    self.sm = StateMachine(outcomes=["finished"])
    self.blackboard = {"target_found":False, "target_location":None, "dispensed":False} 

    # add states
    self.sm.add_state("SEARCH", SearchState(self.state_publisher_),
                      transitions={"target_found": "APPROACH", "target_not_found": "SEARCH"})
    self.sm.add_state("APPROACH", ApproachState(self.state_publisher_, self.nav),
                      transitions={"arrived": "QUERY", "not_arrived": "APPROACH"})
    self.sm.add_state("QUERY", QueryState(self.state_publisher_),
                      transitions={"snack_dispensed": "SEARCH", "snack_not_dispensed": "QUERY"})

    # run state machine
    self.sm.execute(self.blackboard)
    
  def target_callback(self, msg):
    self.blackboard["target_found"] = True
    self.blackboard["target_location"] = msg

  def client_callback(self, msg):
    if msg.data:
      self.blackboard["dispensed"] = True

def main():
  rclpy.init()
  fsm = FSM()
  rclpy.spin(fsm)
  fsm.nav.lifecycleShutdown()
  fsm.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
