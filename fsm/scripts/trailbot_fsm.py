#!/usr/bin/env python3
import datetime
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from simple_node import Node
from yasmin import State 
from yasmin import StateMachine
from robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.action import NavigateToPose


# class SearchState(State):
#   def __init__(self, state_publisher, blackboard):
#     super().__init__(outcomes=["target_found", "target_not_found"])
#     self.state_publisher = state_publisher
#     self.blackboard = blackboard

#   def execute(self, blackboard):
#     state_msg = String()
#     state_msg.data = self.__class__.__name__
#     self.state_publisher.publish(state_msg)

#     # check if the target has been found
#     if blackboard.get("target_found", False):
#       return "target_found"
#     return "target_not_found"








class FSM(Node):
  def __init__(self):
    super().__init__("trailbot_state_machine")
    self.nav = BasicNavigator()

    # create state machine and blackboard
    # sm = StateMachine(outcomes=["finished"])
    self.blackboard = {} 
    # self.current_state = "SEARCH"

    # add states
    # self.sm.add_state("SEARCH", Search)

    # publish trailbot state
    self.state_publisher_ = self.create_publisher(String, 'trailbot_state', 10)

    # subscribe to goal pose topic
    self.goal_subscriber_ = self.create_subscription(PoseStamped, "goal_pose", self.goal_callback,10)

    # initialize goal flag
    self.goal_received = False


    self.nav_target()
    



  def nav_target(self):
    # wait for goal pose message
    while not self.goal_received:
      print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] Waiting for goal pose.")
    
    # reset flag for future goals
    self.goal_received = False

    # proceed with navigation if target found
    target_pose = self.blackboard.get("target_location")
    if target_pose:
      self.nav.goToPose(target_pose)

    while not self.nav.isTaskComplete():
      feedback = self.nav.getFeedback()
      # print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] {feedback.distance_remaining=}")
      msg = String()
      msg.data = f"[{datetime.datetime.now().strftime('%H:%M:%S')}] APPROACH STATE"
      self.state_publisher_.publish(msg)

    print(f"{self.nav.isTaskComplete()=}")
    result = self.nav.getResult()
    if result == TaskResult.SUCCEEDED:
      print("Goal succeeded!")
      msg = String()
      msg.data = f"[{datetime.datetime.now().strftime('%H:%M:%S')}] MOVE TO NEXT STATE"
      self.state_publisher_.publish(msg)


  def goal_callback(self, msg):
    self.blackboard["target_location"] = msg
    self.blackboard["target_found"] = True
    self.goal_received = True

    # print blackboard
    self.get_blackboard()

  def get_blackboard(self):
    print(f"Current Blackboard Contents")
    for key, val in self.blackboard.items():
      print(f"{key}: {val}")


def main():
  rclpy.init()
  fsm = FSM()
  rclpy.spin(fsm)
  fsm.nav.lifecycleShutdown()
  fsm.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()