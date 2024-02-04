#!/usr/bin/env python3
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


class FSM(Node):
  def __init__(self):
    super().__init__("trailbot_state_machine")
    self.nav = BasicNavigator()
    # self.timer = self.create_timer(0.5, self.nav_callback)

    self.nav_target()

  def nav_callback(self):
    print(f"{self.nav.getResult()=}")
    print(f"{self.nav.isTaskComplete()=}")
    print(f"{self.nav.getFeedback()=}")

  def nav_target(self):
    # goal_pose = PoseStamped()
    # goal_pose.header.frame_id = "map"
    # goal_pose.pose.position.x = 1.0
    # goal_pose.pose.position.y = 0.0
    # goal_pose.pose.position.z = 0.0
    # goal_pose.pose.orientation.w = 1.0

    # self.nav.goToPose(goal_pose)

    print(f"{self.nav.isTaskComplete()=}")

    while not self.nav.isTaskComplete():
      feedback = self.nav.getFeedback()
      print(f"{feedback=}")

    result = self.nav.getResult()
    if result == TaskResult.SUCCEEDED:
      print("Goal succeeded!")
    elif result == TaskResult.CANCELED:
      print("Goal was canceled!")
    elif result == TaskResult.FAILED:
      print("Goal failed!")


def main():
  rclpy.init()
  fsm = FSM()
  rclpy.spin(fsm)
  fsm.nav.lifecycleShutdown()
  fsm.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()