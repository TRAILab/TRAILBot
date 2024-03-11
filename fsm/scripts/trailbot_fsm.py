#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

from simple_node import Node
from yasmin import StateMachine
from robot_navigator import BasicNavigator
from trailbot_states import SearchState, ApproachState, QueryState


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
