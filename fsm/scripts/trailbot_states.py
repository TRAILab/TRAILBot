import datetime
from std_msgs.msg import String
from yasmin import State 
from robot_navigator import TaskResult

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