import datetime
from std_msgs.msg import String
from yasmin import State 
from fsm.robot_navigator import TaskResult
import time

from yasmin_ros import ActionState
from nav2_msgs.action import NavigateToPose

class SearchState(State):
  def __init__(self, state_publisher, basic_navigator, logger):
    super().__init__(outcomes=["target_found", "target_not_found"])
    self.state_publisher_ = state_publisher
    self.navigator = basic_navigator
    self.logger = logger

  def execute(self, blackboard):
    blackboard["dispensed"] = False
    state_msg = String()
    current_time = datetime.datetime.now().strftime('%H:%M:%S')
    state_msg.data = f"[{current_time}] {self.__class__.__name__}"
    self.state_publisher_.publish(state_msg)

    if blackboard['new_trail_pose']:
      self.navigator.goToPose(blackboard.get("trail_pose"))
      # self.navigator.goToPose(blackboard.get("trail_out"))               # navigate to goal pose
      blackboard['new_trail_pose'] = False

    if blackboard.get("target_found", False):
      self.logger.info(F" >>> INTO APPROACH {blackboard}")
      return "target_found"
    return "target_not_found"


class ApproachState(State):
  def __init__(self, state_publisher, basic_navigator, logger):
    super().__init__(outcomes=["arrived", "not_arrived"])
    self.state_publisher_ = state_publisher
    self.navigator = basic_navigator
    self.logger = logger
    self.curr_time = None
    self.goal_time = -1
    self.approach_buffer = 5.0 #in seconds, additional buffer to prevent the robot to transition from Approach to Query
    self.distance_threshold = 1.25 # (m) threshold between goal (human) and robot's location
    self.logger.info('Init ApproachState')

  def execute(self, blackboard):
    state_msg = String()
    current_time = datetime.datetime.now().strftime('%H:%M:%S')
    if self.curr_time is None:
      self.logger.info('New time init')
      self.curr_time = time.time()
    state_msg.data = f"[{current_time}] {self.__class__.__name__}"
    self.state_publisher_.publish(state_msg)

    target_location = blackboard.get("target_location")   # get goal pose
    if target_location is None:
      self.logger.info('target location is None')
      return "not_arrived"

    if time.time() - self.goal_time > 0.2:
      self.logger.info("new goal dt {}".format(time.time() - self.goal_time))
      # self.navigator.cancelTask()
      self.navigator.goToPose(target_location)   
      self.goal_time = time.time()           # navigate to goal pose

    # check if robot reached goal
    # self.logger.info('checking task')
    feedback = self.navigator.getFeedback()
    if feedback is not None:
      if feedback.distance_remaining <=  self.distance_threshold and time.time() - self.curr_time > self.approach_buffer: #and feedback.navigation_time.sec > 1.0:
        blackboard["arrived"] = True
        blackboard['target_location'] = None
        self.navigator.cancelTask()
        self.logger.info(F" >>> INTO QUERY (A) {blackboard}")
        self.curr_time = None
        return "arrived"
    else:
      self.logger.info('feedback is None')

    # self.logger.info(self.navigator.getResult())
    if self.navigator.getResult() == TaskResult.SUCCEEDED:
      blackboard["arrived"] = True
      blackboard['target_location'] = None
      self.logger.info("nav success = arrived")
      self.logger.info(F"  >>> INTO QUERY (B) {blackboard}")
      return "arrived"
    return "not_arrived"

class QueryState(State):
  def __init__(self, state_publisher, logger):
    super().__init__(outcomes=["snack_dispensed", "snack_not_dispensed"])
    self.state_publisher_ = state_publisher
    self.logger = logger
    self.curr_time = None

  def execute(self, blackboard):
    blackboard["arrived"] = False
    blackboard['target_location'] = None
    blackboard["target_found"] = False
    state_msg = String()
    current_time = datetime.datetime.now().strftime('%H:%M:%S')
    if self.curr_time is None:
      self.curr_time = time.strftime("%H:%M:%S +0000", time.gmtime())
    state_msg.data = f"[{current_time}] {self.__class__.__name__}"
    self.state_publisher_.publish(state_msg)

    if blackboard.get("dispensed", False):
      # blackboard["target_found"] = False
      blackboard["dispensed"] = False
      self.logger.info(F"  >>> INTO SEARCH {blackboard}")
      return "snack_dispensed"
    return "snack_not_dispensed"
  

class StandbyState(State):
  def __init__(self, state_publisher, logger):
    super().__init__(outcomes=["time_elapsed", "time_left"])
    self.state_publisher_ = state_publisher
    self.logger = logger
    self.start_time = None

  def execute(self, blackboard):
    state_msg = String()
    current_time = datetime.datetime.now().strftime('%H:%M:%S')
    # if self.curr_time is None:
    #   self.curr_time = time.strftime("%H:%M:%S +0000", time.gmtime())
    state_msg.data = f"[{current_time}] {self.__class__.__name__}"
    self.state_publisher_.publish(state_msg)

    if self.start_time is None:
      self.start_time = time.time()
      return "time_left"
    else:
      if time.time() - self.start_time > 10:
        self.start_time = None
        return "time_elapsed"
      else:
        return "time_left"


    
