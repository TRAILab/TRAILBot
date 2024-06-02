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
    state_msg = String()
    current_time = datetime.datetime.now().strftime('%H:%M:%S')
    state_msg.data = f"[{current_time}] {self.__class__.__name__}"
    self.state_publisher_.publish(state_msg)

    if blackboard['new_trail_pose']:
      self.navigator.goToPose(blackboard.get("trail_pose"))
      # self.navigator.goToPose(blackboard.get("trail_out"))               # navigate to goal pose
      blackboard['new_trail_pose'] = False

    if blackboard.get("target_found", False):
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
    self.logger.info('Init ApproachState')

  def execute(self, blackboard):
    state_msg = String()
    current_time = datetime.datetime.now().strftime('%H:%M:%S')
    if self.curr_time is None:
      self.curr_time = time.strftime("%H:%M:%S +0000", time.gmtime())
    state_msg.data = f"[{current_time}] {self.curr_time} {self.__class__.__name__}"
    self.state_publisher_.publish(state_msg)

    target_location = blackboard.get("target_location")   # get goal pose
    if target_location is None:
      self.logger.info('target location is None')
      return "not_arrived"
    self.logger.info('before time check')
    # if time.time() - self.goal_time > 2:
    if self.goal_time < 0:
      self.logger.info('new target location')
      self.goal_time = time.time()
      self.navigator.goToPose(target_location)              # navigate to goal pose

    # check if robot reached goal
    self.logger.info('checking task')
    while not self.navigator.isTaskComplete():
      feedback = self.navigator.getFeedback()
      if feedback is not None:
        # print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] {feedback.distance_remaining=}")
        # current_time = datetime.datetime.now().strftime('%H:%M:%S')
        state_msg.data = f"[{current_time}] {self.__class__.__name__}"
        self.state_publisher_.publish(state_msg)
        self.logger.info("{}".format(feedback.estimated_time_remaining.sec))
        if feedback.estimated_time_remaining.sec == 0 and feedback.navigation_time.sec > 1.0:
          blackboard["arrived"] = True
          blackboard['target_location'] = None
          self.logger.info("no time to target = arrived")
          self.navigator.cancelTask()
          return "arrived"
      else:
        self.logger.info('feedback is None')

    # self.logger.info(self.navigator.getResult())
    if self.navigator.getResult() == TaskResult.SUCCEEDED:
      blackboard["arrived"] = True
      blackboard['target_location'] = None
      self.logger.info("nav success = arrived")
      return "arrived"
    return "not_arrived"

# class Nav2State(ActionState):
#     def __init__(self, state_publisher) -> None:
#         self.state_publisher_ = state_publisher
#         super().__init__(
#             NavigateToPose,  # action type
#             "/navigate_to_pose",  # action name
#             self.goal_pose_handler,  # cb to go to pose
#             None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
#             self.process_reponse,  # cb to process the response
#         )

#     def goal_pose_handler(self, blackboard):
#         target_location = blackboard.get("target_location")  # get goal pose
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose = target_location
#         goal_msg.behavior_tree = "/home/trailbot/trail_ws/src/TRAILBot/nav/config/navigate_to_pose_truncated_simple.xml"
#         state_msg = String()
#         current_time = datetime.datetime.now().strftime('%H:%M:%S')
#         state_msg.data = f"[{current_time}] {self.__class__.__name__}"
#         self.state_publisher_.publish(state_msg)
#         return goal_msg

class QueryState(State):
  def __init__(self, state_publisher, logger):
    super().__init__(outcomes=["snack_dispensed", "snack_not_dispensed"])
    self.state_publisher_ = state_publisher
    self.logger = logger
    self.curr_time = None

  def execute(self, blackboard):
    state_msg = String()
    current_time = datetime.datetime.now().strftime('%H:%M:%S')
    if self.curr_time is None:
      self.curr_time = time.strftime("%H:%M:%S +0000", time.gmtime())
    state_msg.data = f"[{current_time}] {self.__class__.__name__}"
    self.state_publisher_.publish(state_msg)

    if blackboard.get("dispensed", False):
      blackboard["target_found"] = False
      blackboard["dispensed"] = False
      return "snack_dispensed"
    return "snack_not_dispensed"