from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from fsm.trailnav_cmd import Navigator_Command
import rclpy
# from rclpy.duration import Duration
# from time import sleep
# from copy import deepcopy

"""
Basic navigator
"""

def main():
    rclpy.init()

    navigator = Navigator_Command()

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Run until dead
    while rclpy.ok():

        navigator.run()

        if navigator.new_goal and navigator.curr_fsm_state in ('ApproachState', "SearchState"):
            print('New nav goal')
            navigator.is_running = True
            navigator.new_goal = False
            navigator.goToPose(navigator.curr_fsm_goal)

        if navigator.cancel_task and navigator.result_future:
            print('Canceling navigation')
            navigator.goal_handle.cancel_goal_async()
            navigator.is_running = False
        
        if navigator.is_running and navigator.isTaskComplete() and navigator.curr_fsm_state == 'ApproachState':
            print('Arrived at client')
            if navigator.getResult() == TaskResult.SUCCEEDED:
                navigator.publish_nav_success()

    exit(0)


if __name__ == '__main__':
    main()