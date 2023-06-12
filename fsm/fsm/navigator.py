#! /usr/bin/env python3
import time

from std_msgs.msg import Float32
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose

import rclpy
import sys
from threading import Thread
from queue import Queue

from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile


class BasicNavigator(Node):
    def __init__(self):
        super().__init__(node_name='navigator')
        self.initial_pose = Pose()
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.goal_queue = Queue()
        self.goal_received = False

        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.listener_callback,
            10)
        
        self.distance_pub = self.create_publisher(Float32, '/distance_remaining', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.initial_pose_received = False
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def setInitialPose(self, initial_pose):
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()

    def goToPose(self, pose):
        # Sends a `NavToPose` action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                      str(pose.pose.position.y))
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                           str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        # time.sleep(5)
        # result = self.result_future.result()
        # print(f'Action result: {result}')
        return True

    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.info('Goal succeeded!')
        return True

    def getFeedback(self):
        return self.feedback

    def getResult(self):
        return self.status

    def waitUntilNav2Active(self):
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1)
        return

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        print(self.feedback)
        return

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return
    
    def listener_callback(self, msg):
        # self.goToPose(msg)
    
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = msg.pose.position.x
        goal_pose.pose.position.y = msg.pose.position.y 
        goal_pose.pose.orientation.w = msg.pose.orientation.w
        self.get_logger().info(f'goal pose: {goal_pose}') 
        # self.goToPose(goal_pose)
        
        self.goal_queue.put(goal_pose)
        self.handle_navigation()
        self.goal_received = True
        self.get_logger().info('Goal received')
        # rclpy.shutdown()

        # i = 0
        # while not self.isNavComplete():
        #     ################################################
        #     #
        #     # Implement some code here for your application!
        #     #
        #     ################################################

        #     # Do something with the feedback
        #     i = i + 1
        #     feedback = self.getFeedback()
        #     if feedback and i % 5 == 0:
        #         # print('Estimated time of arrival: ' + '{0:.0f}'.format(
        #         #       Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
        #         #       + ' seconds.')
        #         print(f'Estimated distance remaining: {feedback.distance_remaining}')

        #         # Some navigation timeout to demo cancellation
        #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
        #             self.cancelNav()

        # # # Do something depending on the return code
        # result = self.getResult()
        # if result == GoalStatus.STATUS_SUCCEEDED:
        #     print('Goal succeeded!')
        # elif result == GoalStatus.STATUS_CANCELED:
        #     print('Goal was canceled!')
        # elif result == GoalStatus.STATUS_ABORTED:
        #     print('Goal failed!')
        # else:
        #     print('Goal has an invalid return status!')

    def handle_navigation(self):
        while rclpy.ok():
            if not self.goal_queue.empty():
                goal_pose = self.goal_queue.get()
                self.goToPose(goal_pose)

    def timer_callback(self):
        feedback = self.getFeedback()
        if feedback is not None:  # check if feedback is not None before accessing its fields
            msg = Float32()
            msg.data = feedback.distance_remaining
            self.distance_pub.publish(msg)
            # self.get_logger().info(f'Publishing: {msg.data}')
            self.i += 1

def navigation_loop(navigator):
    while rclpy.ok():
        if not navigator.goal_queue.empty():
            goal_pose = navigator.goal_queue.get()
            navigator.goToPose(goal_pose)

def main(argv=sys.argv[1:]):
    
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    # Set our demo's initial pose
    initial_pose = Pose()
    initial_pose.position.x = 0.0
    initial_pose.position.y = 0.0
    initial_pose.orientation.z = 1.0
    initial_pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    rclpy.spin(navigator)
   
    # navigation_thread = Thread(target=navigation_loop, args=(navigator,))
    # navigation_thread.start()

    # while rclpy.ok():
    #     rclpy.spin_once(navigator, timeout_sec=0.1)
    #     time.sleep(0.01)

    # navigation_thread.join()  # wait for the navigation thread to finish


    # navigation_thread = threading.Thread(target=navigator.handle_navigation)
    # navigation_thread.start()
    
    # while rclpy.ok() and not navigator.goal_received:
    #     rclpy.spin_once(navigator, timeout_sec=0.1)
        # navigator.goToPose()

    # navigation_thread = threading.Thread(target=navigator.handle_navigation)
    # navigation_thread.start()
    # rclpy.spin_once(navigator)
    
    # # Go to our demos first goal pose
    # goal_pose = PoseStamped()
    # goal_pose.header.frame_id = 'map'
    # goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose.pose.position.x = 3.0
    # goal_pose.pose.position.y = -3.0
    # goal_pose.pose.orientation.w = 1.0
    # navigator.goToPose(goal_pose)
    # # print('main pose: ', goal_pose)

    # i = 0
    # while not navigator.isNavComplete():
    #     ################################################
    #     #
    #     # Implement some code here for your application!
    #     #
    #     ################################################

    #     # Do something with the feedback
    #     i = i + 1
    #     feedback = navigator.getFeedback()
    #     if feedback and i % 5 == 0:
    #         # print('Estimated time of arrival: ' + '{0:.0f}'.format(
    #         #       Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
    #         #       + ' seconds.')
    #         print(f'Estimated distance remaining: {feedback.distance_remaining}')

    #         # Some navigation timeout to demo cancellation
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
    #             navigator.cancelNav()

    # # # Do something depending on the return code
    # result = navigator.getResult()
    # if result == GoalStatus.STATUS_SUCCEEDED:
    #     print('Goal succeeded!')
    # elif result == GoalStatus.STATUS_CANCELED:
    #     print('Goal was canceled!')
    # elif result == GoalStatus.STATUS_ABORTED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')

    exit(0)


if __name__ == "__main__":
    main()