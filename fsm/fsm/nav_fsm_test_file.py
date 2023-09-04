#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from fsm.nav_fsm_node import Navigator_Command

import rclpy
from rclpy.duration import Duration
from time import sleep
from copy import deepcopy

"""
Basic security route patrol demo. In this demonstration, the expectation
is that there are security cameras mounted on the robots recording or being
watched live by security staff.
"""


def main():
    rclpy.init()

    navigator = Navigator_Command()
    # navigator = BasicNavigator()

    # Security route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    security_route = [
        [1.792, 2.144],
        [1.792, -5.44],
        [1.792, -9.427],
        [-3.665, -9.427],
        [-3.665, -4.303],
        [-3.665, 2.330],
        [-3.665, 9.283]]

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 3.45
    initial_pose.pose.position.y = 2.15
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Do security route until dead
    while rclpy.ok():
        # # Send our route
        # print('running')
        # # navigator.run()
        # route_poses = []
        # pose = PoseStamped()
        # pose.header.frame_id = 'map'
        # pose.header.stamp = navigator.get_clock().now().to_msg()
        # pose.pose.orientation.w = 1.0
        # for pt in security_route:
        #     pose.pose.position.x = pt[0]
        #     pose.pose.position.y = pt[1]
        #     route_poses.append(deepcopy(pose))
        # print('starting run')
        # navigator.goThroughPoses(route_poses)
        # navigator.goToPose(initial_pose)

        navigator.run()

        if navigator.new_goal and navigator.curr_fsm_state == 'ApproachState':
            print('New nav goal')
            navigator.is_running = True
            navigator.new_goal = False
            navigator.goToPose(navigator.curr_fsm_goal)

        if navigator.cancel_task and navigator.result_future:
            print('Canceling navigation')
            navigator.goal_handle.cancel_goal_async()
            navigator.is_running = False
        
        if navigator.is_running and navigator.isTaskComplete():
            print('Arrived at client')
            if navigator.getResult() == TaskResult.SUCCEEDED:
                navigator.publish_nav_success()

        # # Do something during our route (e.x. AI detection on camera images for anomalies)
        # # Simply print ETA for the demonstation
        # i = 0
        # while not navigator.isTaskComplete():
        #     i += 1
        #     feedback = navigator.getFeedback()
        #     if feedback and i % 5 == 0:
        #         print('Estimated time to complete current route: ' + '{0:.0f}'.format(
        #               Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
        #               + ' seconds.')

        #         # Some failure mode, must stop since the robot is clearly stuck
        #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
        #             print('Navigation has exceeded timeout of 180s, canceling request.')
        #             navigator.cancelTask()

        # # If at end of route, reverse the route to restart
        # security_route.reverse()

        # # result = navigator.getResult()
        # # if result == TaskResult.SUCCEEDED:
        # #     print('Route complete! Restarting...')
        # # elif result == TaskResult.CANCELED:
        # #     print('Security route was canceled, exiting.')
        # #     exit(1)
        # # elif result == TaskResult.FAILED:
        # #     print('Security route failed! Restarting from other side...')

    exit(0)


if __name__ == '__main__':
    main()

# def main():
#     ####################
#     request_item_location = 'shelf_C'
#     request_destination = 'pallet_jack7'
#     ####################

#     rclpy.init()

#     navigator = Navigator_Command()

#     # Set our demo's initial pose
#     # initial_pose = PoseStamped()
#     # initial_pose.header.frame_id = 'map'
#     # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
#     # initial_pose.pose.position.x = 3.45
#     # initial_pose.pose.position.y = 2.15
#     # initial_pose.pose.orientation.z = 1.0
#     # initial_pose.pose.orientation.w = 0.0
#     # navigator.setInitialPose(initial_pose)

#     # Wait for navigation to fully activate
#     # navigator.waitUntilNav2Active()

#     while rclpy.ok():
#         navigator.run()
#         if navigator.curr_fsm_state == 'ApproachState' and not navigator.nav_called:
#             print('a')
#             navigator.goToPose(navigator.curr_fsm_goal)
#             navigator.nav_called = True
#         else:
#             print('b')
#         i = 0
#         if navigator.results_future:
#             while not navigator.isTaskComplete():
#                 i += 1
#                 feedback = navigator.getFeedback()
#                 if feedback and i % 5 == 0:
#                     print('Estimated time of arrival at ' + request_item_location +
#                         ' for worker: ' + '{0:.0f}'.format(
#                             Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
#                         + ' seconds.')
#         else:                
#             i += 1
#             if i % 5 == 0:
#                 print("waiting")

#     # shelf_item_pose = PoseStamped()
#     # shelf_item_pose.header.frame_id = 'map'
#     # shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
#     # shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
#     # shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
#     # shelf_item_pose.pose.orientation.z = 1.0
#     # shelf_item_pose.pose.orientation.w = 0.0
#     # print(f'Received request for item picking at {request_item_location}.')
#     # navigator.goToPose(shelf_item_pose)

#     # # Do something during our route
#     # # (e.x. queue up future tasks or detect person for fine-tuned positioning)
#     # # Simply print information for workers on the robot's ETA for the demonstation
#     # i = 0
#     # while not navigator.isTaskComplete():
#     #     i += 1
#     #     feedback = navigator.getFeedback()
#     #     if feedback and i % 5 == 0:
#     #         print('Estimated time of arrival at ' + request_item_location +
#     #               ' for worker: ' + '{0:.0f}'.format(
#     #                   Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
#     #               + ' seconds.')

#     # result = navigator.getResult()
#     # if result == TaskResult.SUCCEEDED:
#     #     print('Got product from ' + request_item_location +
#     #           '! Bringing product to shipping destination (' + request_destination + ')...')
#     #     shipping_destination = PoseStamped()
#     #     shipping_destination.header.frame_id = 'map'
#     #     shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
#     #     shipping_destination.pose.position.x = shipping_destinations[request_destination][0]
#     #     shipping_destination.pose.position.y = shipping_destinations[request_destination][1]
#     #     shipping_destination.pose.orientation.z = 1.0
#     #     shipping_destination.pose.orientation.w = 0.0
#     #     navigator.goToPose(shipping_destination)

#     # elif result == TaskResult.CANCELED:
#     #     print(f'Task at {request_item_location} was canceled. Returning to staging point...')
#     #     initial_pose.header.stamp = navigator.get_clock().now().to_msg()
#     #     navigator.goToPose(initial_pose)

#     # elif result == TaskResult.FAILED:
#     #     print(f'Task at {request_item_location} failed!')
#     #     exit(-1)

#     # while not navigator.isTaskComplete():
#     #     pass

#     # print(f'Tasks completed!')




# if __name__ == '__main__':
#     main()