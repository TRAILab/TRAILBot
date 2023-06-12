#! /usr/bin/env python3
import rclpy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from simple_node import Node
from yasmin import State 
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub


class SearchState(State):
    def __init__(self, state_publisher, blackboard):
        super().__init__(outcomes=['target_found', 'target_not_found'])
        self.state_publisher = state_publisher
        self.blackboard = blackboard

    def execute(self, blackboard):
        state_msg = String()
        state_msg.data = self.__class__.__name__
        self.state_publisher.publish(state_msg)
        # print('SearchState blackboard: ', blackboard)

        # while not blackboard.get('target_found', False):
        #     rclpy.spin_once(self)

        # return 'target_found'

        rclpy.spin_once(self, timeout_sec=0.1)
        
        if blackboard.get('target_found', False):
            return 'target_found'
        return 'target_not_found'

class ApproachState(State):
    def __init__(self, goal_publisher, state_publisher, blackboard):
        super().__init__(outcomes=['arrived', 'not_arrived'])
        self.goal_publisher = goal_publisher
        self.state_publisher = state_publisher
        self.blackboard = blackboard

    def execute(self, blackboard):
        state_msg = String()
        state_msg.data = self.__class__.__name__
        self.state_publisher.publish(state_msg)
        # print('ApproachState blackboard: ', blackboard)

        target_location = blackboard.get('target_location', None)
        if target_location is not None:
            self.goal_publisher.publish(target_location)
        
        rclpy.spin_once(self, timeout_sec=0.1)

        if blackboard.get('goal_reached', False):
            return 'arrived'
        return 'not_arrived'

class WaitState(State):
    def __init__(self, state_publisher, blackboard):
        super().__init__(outcomes=['snack_dispensed', 'snack_not_dispensed'])
        self.state_publisher = state_publisher
        self.blackboard = blackboard
        # print('WaitState blackboard: ', blackboard)

    def execute(self, blackboard):
        state_msg = String()
        state_msg.data = self.__class__.__name__
        self.state_publisher.publish(state_msg)
        
        rclpy.spin_once(self, timeout_sec=0.1)
        
        if blackboard.get('snack_dispensed', False):
            return 'snack_dispensed'
        return 'snack_not_dispensed'

class DoneState(State):
    def __init__(self, state_publisher, blackboard):
        super().__init__(outcomes=['done'])
        self.state_publisher = state_publisher
        self.blackboard = blackboard
        # print('DoneState blackboard: ', blackboard)

    def execute(self, blackboard):
        state_msg = String()
        state_msg.data = self.__class__.__name__
        self.state_publisher.publish(state_msg)
        
        return 'done'
    
class FSM(Node):
    def __init__(self):
        super().__init__('trailbot_fsm')

        self.blackboard = {}
        self.state_publisher = self.create_publisher(String, 'trailbot_state', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # Modify subscriber topics to subscribe to the perception and voice assistant nodes
        self.goal_subscriber = self.create_subscription(
            String,
            'goal_status',
            self.goal_status_callback,
            10
        )
        self.target_subscriber = self.create_subscription(
            PoseStamped,
            'target_location',          # get PoseStamped msg from perception node 
            self.target_callback,
            10
        )
        self.snack_subscriber = self.create_subscription(
            String,
            'snack_dispensed',          # get String msg from voice assistant node      
            self.snack_callback,
            10
        )

        self.sm = StateMachine(outcomes=['finished'])

        self.sm.add_state('SEARCH', SearchState(self.state_publisher, self.blackboard), transitions={'target_found': 'APPROACH', 'target_not_found': 'SEARCH'})
        self.sm.add_state('APPROACH', ApproachState(self.goal_publisher,self.state_publisher, self.blackboard), transitions={'arrived': 'WAIT', 'not_arrived': 'APPROACH'})
        self.sm.add_state('WAIT', WaitState(self.state_publisher, self.blackboard), transitions={'snack_dispensed': 'DONE', 'snack_not_dispensed': 'WAIT'})
        self.sm.add_state('DONE', DoneState(self.state_publisher, self.blackboard), transitions={'done': 'SEARCH'})

    def run(self):
        # while rclpy.ok():
        #     outcome = self.sm.execute(self.blackboard)
        #     print(outcome)
        #     if self.current_state == 'SEARCH' and self.blackboard.get('target_found', False):
        #         self.current_state = 'APPROACH'
        #     else:
        #         self.current_state = outcome
        #         self.blackboard['target_found'] = False
        #         self.blackboard['goal_reached'] = False

        #     self.update_state(self.current_state)
        #     rclpy.spin_once(self)

        while rclpy.ok():
            outcome = self.sm.execute(self.blackboard)
            print(outcome)
            rclpy.spin_once(self)

    def goal_status_callback(self, msg):
        if msg.data == 'goal_reached':
            self.blackboard['goal_reached'] = True

    def target_callback(self, msg):
        # self.blackboard['target_location'] = msg
        # self.blackboard['target_found'] = True
        if msg != self.blackboard.get('target_location', None):
            self.blackboard['target_location'] = msg
            self.blackboard['target_found'] = True

    def snack_callback(self, msg):
        self.blackboard['snack_dispensed'] = True

    def update_state(self, state):
        if state != self.current_state:
            self.current_state = state
            state_msg = String()
            state_msg.data = state.__class__.__name__
            self.state_publisher.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    # node.join_spin()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# class FSM(Node):
#     def __init__(self):
#         super().__init__('trailbot_fsm')

#         self.blackboard = {}
#         self.current_state = None

#         self.state_publisher = self.create_publisher(String, 'trailbot_state', 10)
#         self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)

#         # Update subscriber topics to subsribe to the lidar and voice assistant nodes
#         self.goal_subscriber = self.create_subscription(
#             String,
#             'goal_status',
#             self.goal_status_callback,
#             10
#         )
#         self.target_subscriber = self.create_subscription(
#             PoseStamped,
#             'target_location',
#             self.target_callback,
#             10
#         )
#         self.snack_subscriber = self.create_subscription(
#             String,
#             'snack_dispensed',
#             self.snack_callback,
#             10
#         )

#         sm = StateMachine(outcomes=['finished'])

#         sm.add_state('SEARCH', SearchState(self.state_publisher, self.blackboard), transitions={'target_found': 'APPROACH', 'target_not_found': 'SEARCH'})
#         sm.add_state('APPROACH', ApproachState(self.goal_publisher,self.state_publisher, self.blackboard), transitions={'arrived': 'WAIT', 'not_arrived': 'APPROACH'})
#         sm.add_state('WAIT', WaitState(self.state_publisher, self.blackboard), transitions={'snack_dispensed': 'DONE', 'snack_not_dispensed': 'WAIT'})
#         sm.add_state('DONE', DoneState(self.state_publisher, self.blackboard), transitions={'done': 'SEARCH'})

#         outcome = sm.execute(self.blackboard)
#         print(outcome)

#     def goal_status_callback(self, msg):
#         if msg.data == 'goal_reached':
#             self.blackboard['goal_reached'] = True

#     def target_callback(self, msg):
#         self.blackboard['target_location'] = msg
#         self.blackboard['target_found'] = True

#     def snack_callback(self, msg):
#         self.blackboard['snack_dispensed'] = True

#     def update_state(self, state):
#         if state != self.current_state:
#             self.current_state = state
#             state_msg = String()
#             state_msg.data = state.__class__.__name__
#             self.state_publisher.publish(state_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = FSM()
#     # node.join_spin()
#     node.run()
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




