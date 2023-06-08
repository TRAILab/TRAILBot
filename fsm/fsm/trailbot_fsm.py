#! /usr/bin/env python3
import rclpy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from simple_node import Node
from yasmin import State 
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub


class SearchState(State):
    def __init__(self, blackboard):
        super().__init__(outcomes=['target_found', 'target_not_found'])
        self.blackboard = blackboard

    def execute(self, blackboard):
        if blackboard.get('target_found', False):
            return 'target_found'
        return 'target_not_found'

class ApproachState(State):
    def __init__(self, goal_publisher, blackboard):
        super().__init__(outcomes=['arrived', 'not_arrived'])
        self.goal_publisher = goal_publisher
        self.blackboard = blackboard

    def execute(self, blackboard):
        target_location = blackboard.get('target_location', None)
        if target_location is not None:
            self.goal_publisher.publish(target_location)

        if blackboard.get('goal_reached', False):
            return 'arrived'
        return 'not_arrived'

class WaitState(State):
    def __init__(self, blackboard):
        super().__init__(outcomes=['snack_dispensed', 'snack_not_dispensed'])
        self.blackboard = blackboard

    def execute(self, blackboard):
        if blackboard.get('snack_dispensed', False):
            return 'snack_dispensed'
        return 'snack_not_dispensed'

class DoneState(State):
    def __init__(self, blackboard):
        super().__init__(outcomes=['done'])
        self.blackboard = blackboard

    def execute(self, blackboard):
        blackboard['target_found'] = False
        blackboard['snack_dispensed'] = False
        return 'done'
    
class FSM(Node):
    def __init__(self):
        super().__init__('trailbot_fsm')

        self.blackboard = {}

        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.goal_subscriber = self.create_subscription(
            String,
            'goal_status',
            self.goal_status_callback,
            10
        )
        self.target_subscriber = self.create_subscription(
            PoseStamped,
            'target_location',
            self.target_callback,
            10
        )
        self.snack_subscriber = self.create_subscription(
            String,
            'snack_dispensed',
            self.snack_callback,
            10
        )

        sm = StateMachine(outcomes=['finished'])

        sm.add_state('SEARCH', SearchState(self.blackboard), transitions={'target_found': 'APPROACH', 'target_not_found': 'SEARCH'})
        sm.add_state('APPROACH', ApproachState(self.goal_publisher,self.blackboard), transitions={'arrived': 'WAIT', 'not_arrived': 'SEARCH'})
        sm.add_state('WAIT', WaitState(self.blackboard), transitions={'snack_dispensed': 'DONE', 'snack_not_dispensed': 'WAIT'})
        sm.add_state('DONE', DoneState(self.blackboard), transitions={'done': 'SEARCH'})

        outcome = sm.execute(self.blackboard)
        print(outcome)

    def goal_status_callback(self, msg):
        if msg.data == 'goal_reached':
            self.blackboard['goal_reached'] = True

    def target_callback(self, msg):
        self.blackboard['target_location'] = msg
        self.blackboard['target_found'] = True

    def snack_callback(self, msg):
        self.blackboard['snack_dispensed'] = True

def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    node.join_spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




