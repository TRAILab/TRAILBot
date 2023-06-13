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

        target_location = blackboard.get('target_location', None)
        if target_location is not None:
            self.goal_publisher.publish(target_location)
        
        if blackboard.get('goal_reached', False):
            return 'arrived'
        return 'not_arrived'

class WaitState(State):
    def __init__(self, query_publisher, state_publisher, blackboard):
        super().__init__(outcomes=['snack_dispensed', 'snack_not_dispensed'])
        self.state_publisher = state_publisher
        self.query_publisher = query_publisher
        self.blackboard = blackboard

    def execute(self, blackboard):
        state_msg = String()
        state_msg.data = self.__class__.__name__
        self.state_publisher.publish(state_msg)
        
        query_msg = String()
        query_msg.data = "Query"
        self.query_publisher.publish(query_msg)

        if blackboard.get('snack_dispensed', False):
            return 'snack_dispensed'
        return 'snack_not_dispensed'

class DoneState(State):
    def __init__(self, state_publisher, blackboard):
        super().__init__(outcomes=['done'])
        self.state_publisher = state_publisher
        self.blackboard = blackboard

    def execute(self, blackboard):
        state_msg = String()
        state_msg.data = self.__class__.__name__
        self.state_publisher.publish(state_msg)
        print('Searchstate blackboard: ', blackboard)
        
        blackboard['target_found'] = False
        blackboard['goal_reached'] = False
        blackboard['snack_dispensed'] = False
        return 'done'
    
class FSM(Node):
    def __init__(self):
        super().__init__('trailbot_fsm')

        self.blackboard = {}
        self.current_state = 'SEARCH'

        self.state_publisher = self.create_publisher(String, 'trailbot_state', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.query_publisher = self.create_publisher(String, 'state', 10)

        # Update subscriber topics to subsribe to the lidar and voice assistant nodes
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
            bool,
            'query_complete',
            self.snack_callback,
            10
        )

        self.sm = StateMachine(outcomes=['finished'])

        self.sm.add_state('SEARCH', SearchState(self.state_publisher, self.blackboard), transitions={'target_found': 'APPROACH', 'target_not_found': 'SEARCH'})
        self.sm.add_state('APPROACH', ApproachState(self.goal_publisher, self.state_publisher, self.blackboard), transitions={'arrived': 'WAIT', 'not_arrived': 'APPROACH'})
        self.sm.add_state('WAIT', WaitState(self.query_publisher, self.state_publisher, self.blackboard), transitions={'snack_dispensed': 'DONE', 'snack_not_dispensed': 'WAIT'})
        self.sm.add_state('DONE', DoneState(self.state_publisher, self.blackboard), transitions={'done': 'SEARCH'})

    def run(self):
        while rclpy.ok():
            outcome = self.sm.execute(self.blackboard)
            self.current_state = outcome
            self.update_state(self.current_state)
            rclpy.spin_once(self)

    def goal_status_callback(self, msg):
        if msg.data == 'goal_reached':
            self.blackboard['goal_reached'] = True

    def target_callback(self, msg):
        self.blackboard['target_location'] = msg
        self.blackboard['target_found'] = True

    def snack_callback(self, msg):
        if msg.data == True:
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
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

