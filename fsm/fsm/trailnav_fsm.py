#! /usr/bin/env python3
import rclpy

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

from simple_node import Node
from yasmin import State 
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub


class ApproachState(State):
    def __init__(self, state_publisher, blackboard):
        super().__init__(outcomes=['arrived', 'not_arrived', 'no_target'])
        self.state_publisher = state_publisher
        self.blackboard = blackboard

    def execute(self, blackboard):
        state_msg = String()
        state_msg.data = self.__class__.__name__
        self.state_publisher.publish(state_msg)
        
        if blackboard.get('goal_reached', False):
            return 'arrived'
        elif blackboard.get('no_target', False):
            return 'no_target'
        return 'not_arrived'

class TravelState(State):
    def __init__(self, state_publisher, blackboard):
        super().__init__(outcomes=['customer_close', 'customer_not_close'])
        self.state_publisher = state_publisher
        self.blackboard = blackboard

    def execute(self, blackboard):
        state_msg = String()
        state_msg.data = self.__class__.__name__
        self.state_publisher.publish(state_msg)

        if blackboard.get('customer_is_close', False):
            return 'customer_close'
        return 'customer_not_close'

class QueryState(State):
    def __init__(self, state_publisher, blackboard):
        super().__init__(outcomes=['query_complete', 'query_not_complete'])
        self.state_publisher = state_publisher
        self.blackboard = blackboard

    def execute(self, blackboard):
        state_msg = String()
        state_msg.data = self.__class__.__name__
        self.state_publisher.publish(state_msg)

        if blackboard.get('query_completed', False):
            return 'query_complete'
        return 'query_not_complete'

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
        blackboard['query_completed'] = False
        return 'done'
    
class FSM(Node):
    def __init__(self):
        super().__init__('trailbot_fsm')

        self.blackboard = {}
        self.current_state = 'SEARCH'

        self.state_publisher = self.create_publisher(String, 'trailbot_state', 10)

        # Update subscriber topics to subsribe to the lidar and voice assistant nodes
        self.goal_subscriber = self.create_subscription(
            String,
            'goal_reached',
            self.goal_status_callback,
            10
        )
        self.customer_subscriber = self.create_subscription(
            Bool,
            'customer_is_close',
            self.customer_callback,
            10
        )
        self.query_complete_subscriber = self.create_subscription(
            Bool,
            'query_complete',
            self.query_complete_listener_callback,
            10
        )

        self.sm = StateMachine(outcomes=['finished'])

        self.sm.add_state('TRAVEL', TravelState(self.state_publisher, self.blackboard), transitions={'customer_close': 'APPROACH', 'customer_not_close': 'TRAVEL'})
        self.sm.add_state('APPROACH', ApproachState(self.state_publisher, self.blackboard), transitions={'arrived': 'QUERY', 'not_arrived': 'APPROACH', 'no_target': 'TRAVEL'})
        self.sm.add_state('QUERY', QueryState(self.state_publisher, self.blackboard), transitions={'query_complete': 'DONE', 'query_not_complete': 'QUERY'})
        self.sm.add_state('DONE', DoneState(self.state_publisher, self.blackboard), transitions={'done': 'TRAVEL'})

    def run(self):
        while rclpy.ok():
            outcome = self.sm.execute(self.blackboard)
            self.current_state = outcome
            self.update_state(self.current_state)
            # self.update_state(outcome)
            rclpy.spin_once(self)

    def goal_status_callback(self, msg):
        if msg.data == 'goal_reached':
            self.blackboard['goal_reached'] = True
        elif msg.data == 'no_target':
            self.blackboard['no_target'] = True
    
    def customer_callback(self, msg):
        if msg.data == True:
            self.blackboard['customer_is_close'] = True

    def query_complete_listener_callback(self, msg):
        if msg.data == True:
            self.blackboard['query_completed'] = True

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

