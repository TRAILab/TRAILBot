#!/usr/bin/env python3

import time
import rclpy

from simple_node import Node

from yasmin import State
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub


# define state Standby
class StandbyState(State):
    def __init__(self):
        super().__init__(["launch_search", "shut_down"])
        self.counter = 0

    def execute(self, blackboard):
        print("Executing state Standby")

        while True:
            if blackboard.start == "True":
                return "launch_search"
            if blackboard.shutdown == "True":
                return "shut_down"   
            time.sleep(1)



# define state Exploration
class ExploreState(State):
    def __init__(self):
        super().__init__(outcomes=["found_person","lost", "shut_down"])

    def execute(self, blackboard):
        print("Executing state Exploration")

        while True:
            # Run path segmentation update
            path_seg = 0
            # Run plan update
            lost = 0
            # Run control update

            # Run person search
            found_person = 0
            
            if (lost):
                return "lost"
            if (found_person): 
                return "found_person"
            if blackboard.shutdown == "True":
                return "shut_down"   
            

# define state Approach
class ApproachState(State):
    def __init__(self):
        super().__init__(outcomes=["reached_person","lost_person", "shut_down"])

    def execute(self, blackboard):
        print("Executing state Approach")

        while True:
            # Run person tracking
            lost_person = 0
            # Run approach update
            approach_complete = 0
           
            if lost_person:
                return "lost_person"
            if found_person:
                blackboard.tries = 3  #Set number of query attempts 
                return "approach_complete"
            if blackboard.shut_down == "True":
                return "shut_down"

class ReacquireState(State):
    def __init__(self):
        super().__init__(outcomes=["found_person","lost","shut_down"])

    def execute(self, blackboard):
        print("Executing state Reacquisition")

        while True:
            # Run person search
            found_person = 1

            if found_person:
                return "found_person"
            if blackboard.lost == "True":
                return "lost"
            if blackboard.shut_down == "True":
                return "shut_down"

class QueryState(State):
    def __init__(self):
        super().__init__(outcomes=["query_complete"])

    def execute(self, blackboard):
        print("Executing state Query")

        # Send query
        time.sleep(5)
        query_complete = 1

        if (query_complete):
            return "query_complete"

class WaitForResponseState(State):
    def __init__(self):
        super().__init__(outcomes=["dispense_snack","no_snack","launch_search", "shut_down","response_failed"])

    def execute(self, blackboard):
        print("Executing state Wait For Response")

        blackboard.tries = blackboard.tries - 1
        # Process response
        get_answer = 0 # 0 - failed, 1 - yes, 2 - no

        if get_answer == 1:
            return "dispense_snack"
        if get_answer == 2:
            return "no_snack"
        if blackboard.tries == 1:
            return "launch_search"
        if blackboard.shut_down == "True":
            return "shut_down"


        # Announce failure
        return "response_failed"

    
class DispenseState(State):
    def __init__(self):
        super().__init__(outcomes=["launch_search", "shut_down"])

    def execute(self, blackboard):
        print("Executing state Dispense")

        # Execute dispense action
        blackboard.treats = blackboard.treats - 1
        if blackboard.treats > 0
            # Start a new search for customers
            return "launch_search"

        return "shut_down"

        
class RecoverState(State):
    def __init__(self):
        super().__init__(outcomes=["launch_search","shut_down"])

    def execute(self, blackboard):
        print("Executing state Recovery")

        if blackboard.shut_down == "True":
            return "shut_down"
        # Refind trail, maybe a spin move
        return "launch_search"

class FSMNode(Node):

    def __init__(self):
        super().__init__("trailbot_fsm_node")

        # create a state machine
        sm = StateMachine(outcomes=["Done"])

        # add states
        sm.add_state("Standby", StandbyState(),
                     transitions={"launch_search": "Explore",
                                  "shut_down": "Done"})
        sm.add_state("Explore", ExploreState(),
                     transitions={"found_person": "Approach",
                                  "lost": "Recover",
                                  "shut_down": "Done"})
        sm.add_state("Approach", ExploreState(),
                     transitions={"reached_person": "Query",
                                   "lost_person": "Reacquire",
                                   "shut_down": "Done"})
        sm.add_state("Reacquire", ReacquireState(),
                     transitions={"found_person": "Approach",
                                  "lost": "Recover",
                                   "shut_down": "Done"})
        sm.add_state("Query", QueryState(),
                     transitions={"query_complete": "WaitForResponse"})
        sm.add_state("WaitForResponse", WaitForResponseState(),
                     transitions={"dispense": "Dispense",
                                  "no_snack": "Explore",
                                  "launch_search": "Explore",
                                  "shut_down": "Done",
                                  "response_failed": "Query"})
        sm.add_state("Dispense", DispenseState(),
                     transitions={"launch_search": "Explore",
                                  "shut_down": "Done"}) # Should probably add wait for pickup
        sm.add_state("Recover", RecoverState(),
                     transitions={"launch_search": "Explore",
                                  "shut_down": "Done"})
        
        # add blackboard values
        blackboard.shut_down = False
        blackboard.fault = False
        blackboard.launch_search = False
        blackboard.treats = 2

        # pub
        YasminViewerPub(self, "trailbot_FSM", sm)

        # execute
        outcome = sm()
        print(outcome)


# main
def main(args=None):

    print("trailbot_FSM")
    rclpy.init(args=args)
    node = FSMNode()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()