#!/usr/bin/env python
"""
File Description: File used for definition of actor
"""

# ******************************************    Libraries to be imported    ****************************************** #
from typing import List, Union, Tuple, Optional
from ipyhop.state import State
from ipyhop.actions import Actions
from ipyhop.planner import IPyHOP
from ipyhop.mc_executor import MonteCarloExecutor
import numpy as np


# ******************************************    Class Declaration Start     ****************************************** #
class Actor:
    # ******************************        Class Method Declaration        ****************************************** #
    """
    Actor Constructor.

    :param planner: An instance of IPyHOP class containing the collection of methods and actions in the planning domain.
    :param executor: An instance of MonteCarloExecutor class containing the collection of actions in the planning domain.
    """
    def __init__(self, planner: IPyHOP, executor: MonteCarloExecutor):
        self.planner = planner
        self.executor = executor

    # ******************************        Class Method Declaration        ****************************************** #
    """
        Method that .

        :param initial_state: An instance of State class representing the initial state.
        :param to_do_list: A list of tuples of strings representing tasks and goals that must be completed and achieved.
    """
    def complete_to_do(self, initial_state: State, to_do_list: List[Tuple[str]], verbose: Optional[int]=0) -> List[Tuple[str]]:
        # list of actions performed
        history = []
        # find plan to complete to_do_list
        plan = self.planner.plan(initial_state, to_do_list, verbose=verbose)
        # print( self.planner.iterations )
        if verbose >= 1:
            print("Initial plan created\n")
            if verbose >= 2:
                print("Plan is:\n" + str( plan ) + "\n")
            print("Executing plan...\n")
        curr_state = initial_state
        # act on plan until completion or failure, replanning has needed
        did_replan = False
        plan_impossible = False
        plan_success = False
        exec_index = 0
        while not ( plan_impossible or plan_success ):
            # print(exec_index)
            # print(plan)
            # execute until success or failure
            # print(plan[ exec_index: ])
            exec_result = self.executor.execute( curr_state, plan[ exec_index: ] )
            # print(exec_result)
            # unzip list of tuples into seperate lists
            action_list, state_list = [ *zip( *exec_result ) ]
            # for i in range(len(exec_result)):
            #     print(action_list[i])
            #     print(state_list[i])
            # if no state is None plan executed successfully
            if state_list[ -1 ] != None:
                plan_success = True
                history += [ *action_list[ 1: ] ]
                if verbose >= 1:
                    print("Plan executed successfully")
                break
            else:
                # where failure occurred in most recent execution
                # print(plan)
                # print(action_list)
                action_index = len( state_list ) - 1
                # print(action_index)
                history += action_list[ 1: ]
                if verbose >= 2:
                    print("Plan failed at: " + str(action_list[action_index]))
                # update location relative to whole plan
                exec_index += action_index - 1
                # sanity check
                # print(plan[exec_index])
                # print(action_list[action_index])
                assert plan[ exec_index ] == action_list[ action_index ]
                # replan
                curr_state = state_list[ -2 ]
                # print(curr_state)
                # print(action_list[-1])
                plan, exec_index = self.planner.replan( curr_state, exec_index, verbose )
                if plan[ exec_index: ] == []:
                    plan_impossible = True
                    if verbose >= 1:
                        print( "No plan is possible...")
                    break
                else:
                    if verbose >= 2:
                        print( "New plan is:")
                        print(plan)
        if verbose >= 2 and plan_success:
            print("History:")
            for act in history:
                print(act)

        return history



# ******************************************    Class Declaration End       ****************************************** #
# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError("Test run / Demo routine for Actor isn't implemented.")

"""
Author(s): Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
"""