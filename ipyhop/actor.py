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
        if verbose >= 1:
            print("Initial plan created\n")
            if verbose >= 2:
                print("Plan is:\n" + str( plan ) + "\n")
            print("Executing plan...\n")
        curr_state = initial_state
        # act on plan until completion or failure, replanning has needed
        did_replan = False
        plan_impossible = False
        exec_index = 0
        while not plan_impossible:
            # execute until success or failure
            print(plan[ exec_index: ])
            exec_result = self.executor.execute( curr_state, plan[ exec_index: ] )
            # print(exec_result)
            if not any( map( lambda x: x[1], exec_result) ):
                break
            for i in range( 1, len( exec_result ) ):
                action, state = exec_result[ i ]
                history.append( action )
                # print( state )
                # failure of action has occurred
                if state == None:
                    if verbose >= 1:
                        print("Plan failed at: " + str(action))
                    plan, exec_index = self.planner.replan( exec_result[ i - 1][ 1 ], exec_index + i, verbose )
                    print( exec_index )
                    print( plan[exec_index:])
                    did_replan = True
                    if verbose >= 1:
                        print("New plan created\n")
                        if verbose >= 2:
                            print("New plan is:\n" + str(plan) + "\n")
                        print("Executing new plan...\n")
                    if plan == []:
                        plan_impossible = True
                        if verbose >= 1:
                            print( "No plan is possible...")
                    break
        if verbose >= 1:

            print( "Replanning was used: " + str( did_replan ) )
            print( "History is:\n" + str(history) + "\n" )
        return history


# ******************************************    Class Declaration End       ****************************************** #
# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError("Test run / Demo routine for Actor isn't implemented.")

"""
Author(s): Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
"""