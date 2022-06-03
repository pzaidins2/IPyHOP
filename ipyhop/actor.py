#!/usr/bin/env python
"""
File Description: File used for definition of actor
"""

# ******************************************    Libraries to be imported    ****************************************** #
from typing import List, Union, Tuple
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
    def complete_to_do(self, initial_state: State, to_do_list: List[Tuple[str]]) -> List[Tuple[str]]:
        # list of actions performed
        history = []
        # find plan to complete to_do_list
        plan = self.planner.plan(initial_state, to_do_list)
        curr_state = initial_state
        # act on plan until completion or failure, replanning has needed
        while plan != []:
            # execute until success or failure
            exec_result = self.executor.execute( curr_state, plan )
            plan = []
            for state, action in exec_result:
                history.append( action )
                # failure of action has occurred
                if state == None:
                    plan = self.planner.replan( state, action )
        return history
        print()


# ******************************************    Class Declaration End       ****************************************** #
# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError("Test run / Demo routine for Actor isn't implemented.")

"""
Author(s): Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
"""