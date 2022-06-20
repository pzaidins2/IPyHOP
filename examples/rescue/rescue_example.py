#!/usr/bin/env python
"""
File Description: Rescue example file. Run this file to solve the Rescue planning problem.
"""

# ******************************************    Libraries to be imported    ****************************************** #
from __future__ import print_function
from examples.rescue.domain.rescue_methods import methods
from examples.rescue.domain.rescue_actions import actions
from examples.rescue.problem.rescue_problem_1 import init_state, task_list_1, task_list_2
from ipyhop import IPyHOP, MonteCarloExecutor, planar_plot
# from ipyhop.failure_handler import post_failure_tasks

# ******************************************        Main Program Start      ****************************************** #
def main():
    print(methods)
    print(actions)
    print(init_state)
    planner = IPyHOP(methods, actions)
    plan = planner.plan(init_state, task_list_2, verbose=0)
    graph = planner.sol_tree

    # planar_plot(graph, root_node=0)

    print('Plan: ')
    for action in plan:
        print('\t', action)

    executor = MonteCarloExecutor(actions, seed=10)
    exec_dict = dict()
    ed = exec_dict[1] = {'exec': [], 'stat': []}
    ed['exec'].append(executor.execute(init_state, plan))
    ed['stat'].append((len(planner.sol_tree.nodes), len(plan), planner.iterations))
    print(ed)
    print(ed['exec'][-1][-1][1])

    # ('a_move_euclidean', 'r1', (1, 1), (2, 2), None)
    fail_node = ('a_move_euclidean', 'r1', (1, 1), (2, 2), None)
    fail_index = plan.index( fail_node )
    planner.sol_plan = plan[ :fail_index ]
    state_list = planner.simulate( init_state )
    new_task_list = planner.replan( state_list[-1], fail_node, verbose=3 )
    print(new_task_list)
    print(planner.iterations)
    planar_plot(graph,0)

# ******************************************        Main Program End        ****************************************** #
# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    try:
        main()
        print('\nFile executed successfully!\n')
    except KeyboardInterrupt:
        print('\nProcess interrupted by user. Bye!')

"""
Author(s): Yash Bansod
Repository: https://github.com/YashBansod/IPyHOP
Organization: University of Maryland at College Park
"""