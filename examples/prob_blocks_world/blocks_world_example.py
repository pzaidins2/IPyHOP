#!/usr/bin/env python
"""
File Description: Blocks World example file. Run this file to solve the Blocks World planning problem.
"""

# ******************************************    Libraries to be imported    ****************************************** #
from examples.prob_blocks_world.blocks_world_actions import actions
from examples.prob_blocks_world.blocks_world_methods import methods
from examples.prob_blocks_world.blocks_world_problem import init_state_1, goal1a, goal1b, init_state_2, goal2a, goal2b, \
    init_state_3, goal3
from ipyhop import IPyHOP
from ipyhop.mc_executor import MonteCarloExecutor
from ipyhop.actor import Actor
from io import StringIO


# ******************************************        Main Program Start      ****************************************** #
def main():
    print(methods)
    print(actions)
    planner = IPyHOP(methods, actions)

    print("""\r**************************************************************
        \rFirst, test IPyHOP on some of the actions and smaller tasks.
        \r**************************************************************\n""")

    print('\ninit_state_1: \n', init_state_1, '\n')

    print('- these should fail:')
    plan = planner.plan(init_state_1, [('a_pickup', 'a')], verbose=3)
    assert plan == [], "Result plan and expected plan are not same"
    print('Plan: ', plan, '\n')
    plan = planner.plan(init_state_1, [('a_pickup', 'b')], verbose=3)
    assert plan == [], "Result plan and expected plan are not same"
    print('Plan: ', plan, '\n')
    plan = planner.plan(init_state_1, [('get', 'b')], verbose=3)
    assert plan == [], "Result plan and expected plan are not same"
    print('Plan: ', plan, '\n')

    print('- these should succeed:')
    plan = planner.plan(init_state_1, [('a_pickup', 'c')], verbose=3)
    assert plan == [('a_pickup', 'c')], "Result plan and expected plan are not same"
    print('Plan: ', plan, '\n')
    plan = planner.plan(init_state_1, [('get', 'a')], verbose=3)
    assert plan == [('a_unstack', 'a', 'b')], "Result plan and expected plan are not same"
    print('Plan: ', plan, '\n')
    plan = planner.plan(init_state_1, [('get', 'c')], verbose=3)
    assert plan == [('a_pickup', 'c')], "Result plan and expected plan are not same"
    print('Plan: ', plan, '\n')

    plan = planner.plan(init_state_1, [('move_one', 'a', 'table')], verbose=3)
    val_plans = []
    for str_mod_0 in [ "", "50_", "0_" ]:
        for str_mod_1 in ["", "50_", "0_"]:
            pos_plan = [('a_{}unstack'.format(str_mod_0), 'a', 'b'), ('a_{}putdown'.format(str_mod_1), 'a')]
            val_plans.append(pos_plan)
    assert plan in val_plans, "Result plan is in set of expected plans"

    print("""\r************************************************************************************************
        \rRun IPyHOP on two block-stacking problems, both of which start in init_state_1. 
        \rThe goal for the 2nd problem omits some of the conditions in the goal of the 1st problem, but 
        \rthose conditions will need to be achieved anyway, so both goals should produce the same plan.
        \r************************************************************************************************\n""")

    print('\ngoal1a: \n', goal1a)
    print('\ngoal1b: \n', goal1b, '\n')

    # exp_1 = [('a_unstack', 'a', 'b'), ('a_putdown', 'a'), ('a_pickup', 'b'), ('a_stack', 'b', 'a'),
    #             ('a_pickup', 'c'), ('a_stack', 'c', 'b')]
    val_plans = []
    for str_mod_0 in ["", "50_", "0_"]:
        for str_mod_1 in ["", "50_", "0_"]:
            for str_mod_2 in ["", "50_", "0_"]:
                for str_mod_3 in ["", "50_", "0_"]:
                    for str_mod_4 in ["", "50_", "0_"]:
                        for str_mod_5 in ["", "50_", "0_"]:
                            pos_plan = [('a_{}unstack'.format(str_mod_0), 'a', 'b'),
                                        ('a_{}putdown'.format(str_mod_1), 'a'),
                                        ('a_{}pickup'.format(str_mod_2), 'b'),
                                        ('a_{}stack'.format(str_mod_3), 'b', 'a'),
                                        ('a_{}pickup'.format(str_mod_4), 'c'),
                                        ('a_{}stack'.format(str_mod_5), 'c', 'b')]
                            val_plans.append(pos_plan)
    plan = planner.plan(init_state_1, [('move_blocks', goal1a)], verbose=3)
    assert plan in val_plans, "Result plan and expected plan are not same"
    print('Plan: ', plan, '\n')
    plan = planner.plan(init_state_1, [('move_blocks', goal1b)], verbose=3)
    assert plan in val_plans, "Result plan and expected plan are not same"
    print('Plan: ', plan, '\n')

    print("""\r*******************************************************************************
    \rRun IPyHOP on two more planning problems. As before, the 2nd goal omits some 
    \rof the conditions in the 1st goal, but both goals should produce the same plan.
    \r*******************************************************************************\n""")

    print('\ninit_state_2: \n', init_state_2)
    print('\ngoal2a: \n', goal2a)
    print('\ngoal2b: \n', goal2b, '\n')

    exp_2 = [('a_unstack', 'a', 'c'), ('a_putdown', 'a'), ('a_unstack', 'b', 'd'), ('a_stack', 'b', 'c'),
             ('a_pickup', 'a'), ('a_stack', 'a', 'd')]
    val_plans = []
    for str_mod_0 in ["", "50_", "0_"]:
        for str_mod_1 in ["", "50_", "0_"]:
            for str_mod_2 in ["", "50_", "0_"]:
                for str_mod_3 in ["", "50_", "0_"]:
                    for str_mod_4 in ["", "50_", "0_"]:
                        for str_mod_5 in ["", "50_", "0_"]:
                            pos_plan = [('a_{}unstack'.format(str_mod_0), 'a', 'c'),
                                        ('a_{}putdown'.format(str_mod_1), 'a'),
                                        ('a_{}unstack'.format(str_mod_2), 'b', 'd'),
                                        ('a_{}stack'.format(str_mod_3), 'b', 'c'),
                                        ('a_{}pickup'.format(str_mod_4), 'a'),
                                        ('a_{}stack'.format(str_mod_5), 'a', 'd')]
                            val_plans.append(pos_plan)
    plan = planner.plan(init_state_2, [('move_blocks', goal2a)], verbose=3)
    assert plan in val_plans, "Result plan and expected plan are not same"
    print('Plan: ', plan, '\n')
    plan = planner.plan(init_state_2, [('move_blocks', goal2b)], verbose=3)
    assert plan in val_plans, "Result plan and expected plan are not same"
    print('Plan: ', plan, '\n')

    print("""\r**********************************************************************
    \rTest IPyHOP on planning problem bw_large_d from the SHOP distribution.
    \r**********************************************************************\n""")

    print('\ninit_state_3: \n', init_state_3)
    print('\ngoal3: \n', goal3, '\n')

    exp_3 = [('a_unstack', 1, 12), ('a_putdown', 1), ('a_unstack', 19, 18), ('a_putdown', 19), ('a_unstack', 18, 17),
             ('a_putdown', 18), ('a_unstack', 17, 16), ('a_putdown', 17), ('a_unstack', 9, 8), ('a_putdown', 9),
             ('a_unstack', 8, 7), ('a_putdown', 8), ('a_unstack', 11, 10), ('a_stack', 11, 7), ('a_unstack', 10, 5),
             ('a_putdown', 10), ('a_unstack', 5, 4), ('a_putdown', 5), ('a_unstack', 4, 14), ('a_putdown', 4),
             ('a_pickup', 9), ('a_stack', 9, 4), ('a_pickup', 8), ('a_stack', 8, 9), ('a_unstack', 14, 15),
             ('a_putdown', 14), ('a_unstack', 16, 3), ('a_stack', 16, 11), ('a_unstack', 3, 2), ('a_stack', 3, 16),
             ('a_pickup', 2), ('a_stack', 2, 3), ('a_unstack', 12, 13), ('a_stack', 12, 2), ('a_pickup', 13),
             ('a_stack', 13, 8), ('a_pickup', 15), ('a_stack', 15, 13)]
    val_plans = []
    str_mods = ["", "50_", "0_"]
    plan = planner.plan(init_state_3, [('move_blocks', goal3)], verbose=3)
    def plan_check( base_list, act_list, str_mods ):
        if len( act_list ) != len( base_list ):
            return False
        for i in range( len( base_list ) ):
            base_act = base_list[ i ]
            base_str = base_act[ 0 ]
            act = act_list[ i ]
            val_strs = map( lambda x: base_str[0:2] + x + base_str[2:], str_mods )
            val_acts = map( lambda x: (x, *base_act[1:]), val_strs )
            if act not in val_acts:
                return False
        return True
    assert plan_check(exp_3,plan,str_mods), "Result plan and expected plan are not same"
    print('Plan: ', plan, '\n')

    print("""\r*****************************************************************************
    \rTest acting on probabilistic blocks world
    \r*****************************************************************************\n""")
    mc_executor = MonteCarloExecutor(actions)
    actor = Actor( planner, mc_executor )
    history = actor.complete_to_do(init_state_3, [('move_blocks', goal3)], verbose=3)


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
