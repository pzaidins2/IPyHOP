#!/usr/bin/env python
"""
File Description: Blocks World example file. Run this file to solve the Blocks World planning problem.
"""

# ******************************************    Libraries to be imported    ****************************************** #
from examples.satellite.domain.actions import actions
from examples.satellite.domain.methods import methods
from examples.satellite.domain.deviations import deviation_handler
from ipyhop import IPyHOP, State
from ipyhop.planner_old import IPyHOP_Old
from ipyhop.mulitgoal import MultiGoal
from ipyhop.actor import Actor
from ipyhop.mc_executor import MonteCarloExecutor
import re
import os
from functools import partial

# ******************************************        Helper Functions        ****************************************** #

# create sat pyhop problem
def init_sat( pddl_str, actions, methods, deviation_handler ):
    # create intial state
    state_0 = State("state_0")
    object_re = re.compile("(\w+) - (\w+)")
    init_re = re.compile("\(:init([\w\W]+)\)[\W]+\(:")
    dict_rel_re = re.compile("(supports|calibration_target|on_board|pointing|have_image) (\w+) (\w+)")
    power_avail_re = re.compile("power_avail (\w+)")
    # nil_equality_re = re.compile("= \(([\w\-]+)\) ([0-9\.]+)\)")
    # uni_equality_re = re.compile("= \((\w+) (\w+)\) ([0-9\.]+)\)")
    # bi_equality_re = re.compile("= \((\w+) (\w+) (\w+)\) ([0-9\.]+)\)")
    goal_re = re.compile("\(:goal([\w\W]+)")
    # unique entities
    object_list = object_re.findall(pddl_str)
    state_dict = dict()
    rigid = dict()
    rigid[ "type_dict" ] = dict()
    state_dict["satellite"] = set()
    rigid[ "type_dict" ][ "satellite" ] = state_dict["satellite"]
    state_dict["instrument"] = set()
    rigid[ "type_dict" ][ "instrument" ] = state_dict["instrument"]
    state_dict["mode"] = set()
    rigid[ "type_dict" ][ "mode" ] = state_dict["mode"]
    state_dict["direction"] = set()
    rigid[ "type_dict" ][ "direction" ] = state_dict["direction"]
    for object, obj_cat in object_list:
        state_dict[obj_cat].add(object)
    # initial state
    init_str = init_re.search(pddl_str).group(1)
    # print(init_str)
    # dictionary type relation
    dict_rel_list = dict_rel_re.findall(init_str)
    # print
    state_dict[ "supports" ] = dict()
    rigid[ "supports" ] = state_dict["supports"]
    state_dict["on_board" ] = dict()
    rigid[ "on_board" ] = state_dict["on_board"]
    state_dict["calibration_target"] = dict()
    rigid[ "calibration_target" ] = state_dict["calibration_target"]
    state_dict["pointing"] = dict()
    state_0.pointing = state_dict["pointing"]
    state_dict["have_image"] = dict()
    state_0.have_image = state_dict["have_image"]
    for rel, k, v in dict_rel_list:
        if rel in "supports":
            if k not in state_dict[rel].keys():
                state_dict[ rel ][ k ] = { v }
            else:
                state_dict[ rel ][ k ].add( v )
        elif rel == "on_board":
            if v not in state_dict[rel].keys():
                state_dict[ rel ][ v ] = { k }
            else:
                state_dict[ rel ][ v ].add( k )
        elif rel == "have_image":
            state_dict[ ( k, v ) ] =True
        else:
            state_dict[rel][k] = v
    # bool relation
    power_avail_list = power_avail_re.findall(init_str)
    state_dict["power_avail"] = dict()
    state_0.power_avail = state_dict["power_avail"]
    for sat in state_dict["satellite"]:
        if sat in power_avail_list:
            state_dict["power_avail"][ sat ] = True
        else:
            state_dict["power_avail"][sat] = False
    # # nil input equality
    # nil_equality_list = nil_equality_re.findall(init_str)
    # for rel, v in nil_equality_list:
    #     if rel == "data-stored":
    #         state_0.data_stored = float(v)
    #     elif rel == "fuel-used":
    #         state_0.fuel_used = float(v)
    # # single input equality
    # uni_equality_list = uni_equality_re.findall(init_str)
    # state_0.data_capacity = dict()
    # state_0.fuel = dict()
    # for rel, k, v in uni_equality_list:
    #     if rel == "data_capacity":
    #         rel_dict = state_0.data_capacity
    #     elif rel == "fuel":
    #         rel_dict = state_0.fuel
    #     rel_dict[k] = float(v)
    # # double input equality
    # bi_equality_list = bi_equality_re.findall(init_str)
    # state_dict["data"] = dict()
    # state_0.data = state_dict["data"]
    # state_dict["slew_time"] = dict()
    # state_0.slew_time = state_dict["slew_time"]
    # for rel, k_1, k_2, v in bi_equality_list:
    #     state_dict[rel][(k_1,k_2)] = float(v)
    # misc
    state_0.calibrated = { ins:False for ins in state_dict[ "instrument" ] }
    state_0.power_on = { ins:False for ins in state_dict[ "instrument" ] }
    # goal state
    goal_a = MultiGoal('goal_a')
    goal_str = goal_re.search(pddl_str).group(1)
    # print(init_str)
    # dictionary type relation
    dict_rel_list = dict_rel_re.findall(goal_str)
    # print(supports_list)
    goal_dict = dict()
    goal_dict["pointing"] = dict()
    goal_a.pointing = goal_dict["pointing"]
    goal_dict["have_image"] = dict()
    goal_a.have_image = goal_dict["have_image"]
    for rel, k, v in dict_rel_list:
        if rel == "have_image":
            goal_dict[rel][(k,v)]=True
            if (k,v) not in state_dict[ "have_image" ].keys():
                state_dict[ "have_image" ][ ( k, v ) ] = False
        else:
            goal_dict[ rel ][ k ] = v
    # rigid never needs to be copied so avoid this by partial evaluation of actions, methods, and deviation_handler that take rigid
    methods.goal_method_dict.update( 
        { l: [ partial( m, rigid=rigid ) for m in  ms ] for l, ms in methods.goal_method_dict.items() } )
    methods.task_method_dict.update( 
        { l: [ partial( m, rigid=rigid ) for m in ms ] for l, ms in methods.task_method_dict.items() } )
    methods.multigoal_method_dict.update(
        { l: [ partial( m, rigid=rigid ) for m in ms ] for l, ms in methods.multigoal_method_dict.items() } )
    actions.action_dict.update( { l: partial( a, rigid=rigid ) for l, a in actions.action_dict.items() } )
    deviation_handler = partial( deviation_handler, rigid=rigid )
    return state_0, goal_a, deviation_handler
# ******************************************        Main Program Start       ***************************************** #
def main():
    while True:
        # for problem_file_name in [ "problems/" + x for x in os.listdir( "problems" ) ]:
        # #     problem_file_name = "problems/p05.pddl"
        #     if "pddl" not in problem_file_name:
        #         continue
        #     print( problem_file_name )
            problem_file_name = "problems/p10.pddl"
            problem_file = open( problem_file_name, "r" )
            problem_str = problem_file.read()
            problem_file.close()

            planner = IPyHOP_Old( methods, actions )
            state_0, goal_a, dev_hand=  init_sat( problem_str, actions, methods, deviation_handler )
            mc_executor = MonteCarloExecutor( actions, dev_hand )
            actor = Actor( planner, mc_executor )
            history = actor.complete_to_do( state_0, [ goal_a ], verbose=3 )

# ******************************************        Main Program End        ****************************************** #
# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    try:
        main()
        print('\nFile executed successfully!\n')
    except KeyboardInterrupt:
        print('\nProcess interrupted by user. Bye!')


"""
Author(s): Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
Organization: University of Maryland at College Park
"""