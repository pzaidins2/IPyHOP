#!/usr/bin/env python
"""
File Description: Roversactions file. All the actions for Rovers planning domain are defined here.
Derived from: https://github.com/shop-planner/plan-repair-icaps2020-htnws/blob/master/problems/rovers/domain.pddl

Each IPyHOP action is a Python function. The 1st argument is the current state, and the others are the planning
action's usual arguments. This is analogous to how methods are defined for Python classes (where the first argument
is the class instance). For example, the function "a_pickup(state, b)" implements the planning action for the
task ('a_pickup', b).

The state is described with following properties:
- rigid = dict of properties that should not change
    - type_dict = dict with types as keys and sets of all objects of the corresponding type as values
    - equipped_for_soil_analysis = dict with rover as key and bool value
    - equipped_for_rock_analysis = dict with rover as key and bool value
    - equipped_for_imaging = dict with rover as key and bool value
    - supports = dict with camera as key and set of modes as value
    - visible = dict with waypoint as key and set of waypoints as value
    - visible_from = dict with objective as key and set of waypoints as value
    - store_of = dict with store as key and rover as value
    - calibration_target = dict with camera as key and objective as value
    - on_board = dict with camera as kay and rover as value
    - at_lander = dict with lander as key and the location waypoint as value
    - can_traverse = dict with rover as key and set of ( waypoint, waypoint ) as value
- at = dict with rover as key and the location waypoint as value

- empty = dict with store as key and bool as value
- have_rock_analysis =dict with ( rover, waypoint ) key and bool value
- have_soil_analysis =dict with ( rover, waypoint ) key and bool value
- full = dict with store as key and bool as value
- calibrated = dict with ( camera, rover ) key and bool value
- available = dict with rover as key and bool value
- have_image = dict with rover as key and set of ( objective, mode ) as value
- communicated_soil_data = dict with waypoint as key and bool value
- communicated_rock_data = dict with waypoint as key and bool value
- communicated_image_data = dict with waypoint as key and bool value
- at_soil_sample = dict with waypoint as key and bool value
- at_rock_sample = dict with waypoint as key and bool value
- channel_free = dict with lander as key and bool as value
"""

from ipyhop import Actions
from typing import List, Dict, Set

def navigate( state, r, p_0, p_1 ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    can_traverse = rigid[ "can_traverse" ]
    available = state.available
    at = state.at
    visible = rigid[ "visible" ]
    # type check
    if type_check( [ r, p_0, p_1 ], [ "rover", "waypoint", "waypoint" ], type_dict ):
        # preconditions
        # r can traverse from p_0 to p_1
        # r is available
        # r is at p_0
        # w_1 is visible from p_0
        if all( [
            (p_0, p_1) in can_traverse[ r ],
            available[ r ],
            at[ r ] == p_0,
            p_1 in visible[ p_0 ]
        ] ):
            # effects
            # location of r is now p_1
            state.at[ r ] = p_1
            return state

def sample_soil( state, r, s, p ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    can_traverse = rigid[ "can_traverse" ]
    available = state.available
    at = state.at
    at_soil_sample = state.at_soil_sample
    equipped_for_soil_analysis = rigid[ "equipped_for_soil_analysis" ]
    store_of = rigid[ "store_of" ]
    empty = state.empty
    # type check
    if type_check( [ r, s, p ], [ "rover", "store", "waypoint" ], type_dict ):
        # preconditions
        # r is at p
        # p is at soil sample
        # r is equipped for soil analysis
        # s is a store of r
        # s is empty
        if all( [
            at[ r ] == p,
            at_soil_sample[ p ],
            equipped_for_soil_analysis[ r ],
            store_of[ s ] == r,
            empty[ s ]
        ] ):
            # effects
            # s is not emptu
            # s is full
            # have soil analysis ( r, p )
            # p is not at soil_sample
            state.empty[ s ] = False
            state.full[ s ] = True
            state.have_soil_analysis[ ( r, p ) ] = True
            state.at_soil_sample[ p ] = False
            return state

def sample_rock( state, r, s, p ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    at = state.at
    at_rock_sample = state.at_rock_sample
    equipped_for_rock_analysis = rigid[ "equipped_for_rock_analysis" ]
    store_of = rigid[ "store_of" ]
    empty = state.empty
    # type check
    if type_check( [ r, s, p ], [ "rover", "store", "waypoint" ], type_dict ):
        # preconditions
        # r is at p
        # p is at rock sample
        # r is equipped for rock analysis
        # s is a store of r
        # s is empty
        if all( [
            at[ r ] == p,
            at_rock_sample[ p ],
            equipped_for_rock_analysis[ r ],
            store_of[ s ] == r,
            empty[ s ]
        ] ):
            # effects
            # s is not emptu
            # s is full
            # have rock analysis ( r, p )
            # p is not at rock_sample
            state.empty[ s ] = False
            state.full[ s ] = True
            state.have_rock_analysis[ ( r, p ) ] = True
            state.at_rock_sample[ p ] = False
            return state

def drop( state, r, s ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    store_of = rigid[ "store_of" ]
    full = state.full
    # type check
    if type_check( [ r, s ], [ "rover", "store" ], type_dict ):
        # preconditions
        # s is store of r
        # s is full
        if all( [
            store_of[ s ] == r,
            full[ s ]
        ] ):
            # effects
            # s is not full
            # s is empty
            state.full[ s ] = False
            state.empty[ s ] = True
            return state

def calibrate( state, r, i, t, w ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    equipped_for_imaging = rigid[ "equipped_for_imaging" ]
    calibration_target = rigid[ "calibration_target" ]
    at = state.at
    visible_from = rigid[ "visible_from" ]
    on_board = rigid[ "on_board" ]
    # type check
    if type_check( [ r, i, t, w ], [ "rover", "camera", "objective", "waypoint" ], type_dict ):
        # preconditions
        # r is equipped for imagining
        # t is the calibration target of i
        # r is at w
        # t is visible from w
        # i is on r
        if all( [
            equipped_for_imaging[ r ],
            calibration_target[ i ] == t,
            at[ r ] == w,
            w in visible_from[ t ],
            on_board[ i ] == r
        ] ):
            # effects
            # calibrate i on r
            state.calibrated[ ( i, r ) ] == True
            return state

def take_image( state, r, p, o, i, m ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    equipped_for_imaging = rigid[ "equipped_for_imaging" ]
    at = state.at
    visible_from = rigid[ "visible_from" ]
    on_board = rigid[ "on_board" ]
    calibrated = state.calibrated
    supports = rigid[ "supports" ]

    # type check
    if type_check( [ r, p, o, i , m ], [ "rover", "waypoint", "objective", "camera", "mode" ], type_dict ):
        # preconditions
        # i on r calibrated
        # i on r
        # i supports m
        # o visible from p
        # r at p
        if all( [
            calibrated[ ( i, r ) ],
            on_board[ i ] == r,
            equipped_for_imaging[ r ],
            m in supports[ i ],
            p in visible_from[ o ],
            at[ r ] == p
        ] ):
            # effects
            # r has image ( o, m )
            # ( i, r ) no longer calibrated
            state.have_image[ r ].add( ( o, m ) )
            state.calibrated[ ( i, r ) ] = False
            return state

def communicate_soil_data( state, r, l, p_0, p_1, p_2 ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    at = state.at
    visible= rigid[ "visible" ]
    at_lander = rigid[ "at_lander" ]
    have_soil_analysis = state.have_soil_analysis
    available = state.available
    free_channel = state.free_channel
    # type check
    if type_check( [ r, l, p_0, p_1, p_2 ], [ "rover", "lander", "waypoint", "waypoint", "waypoint" ], type_dict ):
        # preconditions
        # r at p_1
        # l at p_2
        # have soil analysis ( r, p_0 )
        # p_2 visible from p_1
        # r available
        # l has free channel
        if all( [
            at[ r ] == p_1,
            at_lander[ l ] == p_2,
            have_soil_analysis[ (r, p_0) ],
            p_2 in visible[ p_1 ],
            available[ r ],
            free_channel[ l ]
        ] ):
            # effects
            # soil data of p_0 has been communicated
            state.communicated_soil_data[ p_0 ] = True
            return state

def communicate_rock_data( state, r, l, p_0, p_1, p_2 ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    at = state.at
    visible= rigid[ "visible" ]
    at_lander = rigid[ "at_lander" ]
    have_rock_analysis = state.have_rock_analysis
    available = state.available
    free_channel = state.free_channel
    # type check
    if type_check( [ r, l, p_0, p_1, p_2 ], [ "rover", "lander", "waypoint", "waypoint", "waypoint" ], type_dict ):
        # preconditions
        # r at p_1
        # l at p_2
        # have rock analysis ( r, p_0 )
        # p_2 visible from p_1
        # r available
        # l has free channel
        if all( [
            at[ r ] == p_1,
            at_lander[ l ] == p_2,
            have_rock_analysis[ (r, p_0) ],
            p_2 in visible[ p_1 ],
            available[ r ],
            free_channel[ l ]
        ] ):
            # effects
            # rock data of p_0 has been communicated
            state.communicated_rock_data[ p_0 ] = True
            return state

def communicate_image_data( state, r, l, o, m, p_0, p_1 ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    at = state.at
    visible= rigid[ "visible" ]
    at_lander = rigid[ "at_lander" ]
    have_image = state.have_image
    available = state.available
    free_channel = state.free_channel
    # type check
    if type_check( [ r, l, o, m, p_0, p_1 ],
                   [ "rover", "lander", "objective", "mode", "waypoint", "waypoint" ], type_dict ):
        # preconditions
        # r at p_0
        # l at p_1
        # have rock analysis ( r, p_0 )
        # p_2 visible from p_1
        # r available
        # l has free channel
        if all( [
            at[ r ] == p_0,
            at_lander[ l ] == p_1,
            ( o, m ) in have_image[ r ],
            p_1 in visible[ p_0 ],
            available[ r ],
            free_channel[ l ]
        ] ):
            # effects
            # rock data of p_0 has been communicated
            state.communicated_image[ ( o, m ) ] = True
            return state





# Create a IPyHOP Actions object. An Actions object stores all the actions defined for the planning domain.
actions = Actions()
actions.declare_actions( [ ] )

action_probability = {

}

action_cost = {

}

actions.declare_action_models(action_probability, action_cost)

# ******************************************    Helper Functions            ****************************************** #

# takes list of entities to be type checked, their expected types, and a dict defining the objects for each type
# returns True if all checks are good else False
def type_check( entity_list: List[ str ], type_list: List[ str ], type_dict: Dict[ str, Set[ str ] ] ) -> bool:
    return all( map( lambda x, y: True if x in type_dict[ y ] else False, entity_list, type_list ) )

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError("Test run / Demo routine for Rovers Actions isn't implemented.")

"""
Author(s): Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
Organization: University of Maryland at College Park
"""