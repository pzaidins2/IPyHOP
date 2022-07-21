#!/usr/bin/env python
"""
File Description: Rovers methods file. All the methods for Rovers planning domain are defined here.
Derived from: https://github.com/shop-planner/plan-repair-icaps2020-htnws/blob/master/problems/rovers/domain.lisp

Each IPyHOP method is a Python function. The 1st argument is the current state (this is analogous to Python methods,
in which the first argument is the class instance). The rest of the arguments must match the arguments of the
task that the method is for. For example, the task ('get', b1) has a method "tm_get(state, b1)", as shown below.
"""

from ipyhop import Methods
from examples.rovers.domain.actions import type_check
from typing import TypeVar, Tuple, Set, Dict

# ******************************************    Helper Functions                     ********************************* #

rover = TypeVar( "rover" )
waypoint = TypeVar( "waypoint" )

# returns path if a path exists else returns None
def path( can_traverse: Dict[ Tuple[ waypoint ], bool ], r: rover, f: waypoint, t: waypoint ):
    rover_can_traverse = can_traverse[ r ]
    curr_wp = f
    paths = [ *map( lambda y: ( y, ), filter( lambda x: x[ 0 ], rover_can_traverse ) ) ]
    visited = set()
    # keep expanding frontier until either we no longer have any new reachable waypoints or t is in path
    while True:
        active_path = paths.pop( 0 )
        visited.add( active_path[ -1 ][ 1 ] )
        viable_traversals = { *filter( lambda x: x[ 0 ] and x[ 1 ] not in visited, rover_can_traverse ) }
        new_paths = [ ( *active_path, traversal ) for traversal in viable_traversals ]
        paths += new_paths
        if paths == [] or any( map( lambda x: x[ 1 ] == t, viable_traversals ) ):
            break
    usable_paths = filter( lambda x: x[ -1 ][ 1 ] == t, new_paths )
    if len( usable_paths ) > 0:
        return usable_paths[ 0 ]
    else:
        return None

# ******************************************    Methods                     ****************************************** #
N = 1

def mgm_achieve_goals_0( state, multigoal, rigid ):
    communicated_soil_data = state.communicated_soil_data
    desired_soil_data = { *multigoal.communicated_soil_data.keys() }
    # get unachieved soil goals
    for wp in desired_soil_data:
        if not communicated_soil_data[ wp ]:
            yield [ ( "communicated_soil_data", wp, True ), multigoal ]

def mgm_achieve_goals_1( state, multigoal, rigid ):
    communicated_rock_data = state.communicated_rock_data
    desired_rock_data = { *multigoal.communicated_rock_data.keys() }
    # get unachieved rock goals
    for wp in desired_rock_data:
        if not communicated_rock_data[ wp ]:
            yield [ ( "communicated_rock_data", wp, True ), multigoal ]

def mgm_achieve_goals_2( state, multigoal, rigid ):
    communicated_image_data = state.communicated_image_data
    desired_image_data = { *multigoal.communicated_image_data.keys() }
    # get unachieved image goals
    for g_image in desired_image_data:
        if not communicated_image_data[ g_image ]:
            yield [ ( "communicated_image_data", g_image, True ), multigoal ]

def mgm_achieve_goals_3( state, multigoal, rigid ):
    state_communicated_soil_data = { *state.communicated_soil_data.items() }
    desired_soil_data = { *multigoal.communicated_soil_data.items() }
    needed_soil_data = desired_soil_data - state_communicated_soil_data
    state_communicated_rock_data = { *state.communicated_rock_data.items() }
    desired_rock_data = { *multigoal.communicated_rock_data.items() }
    needed_rock_data = desired_rock_data - state_communicated_rock_data
    state_communicated_image_data = { *state.communicated_rock_image.items() }
    desired_image_data = { *multigoal.communicated_image_data.items() }
    needed_image_data = desired_image_data - state_communicated_image_data
    # if all goals achieved end
    if len( needed_soil_data ) == 0 and len( needed_rock_data ) == 0 and len( needed_image_data ) == 0:
        yield []

# Create a IPyHOP Methods object. A Methods object stores all the methods defined for the planning domain.
methods = Methods()

methods.declare_multigoal_methods( None,
                                   N * [ mgm_achieve_goals_0,
                                         mgm_achieve_goals_1,
                                         mgm_achieve_goals_2,
                                         mgm_achieve_goals_3 ] )

def tm_empty_store_0( state, s, r, rigid ):
    empty = state.empty
    if empty[ s ]:
        yield []

def tm_empty_store_1( state, s, r, rigid ):
    empty = state.empty
    if not empty[ s ]:
        yield [ ( "drop", r, s ) ]

methods.declare_goal_methods( "t_empty_store", N * [ tm_empty_store_0, tm_empty_store_1 ] )

def gm_navigate_0( state, r, to, rigid ):
    at = state.at
    if at[ r ] == to:
        yield []

def gm_navigate_1( state, r, to, rigid ):
    at = state.at
    can_traverse = rigid[ "can_traverse" ]
    # recursively call move task until to has been reached
    f = at[ r ]
    if f != to:
        p = path( can_traverse, r, f, to )
        if p != None:
            yield [ ( "t_move", r, p ) ]

methods.declare_goal_methods( "at", [ gm_navigate_0, gm_navigate_1 ] )

# recursively pop first move tuple off path
def tm_move_0( state, r, p, rigid ):
    if p == []:
        yield []

def tm_move_1( state, r, p, rigid ):
    if p != []:
        link = p.pop( 0 )
        yield [ ( "navigate", r, *link ), ( "t_move", r, p ) ]

methods.declare_task_methods( "t_move", [ tm_move_0, tm_move_1 ] )

def tm_communicated_soil_data_0( state, goal_loc, r, rigid ):
    have_soil_analysis = state.have_soil_analysis
    at = state.at
    # have sample, but have not communicated
    if have_soil_analysis[ ( r, goal_loc ) ]:
        yield(
            [
                ( "at", r, goal_loc ),
                ( "t_communicate", "soil", goal_loc, at[ r ], r )
            ]
        )

def tm_communicated_soil_data_1( state, goal_loc, r, rigid ):
    has_store = rigid[ "has_store" ]
    at = state.at
    s = has_store[ r ]
    yield(
        [
            ( "at", r, goal_loc ),
            ( "t_empty", s, r ),
            ( "sample_soil", r, s, goal_loc ),
            ( "t_communicate", "soil", goal_loc, at[ r ], r )
        ]
    )

methods.declare_goal_methods( "t_communicated_soil_data", [ tm_communicated_soil_data_0, tm_communicated_soil_data_1 ] )

def tm_communicated_rock_data_0( state, goal_loc, r, rigid ):
    have_rock_analysis = state.have_rock_analysis
    at = state.at
    # have sample, but have not communicated
    if have_rock_analysis[ ( r, goal_loc ) ]:
        yield(
            [
                ( "at", r, goal_loc ),
                ( "t_communicate", "rock", goal_loc, at[ r ], r )
            ]
        )

def tm_communicated_rock_data_1( state, goal_loc, r, rigid ):
    has_store = rigid[ "has_store" ]
    at = state.at
    s = has_store[ r ]
    yield(
        [
            ( "at", r, goal_loc ),
            ( "t_empty", s, r ),
            ( "sample_rock", r, s, goal_loc ),
            ( "t_communicate", "rock", goal_loc, at[ r ], r )
        ]
    )

methods.declare_goal_methods( "t_communicated_rock_data", [ tm_communicated_rock_data_0, tm_communicated_rock_data_1 ] )

def tm_communicated_image_data( state, goal_image, r, rigid ):
    carries = rigid[ "carries" ]
    supports = rigid[ "supports" ]
    at_lander = rigid[ "at_lander" ]
    type_dict = rigid[ "type_dict" ]
    landers = type_dict[ "lander" ]
    visible_from = rigid[ "visible_from" ]
    goal_obj, goal_mode = goal_image
    # iterate over on board cameras
    for c in carries[ r ]:
        if goal_mode in supports[ c ]:
            # iterate over location that can see object
            for wp in visible_from[ goal_obj ]:
                # iterate over potential landers
                for l in landers:
                    yield (
                        [
                            ( "calibrated", ( c, r ), True ),
                            ( "t_get_line_of_sight", r, goal_obj, wp ),
                            ( "take_image", r, wp, goal_obj, c, goal_mode ),
                            ( "t_communicate_image", wp, at_lander[ l ], r, goal_obj, goal_mode )
                        ] )

methods.declare_goal_methods( "t_communicated_image_data", [ tm_communicated_image_data ] )

def gm_calibrate_camera_0( state, r, c, rigid ):
    calibrated = state.calibrated
    if calibrated[ ( c, r ) ]:
        yield []

def gm_calibrate_camera_1( state, r, c, rigid ):
    calibrated = state.calibrated
    calibration_target = rigid[ "calibration_target" ]
    visible_from = rigid[ "visible_from" ]
    if not calibrated[ ( c, r ) ]:
        calibration_target_c = calibration_target[ c ]
        for calibration_loc in visible_from[ calibration_target_c ]:
            yield [ ( "at", r, calibration_loc ), ( "calibrate", r, c, calibration_target_c, calibration_loc ) ]

methods.declare_goal_methods( "calibrated", [ gm_calibrate_camera_0, gm_calibrate_camera_1 ] )

def tm_get_line_of_sight_0( state, r, obj, photo_loc, rigid ):
    at = state.at
    visible_from = rigid[ "visible_from" ]
    # has line of sight
    if at[ r ] == photo_loc and photo_loc in visible_from[ obj ]:
        yield []

def tm_get_line_of_sight_1( state, r, obj, photo_loc, rigid ):
    at = state.at
    visible_from = rigid[ "visible_from" ]
    # needs line of sight
    visible_from_obj = visible_from[ obj ]
    if at[ r ] not in visible_from_obj and photo_loc in visible_from_obj:
        yield [ ( "at", r, photo_loc ) ]

methods.declare_task_methods( "t_get_line_of_sight", [ tm_get_line_of_sight_0, tm_get_line_of_sight_1 ] )

def t_communicate_0( state, sample_type, a_loc, r_loc, r, rigid ):
    at = state.at
    at_lander = rigid[ "at_lander" ]
    visible = rigid[ "visible" ]
    type_dict = rigid[ "type_dict" ]
    landers = type_dict[ "lander" ]
    # have line of sight to lander
    for l in landers:
        l_loc = at_lander[ l ]
        if at[ r ] == r_loc and l_loc in visible[ r_loc ]:
            yield [ ( "communicate_" + sample_type + "_data", r, l, a_loc, r_loc, l_loc ) ]

def t_communicate_1( state, sample_type, a_loc, r_loc, r, rigid ):
    at = state.at
    at_lander = rigid[ "at_lander" ]
    visible = rigid[ "visible" ]
    type_dict = rigid[ "type_dict" ]
    landers = type_dict[ "lander" ]
    waypoints = type_dict[ "waypoint" ]
    # do not have line of sight to a lander
    for l in landers:
        l_loc = at_lander[ l ]
        if at[ r ] == r_loc and l_loc not in visible[ r_loc ]:
            for new_loc in waypoints:
                if l_loc in visible[ new_loc ]:
                    yield(
                        [
                            ( "at", r, new_loc )
                            ( "communicate_" + sample_type + "_data", r, l, a_loc, r_loc, new_loc )
                        ]
                    )

methods.declare_task_methods( "t_communicate", [ t_communicate_0, t_communicate_1 ] )

def communicate_image_0( state, r_loc, l_loc, r, obj, m, rigid ):
    type_dict = rigid[ "type_dict" ]
    landers =  type_dict[ "lander" ]
    at = state.at
    at_lander = rigid[ "at_lander" ]
    visible = rigid
    #  have line of sight
    if at[ r ] == r_loc and l_loc in visible[ r_loc ]:
        for l in landers:
            if at_lander[ l ] == l_loc:
                yield [ ( "communicate_image_data", r, l, obj, m, r_loc, l_loc ) ]

def communicate_image_1( state, r_loc, l_loc, r, obj, m, rigid ):
    type_dict = rigid[ "type_dict" ]
    landers =  type_dict[ "lander" ]
    waypoints = type_dict[ "waypoint" ]
    at = state.at
    at_lander = rigid[ "at_lander" ]
    visible = rigid
    # need line of sight
    if at[ r ] == r_loc and l_loc not in visible[ r_loc ]:
        for new_loc in waypoints:
            if l_loc in visible[ new_loc ]:
                for l in landers:
                    if at_lander[ l ] == l_loc:
                        yield(
                            [
                                ( "at", r, new_loc)
                                ( "communicate_image_data", r, l, obj, m, new_loc, l_loc )
                            ]
                        )
"""
Author(s): Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
Organization: University of Maryland at College Park
"""