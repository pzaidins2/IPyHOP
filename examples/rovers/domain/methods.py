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

def mgm_achieve_goals_0( state, multigoal ):
    communicated_soil_data = state.communicated_soil_data
    desired_soil_data = multigoal.communicated_soil_data
    # get unachieved soil goals
    possessed_soil_data = filter( lambda x: communicated_soil_data[ x ], { *communicated_soil_data.keys() } )
    needed_soil_data = map( lambda x: x[ 0 ], desired_soil_data )
    missing_soil_data = needed_soil_data - possessed_soil_data
    for goal_loc in missing_soil_data:
        yield [ ( "communicated_soil_data", goal_loc, True ), multigoal ]

def mgm_achieve_goals_1( state, multigoal ):
    communicated_rock_data = state.communicated_rock_data
    desired_rock_data = multigoal.communicated_rock_data
    # get unachieved rock goals
    possessed_rock_data = filter( lambda x: communicated_rock_data[ x ], { *communicated_rock_data.keys() } )
    needed_rock_data = map( lambda x: x[ 0 ], desired_rock_data )
    missing_rock_data = needed_rock_data - possessed_rock_data
    for goal_loc in missing_rock_data:
        yield [ ( "communicated_rock_data", goal_loc, True ), multigoal ]

def mgm_achieve_goals_2( state, multigoal ):
    communicated_image_data = state.communicated_image_data
    desired_image_data = multigoal.communicated_image_data
    # get unachieved image goals
    possessed_image_data = filter( lambda x: communicated_image_data[ x ], { *communicated_image_data.keys() } )
    needed_image_data = map( lambda x: x[ 0 ], desired_image_data )
    missing_image_data = needed_image_data - possessed_image_data
    for goal_image in missing_image_data:
        yield [ ( "communicated_image_data", goal_image, True ), multigoal ]

def mgm_achieve_goals_3( state, multigoal ):
    communicated_soil_data = state.communicated_soil_data
    desired_soil_data = multigoal.communicated_soil_data
    communicated_rock_data = state.communicated_rock_data
    desired_rock_data = multigoal.communicated_rock_data
    communicated_image_data = state.communicated_image_data
    desired_image_data = multigoal.communicated_image_data
    # if all goals achieved end
    possessed_soil_data = filter( lambda x: communicated_soil_data[ x ], { *communicated_soil_data.keys() } )
    needed_soil_data = map( lambda x: x[ 0 ], desired_soil_data )
    missing_soil_data = needed_soil_data - possessed_soil_data
    possessed_rock_data = filter( lambda x: communicated_rock_data[ x ], { *communicated_rock_data.keys() } )
    needed_rock_data = map( lambda x: x[ 0 ], desired_rock_data )
    missing_rock_data = needed_rock_data - possessed_rock_data
    possessed_image_data = filter( lambda x: communicated_image_data[ x ], { *communicated_image_data.keys() } )
    needed_image_data = map( lambda x: x[ 0 ], desired_image_data )
    missing_image_data = needed_image_data - possessed_image_data
    if all( [
        len( missing_soil_data ) == 0,
        len( missing_rock_data ) == 0,
        len( missing_image_data ) == 0,
    ] ):
        yield []

# Create a IPyHOP Methods object. A Methods object stores all the methods defined for the planning domain.
methods = Methods()

methods.declare_multigoal_methods( None,
                                   N * [ mgm_achieve_goals_0,
                                         mgm_achieve_goals_1,
                                         mgm_achieve_goals_2,
                                         mgm_achieve_goals_3 ] )

def gm_empty_store_0( state, s, r ):
    empty = state.empty
    if empty[ s ]:
        yield []

def gm_empty_store_1( state, s, r ):
    empty = state.empty
    if not empty[ s ]:
        yield [ ( "drop", r, s ) ]

methods.declare_goal_methods( "empty", N * [ gm_empty_store_0, gm_empty_store_1 ] )

def gm_navigate_0( state, r, to ):
    at = state.at
    if at[ r ] == to:
        yield []

def gm_navigate_1( state, r, to ):
    at = state.at
    rigid = state.rigid
    can_traverse = rigid[ "can_traverse" ]
    # recursively call move task until to has been reached
    f = at[ r ]
    if f != to:
        p = path( can_traverse, r, f, to )
        if p != None:
            yield [ ( "t_move", r, p ) ]

methods.declare_goal_methods( "at", [ gm_navigate_0, gm_navigate_1 ] )

# recursively pop first move tuple off path
def tm_move_0( state, r, p ):
    if p == []:
        yield []

def tm_move_1( state, r, p ):
    if p != []:
        link = p.pop( 0 )
        yield [ ( "navigate", r, *link ), ( "t_move", r, p ) ]

methods.declare_task_methods( "t_move", [ tm_move_0, tm_move_1 ] )

def gm_communicated_soil_data_0( state, goal_loc ):
    # REPLANNING CASE
    yield

def gm_communicated_soil_data_1( state, goal_loc ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    rovers = type_dict[ "rover" ]
    have_soil_analysis = state.have_soil_analysis
    at = state.at
    # have sample, but have not communicated
    for r in rovers:
        if have_soil_analysis[ ( r, goal_loc ) ]:
            yield(
                [
                    ( "at", r, goal_loc )
                    ( "tm_communicate", "soil", goal_loc, at[ r ], r )
                ]
            )

def gm_communicated_soil_data_2( state, goal_loc ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    rovers = type_dict[ "rover" ]
    have_soil_analysis = state.have_soil_analysis
    at = state.at
    # have sample, but have not communicated
    for r in rovers:
        if have_soil_analysis[ ( r, goal_loc ) ]:
            yield(
                [
                    ( "at", r, goal_loc )
                    ( "tm_communicate", "soil", goal_loc, at[ r ], r )
                ]
            )
methods.declare_goal_methods( "communicated_soil_data", [ gm_communicated_soil_data_0, gm_communicated_soil_data_1, gm_communicated_soil_data_2 ] )

# REPLICATE ABOVE FOR ROCK AND IMAGE

def calibrate_camera_0( state, r, c ):
    calibrated = state.calibrated
    if calibrated[ ( r, c ) ]:
        yield []

def calibrate_camera_1( state, r, c ):
    calibrated = state.calibrated
    rigid = state.rigid
    calibration_target = rigid[ "calibration_target" ]
    visible_from = rigid[ "visible_from" ]
    if not calibrated[ ( r, c ) ]:
        calibration_target_c = calibration_target[ c ]
        for calibration_loc in visible_from[ calibration_target_c ]:
            yield [ ( "at", r, calibration_loc ), ( "calibrate", r, c, calibration_target_c, calibration_loc ) ]

methods.declare_goal_methods( "calibrated", [ calibrate_camera_0, calibrate_camera_1 ] )


"""
Author(s): Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
Organization: University of Maryland at College Park
"""