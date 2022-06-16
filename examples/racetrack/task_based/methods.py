#!/usr/bin/env python
"""
File Description: Racetrack methods file. All the methods for Racetrack planning domain are defined here.
Derived from: PyHOP example of Racetrack varient domain by Dana Nau <nau@cs.umd.edu>

Each IPyHOP method is a Python function. The 1st argument is the current state (this is analogous to Python methods,
in which the first argument is the class instance). The rest of the arguments must match the arguments of the
task that the method is for. For example, the task ('get', b1) has a method "tm_get(state, b1)", as shown below.
"""
from typing import List

import numpy as np

from ipyhop import Methods
from examples.racetrack.search.racetrack import crash, intersect, goal_test
from examples.racetrack.search.racetrack import main as search
from examples.racetrack.search.sample_heuristics import h_esdist
import numpy


# ******************************************        Helper Functions        ****************************************** #

# ******************************************        Method Definitions        **************************************** #

# Create a IPyHOP Methods object. A Methods object stores all the methods defined for the planning domain.
methods = Methods()

def tm_finish_at(state, f_line):
    return [ ( "generate_visibility_graph", f_line ), ( "hill_climb", f_line ) ]
    # strategy = "gbf"
    # h = h_esdist
    # loc = state.loc
    # v = state.v
    # walls = [ *state.walls ]
    # problem = [ loc, [*f_line], walls ]
    # path_onwards = search( problem, strategy, h, v_0=v, draw=1)
    # # use first action in list until at f_line with v= (0,0)
    # if goal_test( path_onwards[ 1 ], f_line ):
    #     return [ ( "set_v", path_onwards[ 1 ][ 1 ]  ) ]
    # else:
    #     return [ ( "set_v", path_onwards[ 1 ][ 1 ]  ), ( "finish_at", f_line ) ]


methods.declare_task_methods( "finish_at", [tm_finish_at])

# generate point grid
def tm_generate_visibility_graph(state, f_line):
    # get all points in problem
    loc = state.loc
    walls = state.walls
    wall_points = set()
    for line in walls:
        wall_points.add( line[ 0 ] )
        wall_points.add( line[ 1 ] )
    points = { loc, *f_line, *wall_points }
    point_array = np.array( [ np.array( pt ) for pt in points ] )
    # get bounding box
    x_min, y_min = np.min( point_array, axis=0 )
    x_max, y_max = np.max( point_array, axis=0 )
    print( ( x_max - x_min+1 ) * (y_max-y_min+1))
    # generate visibility graph for all points in bounding box
    vis_graph = dict()

    start_pt = loc
    unexpanded_pts = [ start_pt ]
    visited_pts = set()
    # continue expansion until all nodes reachable by start
    while unexpanded_pts != []:
        print(len(visited_pts))
        curr_pt = unexpanded_pts.pop()
        visited_pts.add( curr_pt )
        vis_graph[ curr_pt ] = set()
        rng = np.random.default_rng()
        N = 1
        # check each int point in bounding box for visibility
        for i in range( x_min, x_max ): #( rng.choice( x_max-x_min, size=( x_max-x_min ) // N, replace=False ) + x_min ):
            for j in range( y_min, y_max ): #( rng.choice( y_max-y_min, size=( y_max-y_min ) // N, replace=False ) + y_min ):
                pt = ( i, j )
                # remove duplicate operations
                if pt in visited_pts:
                    continue
                move = ( curr_pt, pt )
                # visible if move would not cause crash
                if not crash( move, walls ):
                    vis_graph[ curr_pt ].add( pt )
                    if pt in vis_graph.keys():
                        vis_graph[ pt ].add( curr_pt )
                    else:
                        vis_graph[ pt ] = { curr_pt }
                    # add node to unexpanded if not previously encountered
                    if pt not in visited_pts and pt not in unexpanded_pts:
                        unexpanded_pts.append( pt )
    # no path exists
    if f_line[ 0 ] not in vis_graph.keys():
        return
    # iterate over points in visibility graph to get minimum distance to
    # create dict of min distance to start point of finish line
    dist_dict = dict()
    # start with points directly reachable by finish line start
    unexpanded_pts = [ *f_line ]
    dist_dict[ f_line[ 0 ] ] = 0
    dist_dict[ f_line[ 1 ] ] = 0
    visited_pts = set()
    # continue expansion until all nodes have min cost
    while unexpanded_pts != []:
        curr_pt = unexpanded_pts.pop()
        visited_pts.add( curr_pt )
        # get visible points
        vis_points = vis_graph[ curr_pt ]
        # get distance to visible points
        dist_to = { pt: np.sqrt( np.power( pt[ 0 ] - curr_pt[ 0 ], 2 ) + np.power( pt[ 1 ] - curr_pt[ 1 ], 2 ) )
                      for pt in vis_points }
        dist_from = dist_dict[ curr_pt ]
        # update dist_dict for all visible points
        for pt in vis_points:
            dist = dist_to[ pt ] + dist_from
            if pt in dist_dict.keys():
                dist_dict[ pt ] = min( dist_dict[ pt ], dist )
            else:
                dist_dict[ pt ] = dist
                unexpanded_pts.append( pt )
    state.vis_graph = vis_graph
    state.dist_dict = dist_dict
    return []

methods.declare_task_methods( "generate_visibility_graph", [tm_generate_visibility_graph])

def tm_hill_climb( state, f_line ):
    loc = state.loc
    v = state.v
    vis_graph = state.vis_graph
    dist_dict = state.dist_dict
    # visible points
    vis_points = vis_graph[ loc ]
    print( vis_points )
    pos_next_attitude = dict()
    # points that can be achieved given current v, loc and visibility
    for dv_x in [ -1, 0, 1 ]:
        for dv_y in [ -1, 0, 1 ]:
            new_v = ( v[ 0 ] + dv_x, v[ 1 ] + dv_y )
            new_loc = ( loc[ 0 ] + new_v[ 0 ], loc[ 1 ] + new_v[ 1 ] )
            print(new_loc)
            print(new_loc in vis_points)
            if new_loc in vis_points:
                pos_next_attitude[ new_loc ] = new_v
    # print(pos_next_attitude)
    if len( pos_next_attitude ) == 0:
        return
    # get minimal distance
    dist = np.inf
    min_s = np.inf
    for point, velocity in pos_next_attitude.items():
        # get speed
        s = np.sqrt( np.power( velocity[ 0 ], 2 ) + np.power( velocity[ 1 ], 2 ) )
        # go towards with point with lowest distance to f_line
        # tie break favoring lowest speed
        if dist_dict[ point ] < dist or ( dist_dict[ point ] == dist and s < min_s ):
            target_point = point
            min_s = s
    action = ( "set_v", pos_next_attitude[ point ] )
    if goal_test( ( target_point, pos_next_attitude[ target_point ] ), f_line ):
        return [ action ]
    else:
        return [ action, ( "hill_climb", f_line ) ]

methods.declare_task_methods( "hill_climb", [tm_hill_climb] )









# # have v = (0,0) and loc = dest
# def tm_finish_at( state, f_line ):
#     f_start = f_line[ 0 ]
#     f_end = f_line[ 1 ]
#     dest = ( f_start[ 0 ] + f_end[ 0 ] // 2, f_start[ 1 ] + f_end[ 1 ] // 2)
#     state.f_line = f_line
#     return [("move_close",dest),("park_at",dest)]
#
#
# methods.declare_task_methods('finish_at', [tm_finish_at])
#
# # move car close enough to finish that there are no obstructions
# # no obstruction between loc and dest, attempt to move directly towards
# def tm_move_direct_to( state, dest ):
#     dest_x, dest_y = dest
#     loc = state.loc
#     loc_x, loc_y = loc
#     v = state.v
#     v_x, v_y = v
#     walls = state.walls
#     # prioritize velocity increase in direction of greatest difference
#     strong_axis_index = 0 if ( dest_x - loc_x ) ** 2 >= ( dest_x - loc_x ) else 1
#     weak_axis_index = 1 if strong_axis_index == 0 else 0
#     # strong axis velocity change
#     # increase velocity
#     if dest[ strong_axis_index ]- loc[ strong_axis_index ] > 0:
#         dv = 1
#     # maintain velocity
#     elif dest[ strong_axis_index ] == loc[ strong_axis_index ]:
#         dv = 0
#     # decrease velocity
#     else:
#         dv = -1
#     # crash check
#     dv_tup = [ 0, 0 ]
#     dv_tup[ strong_axis_index ] += dv
#     dv_tup = tuple( dv_tup )
#     new_loc = [ *loc ]
#     new_loc[ strong_axis_index ] += dv
#     new_loc = tuple( new_loc )
#     move = ( loc, new_loc )
#     if not ( dv == 0 or crash( move, walls ) ):
#         # enter parking phase
#         if not crash( ( new_loc, dest ), walls ):
#             return [ ( "change_v", *dv_tup) ]
#         # continue move phase
#         else:
#             return [ ( "change_v", *dv_tup ), ( "move_close", dest ) ]
#     # weak axis velocity change
#     # increase velocity
#     if dest[ weak_axis_index ] - loc[ weak_axis_index ] > 0:
#         dv = 1
#     # maintain velocity
#     elif dest[ weak_axis_index ] == loc[ weak_axis_index ]:
#         dv = 0
#     # decrease velocity
#     else:
#         dv = -1
#     # crash check
#     dv_tup = [ 0, 0 ]
#     dv_tup[ weak_axis_index ] += dv
#     dv_tup = tuple( dv_tup )
#     new_loc = [ *loc ]
#     new_loc[ weak_axis_index ] += dv
#     new_loc = tuple( new_loc )
#     move = ( loc, new_loc )
#     # NOTE that here we allow for no velocity change
#     if not crash( move, walls ):
#         # enter parking phase
#         if not crash( (new_loc, dest ), walls ):
#             return [ ( "change_v", *dv_tup ) ]
#         # continue move phase
#         else:
#             return [ ( "change_v", *dv_tup ), ( "move_close", dest ) ]
#
# # an obstacle exists between racecar and destination
# # try and set velocity parallel to wall
# def tm_follow_wall_parallel( state, dest ):
#     loc = state.loc
#     loc_x, loc_y = loc
#     v = state.v
#     v_x, v_y = v
#     walls = state.walls
#     move = ( loc, dest )
#     # iterate over walls
#     for wall in walls:
#         # follow along wall with first intersection
#         if intersect( move, wall ):
#             wall_start, wall_end = wall
#             d_wall = ( wall_end[ 0 ] - wall_start[ 0 ], wall_end[ 1 ] - wall_start[ 1 ] )
#             sub_dest = ( loc[ 0 ] + d_wall[ 0 ], loc[ 1 ] + d_wall[ 1 ] )
#             plan = tm_move_direct_to( state, sub_dest )
#             # move in direction of wall one step if possible
#             if plan != None:
#                 return [ plan[ 0 ], ( "move_close", dest ) ]
#
# # an obstacle exists between racecar and destination
# # try and set velocity antiparallel to wall
# def tm_follow_wall_antiparallel( state, dest ):
#     loc = state.loc
#     loc_x, loc_y = loc
#     v = state.v
#     v_x, v_y = v
#     walls = state.walls
#     move = ( loc, dest )
#     # iterate over walls
#     for wall in walls:
#         # follow along wall with first intersection
#         if intersect( move, wall ):
#             wall_start, wall_end = wall
#             d_wall = ( wall_end[ 0 ] - wall_start[ 0 ], wall_end[ 1 ] - wall_start[ 1 ] )
#             sub_dest = ( loc[ 0 ] - d_wall[ 0 ], loc[ 1 ] - d_wall[ 1 ] )
#             plan = tm_move_direct_to( state, sub_dest )
#             # move in direction of wall one step if possible
#             if plan != None:
#                 return [ plan[ 0 ], ( "move_close", dest ) ]
#
# methods.declare_task_methods( 'move_close', [ tm_move_direct_to, tm_follow_wall_parallel, tm_follow_wall_antiparallel ] )
#
# # we now have an unobstructed path to goal
# # guide racecar towards goal while slowing down to stop at goal
# def tm_park_at( state, dest ):
#     v = state.v
#     loc = state.loc
#     plan = tm_move_direct_to( state, dest )
#     # guide car towards destination, if obstruction occurs then use long distance protocal again
#     if plan != None:
#         _, dv_x, dv_y = plan[ 0 ]
#         new_v = ( v[ 0 ] + dv_x, v[ 1 ] + dv_y )
#         new_loc = ( loc[ 0 ] + new_v[ 0 ], loc[ 1 ] + new_v[ 1 ] )
#         f_line = state.f_line
#         if goal_test( ( new_loc, new_v ), f_line ):
#             return [ plan[ 0 ] ]
#         else:
#             return [ plan[ 0 ], ( "park_at", dest ) ]
#     else:
#         return [ ( "move_close",dest ),( "park_at", dest ) ]
#
# methods.declare_task_methods( 'part_at', [ tm_park_at ] )
# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError("Test run / Demo routine for Blocks World Mehthods isn't implemented.")

"""
Author(s): Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
Organization: University of Maryland at College Park
"""