#!/usr/bin/env python
"""
File Description: Racetrack methods file. All the methods for Racetrack planning domain are defined here.
Derived from: PyHOP example of Racetrack varient domain by Dana Nau <nau@cs.umd.edu>

Each IPyHOP method is a Python function. The 1st argument is the current state (this is analogous to Python methods,
in which the first argument is the class instance). The rest of the arguments must match the arguments of the
task that the method is for. For example, the task ('get', b1) has a method "tm_get(state, b1)", as shown below.
"""
from typing import List

from ipyhop import Methods
from examples.racetrack.search.racetrack import crash, intersect, goal_test
from examples.racetrack.search.racetrack import main as search
from examples.racetrack.search.sample_heuristics import h_esdist


# ******************************************        Helper Functions        ****************************************** #

# ******************************************        Method Definitions        **************************************** #

# Create a IPyHOP Methods object. A Methods object stores all the methods defined for the planning domain.
methods = Methods()

def tm_go_from_to( state, loc, v, f_line ):
    strategy = "gbf"
    h = h_esdist
    walls = [ *state.walls ]
    problem = [ loc, [*f_line], walls ]
    path_onwards = search( problem, strategy, h, v_0=v, draw=1)
    print( "PATH IS")
    print(path_onwards)
    print("PATH END")
    # use first action in list until at f_line with v= (0,0)
    if goal_test( path_onwards[ 1 ], f_line ):
        return [ ( "set_v", *path_onwards[ 0 ], *path_onwards[ 1 ]  ) ]
    else:
        return [ ( "set_v", *path_onwards[ 0 ], *path_onwards[ 1 ]  ), ( "go_from_to", *path_onwards[ 1 ], f_line ) ]


methods.declare_task_methods( "go_from_to", [tm_go_from_to] )

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