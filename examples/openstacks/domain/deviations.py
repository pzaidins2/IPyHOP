#!/usr/bin/env python
"""
File Description: Openstacks deviations file. All the deviations for Openstack planning domain are defined here.
Derived from: https://github.com/shop-planner/plan-repair-icaps2020-htnws/blob/master/problems/openstacks/deviation-operators.lisp

Each IPyHOP action is a Python function. The 1st argument is the current state, and the others are the planning
action's usual arguments. This is analogous to how methods are defined for Python classes (where the first argument
is the class instance). For example, the function "a_pickup(state, b)" implements the planning action for the
task ('a_pickup', b).

"""

from typing import List, Dict, Set
from examples.satellite.domain.actions import type_check
import random
from functools import partial
from networkx import dfs_preorder_nodes
from copy import deepcopy
import time

# initialization must have original solution plan and a deviation must occur exactly once
class shopfixer_deviation_handler():
    def __init__( self, init_state, actions, planner, rigid ):
        self.init_state = init_state
        self.actions = actions
        self.rigid = rigid
        self.planner = planner
        self.determine_deviation_time = 0

    def determine_deviation( self ):
        start = time.process_time_ns()
        # find all valid deviations in original plan
        # get nodes in preorder
        sol_tree = self.planner.sol_tree
        # filter to actions
        nodes = [ *filter( lambda x: sol_tree.nodes[ x ][ "type" ] == "A", dfs_preorder_nodes( sol_tree, source=0 ) ) ]
        act_deviation_pairs = []
        state = self.init_state.copy()
        # trqaverse plan getting all valid deviation at each stage
        for node in nodes:
            curr_node = sol_tree.nodes[ node ]
            for d_operator in self.deviation_operators:
                for deviation_state in d_operator( curr_node[ "info" ], state ):
                    act_deviation_pairs.append( ( curr_node[ "info" ], deviation_state ) )
            act_name = curr_node[ "info" ][ 0 ]
            act_arg = curr_node[ "info" ][ 1: ]

            state = self.actions.action_dict[ act_name ]( state, *act_arg )
        chosen_pair = random.choice( act_deviation_pairs )
        self.chosen_pair = chosen_pair
        self.has_deviated = False
        self.determine_deviation_time = time.process_time_ns() - start

    def __call__( self, act_tuple, state ):
        # print( (act_tuple, self.chosen_pair[ 0 ]) )
        if act_tuple == self.chosen_pair[ 0 ] and not self.has_deviated:
            self.has_deviated = True
            return self.chosen_pair[ 1 ]
        else:
            return state


# openstacks deviation handler
class deviation_handler( shopfixer_deviation_handler ):

    def __init__( self, init_state, actions, planner, rigid ):
        super().__init__( init_state, actions, planner, rigid )
        self.deviation_operators = [
            # original says this can make unfixable plan, but still present and no work around given
            # d_unmake_product,
            d_unship_order
        ]
        self.deviation_operators = [ partial( d_operator, rigid=self.rigid ) for d_operator in
                                     self.deviation_operators ]



def d_unmake_product( act_tuple, state, rigid ):
    made = state.made
    included_in = rigid[ "included_in" ]
    type_dict = rigid[ "type_dict" ]
    products = [ *type_dict[ "product" ] ]
    shipped = state.shipped
    # random product that is made and not shipped is unmade
    # random.shuffle( products )
    for p in products:
        if made[ p ] and all([ not( shipped[ o ] ) for o in included_in[ p ] ] ):
            new_state = state.shallow_copy()
            new_state.made = deepcopy( new_state.made )
            new_state.made[ p ] = False
            yield new_state

def d_unship_order( act_tuple, state, rigid ):
    shipped = state.shipped
    stacks_open = state.stacks_open
    max_stacks = rigid[ "max_stacks" ]
    type_dict = rigid[ "type_dict" ]
    orders = [ *type_dict[ "order" ] ]
    # unship random order
    if stacks_open < max_stacks:
        # random.shuffle( orders )
        for o in orders:
            if shipped[ o ]:
                new_state = state.shallow_copy()
                new_state.started = deepcopy( new_state.started )
                new_state.shipped = deepcopy( new_state.shipped )
                new_state.waiting = deepcopy( new_state.waiting )
                new_state.started[ o ] = False
                new_state.shipped[ o ] = False
                new_state.waiting[ o ] = True
                yield new_state




# ******************************************    Helper Functions            ****************************************** #

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError("Test run / Demo routine for Openstacks Deviations isn't implemented.")

"""
Author(s): Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
Organization: University of Maryland at College Park
"""