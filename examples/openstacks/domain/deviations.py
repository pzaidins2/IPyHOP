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

# satellite deviation handler
def deviation_handler( act_tuple, state, rigid ):
    deviation_operators = [
        d_unmake_product,
        d_unship_order
    ]
    d_operator = random.choice( deviation_operators )
    d_operator = partial( d_operator, rigid=rigid )
    return d_operator( state )

def d_unmake_product( state, rigid ):
    made = state.made
    included_in = rigid[ "included_in" ]
    type_dict = rigid[ "type_dict" ]
    products = type_dict[ "product" ]
    shipped = state.shipped
    # random product that is made and not shipped is unmade
    products = random.shuffle( [ *products ] )
    for p in products:
        if made[ p ] and all([ not( shipped[ o ] ) for o in included_in[ p ] ] ):
            state.made[ p ] = False
            break
    return state

def d_unship_order( state, rigid ):
    shipped = state.shipped
    open_stacks = state.open_stacks
    max_stacks = rigid[ "max_stacks" ]
    type_dict = rigid[ "type_dict" ]
    orders = type_dict[ "order" ]
    # unship random order
    if open_stacks < max_stacks:
        orders = random.shuffle( [ *orders ] )
        for o in orders:
            if shipped[ o ]:
                state.started[ o ] = False
                state.shipped[ o ] = False
                state.waiting[ o ] = True
                break
    return state




# ******************************************    Helper Functions            ****************************************** #

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError("Test run / Demo routine for Openstacks Deviations isn't implemented.")

"""
Author(s): Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
Organization: University of Maryland at College Park
"""