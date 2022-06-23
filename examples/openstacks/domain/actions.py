#!/usr/bin/env python
"""
File Description: Openstacks (Propositional) actions file. All the actions for Openstacks planning domain are defined here.
Derived from: PyHOP example of Openstacks domain variant as described https://ipg.host.cs.st-andrews.ac.uk/challenge/#challenge

Each IPyHOP action is a Python function. The 1st argument is the current state, and the others are the planning
action's usual arguments. This is analogous to how methods are defined for Python classes (where the first argument
is the class instance). For example, the function "a_pickup(state, b)" implements the planning action for the
task ('a_pickup', b).

The state is described with following properties:
- rigid = dict of properties that should not change
    - type_dict = dict with types as keys and sets of all objects of the corresponding type as values
    - includes = dict with orders as keys and set of products needed as values
- made = dict with products as keys and boolean value representing whether product has been made
- busy = boolean value that is True when product in production else False
- making = dict with products as keys and bool indicating whether product is in production as values
- started = dict with orders as keys and bool indicating whether order has open stack as values
- delivered = dict with ( order, product ) as keys and bool representing whether product has been delivered for order as values
- stacks_open = int for number of current open stacks
- shipped = dict with orders as keys and bool indicating whether the order has shipped as values

"""

from ipyhop import Actions
from typing import List, Dict, Set

# begin product production
def start_make_product( state, p ):
    type_dict = state.rigid[ "type_dict" ]
    made = state.made
    busy = state.busy
    # type check
    if type_check( [ p ], [ "product" ], type_dict ):
        # product must not ahve been previously made and not busy
        if not( made[ p ] ) and not( busy ):
            state.busy = True
            state.making[ p ] = True
            return state

# end product production
def end_make_product( state, p ):
    type_dict = state.rigid["type_dict"]
    making = state.making
    # type check
    if type_check( [ p ], [ "product" ], type_dict):
        # must be making the product
        if making[ p ]:
            state.made[ p ] = True
            state.busy = False
            state.making[ p ] = False
            return state

# deliver product for order
def deliver_product( state, p, o ):
    type_dict = state.rigid[ "type_dict" ]
    making = state.making
    started = state.started
    includes = state.rigid[ "includes"]
    # type check
    if type_check( [ p, o ], [ "product", "order" ], type_dict ):
        # must be making the product, started the order, and the product must be in the order
        if all( [ making[ p ], started[ o ], p in includes[ o ] ] ):
            state.delivered[ o ].add( p )
            return state

# start order
def start_order( state, o ):
    type_dict = state.rigid[ "type_dict" ]
    busy = state.busy
    waiting = state.waiting
    stacks_open = state.stacks_open
    max_stacks = state.rigid[ "max_stacks" ]
    # type check
    if type_check( [ o ], [ "order" ], type_dict ):
        # not busy, waiting for order, and have available stacks
        if all( [ not( busy ), waiting[ o ], stacks_open < max_stacks ] ):
            state.waiting[ o ] = False
            state.started[ o ] = True
            state.stacks_open += 1
            return state

# close order
def ship_order( state, o ):
    type_dict = state.rigid[ "type_dict" ]
    busy = state.busy
    started = state.started
    open_stacks = state.open_stacks
    # type check
    if type_check( [ o ], [ "order" ], type_dict ):
        # not busy, started order, at least 1 open stacks
        if all( [ busy, started[ o ], open_stacks >= 1 ] ):
            state.started[ o ] =  False
            state.shipped[ o ] = True
            state.open_stacks -= 1
            return state







# Create a IPyHOP Actions object. An Actions object stores all the actions defined for the planning domain.
actions = Actions()
actions.declare_actions( [  ] )

action_probability = {
    "set_v": [ 1, 0 ]
}

action_cost = {
    "set_v": 1
}

actions.declare_action_models(action_probability, action_cost)

# ******************************************    Helper Functions            ****************************************** #
# takes list of entities to be type checked, their expected types, and a dict defining the objects for each type
# returns True if all checks are good else False
def type_check( entity_list: List[ str ], type_list: List[ str ], type_dict: Dict[ str, Set[ str ] ] ) -> bool:
    return all( map( lambda x, y: True if x in type_dict[ y ] else False, entity_list, type_list ) )


# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError("Test run / Demo routine for Racetrack Actions isn't implemented.")

"""
Author(s): Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
Organization: University of Maryland at College Park
"""