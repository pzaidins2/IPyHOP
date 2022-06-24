#!/usr/bin/env python
"""
File Description: Openstacks methods file. All the methods for Openstacks planning domain are defined here.
Derived from: PyHOP example of Openstacks domain variant as described https://ipg.host.cs.st-andrews.ac.uk/challenge/#challenge

Each IPyHOP method is a Python function. The 1st argument is the current state (this is analogous to Python methods,
in which the first argument is the class instance). The rest of the arguments must match the arguments of the
task that the method is for. For example, the task ('get', b1) has a method "tm_get(state, b1)", as shown below.
"""

from ipyhop import Methods
from actions import type_check

# ******************************************    Helper Functions                     ********************************* #

def ship_cost_heuristic( included_in, started, includes, made, p ):
    # set of not started orders that includes p
    includes_p_orders = included_in[ p ]
    unstarted_orders = filter( lambda x: not( started[ x ] ), includes_p_orders )
    return order_costs( started, includes, made, unstarted_orders )

def order_costs( started, includes, made, os ):
    # sum of invifdual order costs
    sum( map( lambda x: order_cost( started, includes, made, x ), os ) )

def order_cost( started, includes, made, o ):
    # product cost, add 1 if order started
    cost = product_cost( includes, made, o )
    if started[ o ]:
        cost += 1
    return cost

def product_cost( includes, made, o ):
    # number of products included in order and not made
    ps = includes[ o ]
    unmade_ps = filter( lambda x: not( made[ x ], ps ) )
    return len( unmade_ps )

# ******************************************    Methods                     ****************************************** #

def tm_make_a_product( state ):
    rigid = state.rigid
    includes = rigid[ "includes" ]
    included_in = rigid[ "included_in" ]
    started = state.started
    shipped = state.shipped
    made = state.made
    type_dict = rigid[ "type_dict" ]
    orders = type_dict[ "order" ]
    # filter products to only those that are not made and in unshipped orders
    valid_products = set()
    unshipped_orders = filter( lambda x: not( shipped[ x ] ), orders )
    for o in unshipped_orders:
        included_products = includes[ o ]
        unmade_products = { *filter( lambda x: not( made[ x ] ), included_products ) }
        valid_products.union( unmade_products )
    if len( valid_products ) == 0:
        return
    # sort products by ship cost heuristic
    valid_products = [ *valid_products ]
    valid_products.sort( key=lambda x:ship_cost_heuristic( included_in, started, includes, x ) )
    return [ ( "tm_make_product", valid_products[ 0 ] ) ]

def tm_ship_an_order( state, o ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    shipped = state.shipped
    includes = rigid[ "includes" ]
    made = state.made
    # o is order
    # o is not shipped
    # all p that o includes are made
    # there exists an open stack
    if all( [
        type_check( [ o ], [ "order" ], type_dict ),
        not( shipped[ o ] ),
        all( [ made[ p ] for p in includes[ o ] ] ),
    ] ):
        return [ ( "ship_order", o ) ]

def tm_make_product( state, p ):
    return [ ( "tm_start_orders", p ), ( "make_product", p ) ]

def tm_start_orders( state, p ):
    rigid = state.rigid
    included_in = rigid[ "included_in" ]
    started = state.started
    # start all orders that include p that are not started
    include_p_orders = included_in[ p ]
    unstarted_orders = filter( lambda x: started[ x ], include_p_orders )
    return [ ( "tm_start_an_order", o ) for o in unstarted_orders ]

def start_an_order( state, o ):
    rigid = state.rigid
    max_stacks = rigid[ "max_stacks" ]
    open_stacks = state.open_stacks
    if open_stacks < max_stacks:
        return [ ( "start_order", o ) ]

def ship_products( state, o ):
    open_stacks = state.open_stacks
    if open_stacks > 0:
        return [ ( "ship_order", o ) ]



# Create a IPyHOP Methods object. A Methods object stores all the methods defined for the planning domain.
methods = Methods()

methods.declare_task_methods( "make_product", [ ] )


"""
Author(s): Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
Organization: University of Maryland at College Park
"""