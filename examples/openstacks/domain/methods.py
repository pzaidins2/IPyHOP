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

# ******************************************    Methods                     ****************************************** #

# method to make product and deliver to all relevant orders
def tm_make_product( state, p ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    made = state.made
    busy = state.busy
    orders = type_dict[ "order" ]
    includes = rigid[ "includes" ]
    waiting= state.waiting
    delivered = state.delivered
    # type check
    if type_check( [ p ], [ "product" ], type_dict ):
        # can only make unmade products and only when not busy
        if not( made[ p ] ) and not( busy ):
            task_list = [ ]
            # get orders that include product
            rel_orders = filter( lambda x: p in includes[ x ], orders )
            # for each relevant order open if not open
            for o in rel_orders:
                if waiting[ o ]:
                    task_list.append( ( "start_order" , o ) )
            # start production
            task_list.append( ( "start_make_product", p ) )
            # deliver for all relevant orders and ship if this is last needed product for that order
            for o in rel_orders:
                task_list.append( ( "deliver_product", p, o ) )
                # find instances of delivered where order is o and product is needed for o
                o_delivered_k = filter( lambda x: x[ 0 ] == o and x[ 1 ] in includes[ o ], delivered.keys() )
                # only current product has yet to be delivered
                if all( map( lambda x: delivered[ x ] or x[ 1 ] == p, o_delivered_k ) ):
                    task_list.append( "ship_order", o )
            # end product production
            task_list.append( "end_make_product", p )
            return task_list


# Create a IPyHOP Methods object. A Methods object stores all the methods defined for the planning domain.
methods = Methods()

methods.declare_task_methods( "make_product", [ tm_make_product ] )

# method to fill order from start
def tm_fill_order( state, o ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    includes = rigid[ "includes" ]
    delivered = state.delivered
    waiting = state.waiting
    busy = state.busy
    # type check
    if type_check( [ o ], [ "order" ], type_dict ):
        # can only fill unshipped orders
        if waiting[ o ] and not( busy ):
            # get all products that o still needs
            # deliveries needed
            included_deliveries = filter( lambda x: delivered[ x ], delivered.keys() )
            missing_products = includes[ o ] - { d[ 1 ] for d in included_deliveries }
            return [ ( "make_product", p ) for p in missing_products ]

methods.declare_task_methods( "fill_order", [ tm_fill_order ] )

# method to choose which order to fill next
def tm_select_order( state ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    orders = type_dict[ "order" ]
    includes = rigid[ "includes" ]
    included_in = rigid[ "included_in"]
    shipped = state.shipped
    delivered = state.delivered
    waiting = state.waiting
    # for each unshipped order get set of orders that need that product and are waiting
    unshipped_orders = filter( lambda x: not( shipped[ x ] ), orders )
    if len( unshipped_orders ) == 0:
        return []
    waiting_orders = { *filter( lambda x: waiting[ x ], orders ) }
    order_set_dict = { o: set() for o in unshipped_orders }
    for o in unshipped_orders:
        # products that have been shipped to order
        p_have = filter( lambda x: x[ 0 ] == o and delivered[ x ], delivered.keys() )
        p_have = { x[ 1 ] for x in p_have }
        # products needed for full order
        p_includes = includes[ o ]
        # set difference
        p_needed = p_includes - p_have
        # get waiting orders that need any p in p_needed
        for p in p_needed:
            p_orders = included_in[ p ].intersection( waiting_orders )
            order_set_dict[ o ].union( p_orders )
    prioritized_orders = sorted( [ *unshipped_orders ], key=lambda x: len( x ) )
    if len( prioritized_orders ) > 0:
        return [ ( "fill_order", prioritized_orders[ 0 ] ), ( "select_task", ) ]



# method to select which product to produce next
def tm_select_product( state ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    included_in = rigid[ "included_in" ]
    orders = type_dict[ "order" ]
    made = state.made
    includes = rigid[ "includes" ]
    # products yet to be made
    demanded_products = set()
    for o in orders:
        demanded_products.union( includes[ o ] )
    # products that still need to be produced
    needed_products = [ *filter( lambda x: not( made[ x ] ), demanded_products ) ]
    if len( needed_products ) > 0:
        return []
    product_set_dict = dict()
    # get product that included in set of orders with fewest products still needed
    for p in needed_products:
        o_that_includes = included_in[ p ]
        product_set_dict[ p ] = set()
        for o in o_that_includes:
            o_products = includes[ o ].intersection( needed_products )
            product_set_dict[ p ].union( o_products )
    prioritized_products = sorted( needed_products, lambda x: len( product_set_dict[ x ] ) )
    return [ ( "make_product", p ), ( "select_task", ) ]

methods.declare_task_methods( "select_task", [ tm_select_order, tm_select_product ] )

"""
Author(s): Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
Organization: University of Maryland at College Park
"""