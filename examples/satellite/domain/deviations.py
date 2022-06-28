#!/usr/bin/env python
"""
File Description: Satellite deviations file. All the deviations for Satellite planning domain are defined here.
Derived from: https://github.com/shop-planner/plan-repair-icaps2020-htnws/blob/master/problems/satellite/adlSat-pure.lisp

Each IPyHOP action is a Python function. The 1st argument is the current state, and the others are the planning
action's usual arguments. This is analogous to how methods are defined for Python classes (where the first argument
is the class instance). For example, the function "a_pickup(state, b)" implements the planning action for the
task ('a_pickup', b).

The state is described with following properties:
- rigid = dict of properties that should not change
    - type_dict = dict with types as keys and sets of all objects of the corresponding type as values
    - on_board = dict with satellites as keys and set of on board instrument as values
    - calibration_target = dict with instrument as key and direction for calibration as value
    - supports = dict with instrument as key and set of supported modes as value
- pointing = dict with satellite as key and diretion pointing as value
- power_avail = dict with satellite as key and bool indicating whether power is available as value
- power_on = dict with instrument as key and bool indicating whether instrument is turn on
- calibrated = dict with instrument as key and bool indicating whether instrument is calibrated
- have_image = dict with ( direction, mode ) as key and bool representing whether image has been taken as value

"""

from typing import List, Dict, Set
from examples.satellite.domain.actions import type_check
import random

# satellite deviation handler
def deviation_handler( act_tuple, state ):
    deviation_operators = [ d_change_direction, d_decalibration, d_power_loss ]
    d_operator = random.choice( deviation_operators )
    return d_operator( state )

# mutate random satellite pointing
def d_change_direction( state ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    pointing = state.pointing
    satellites = type_dict[ "satellite" ]
    directions = type_dict[ "direction" ]
    # deviation only valid if more than one direction exists
    if len( directions ) >= 2:
        # select random satellite
        satellites = tuple( satellites )
        s = random.choice( satellites )
        # select random direction other than satellite pointing
        d = pointing[ s ]
        directions = tuple( type_dict[ "direction" ] - { d } )
        d_new = random.choice( directions )
        # change satellite pointing
        state.pointing[ s ] = d_new
    return state

# decalibrate random instrument
def d_decalibration( state ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    instruments = type_dict[ "instrument" ]
    calibrated = state.calibrated
    calibrated_instruments = tuple( filter( lambda x: calibrated[ x ], instruments ) )
    # can only decalibrate calibrated instrument
    if len( calibrated_instruments ) > 0:
        decalibrated_instrument = random.choice( calibrated_instruments )
        state.calibrated[ decalibrated_instrument ] = False
    return state

# cause random powered instrument to lose power
def d_power_loss( state ):
    rigid = state.rigid
    type_dict = rigid[ "type_dict" ]
    instruments = type_dict[ "instrument" ]
    power_on = state.power_on
    power_on_instruments = tuple( filter( lambda x: power_on[ x ], instruments ) )
    # only powered on instruments can lose power
    if len( power_on_instruments ) > 0:
        power_loss_instrument = random.choice( power_on_instruments )
        state.power_on[ power_loss_instrument ] = False
    return state


# ******************************************    Helper Functions            ****************************************** #

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError("Test run / Demo routine for Satellites Deviations isn't implemented.")

"""
Author(s): Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
Organization: University of Maryland at College Park
"""