#!/usr/bin/env python
"""
File Description: Blocks World actions file. All the actions for Blocks World planning domain are defined here.
Derived from: PyHOP example of Blocks World domain by Dana Nau <nau@cs.umd.edu>, November 15, 2012.

Each IPyHOP action is a Python function. The 1st argument is the current state, and the others are the planning
action's usual arguments. This is analogous to how methods are defined for Python classes (where the first argument
is the class instance). For example, the function "a_pickup(state, b)" implements the planning action for the
task ('a_pickup', b).

The blocks-world actions use three state variables:
- pos[b] = block b's position, which may be 'table', 'hand', or another block.
- clear[b] = False if a block is on b or the hand is holding b, else True.
- holding['hand'] = name of the block being held by hand, or False if the hand is empty.
"""

from ipyhop import Actions


def a_pickup(state, b):
    if state.pos[b] == 'table' and state.clear[b] == True and state.holding['hand'] == False:
        state.pos[b] = 'hand'
        state.clear[b] = False
        state.holding['hand'] = b
        return state

def a_50_pickup( state, b ):
    return a_pickup( state, b )
def a_0_pickup( state, b ):
    return a_pickup( state, b )


def a_unstack(state, b, c):
    if state.pos[b] == c and c != 'table' and state.clear[b] == True and state.holding['hand'] == False:
        state.pos[b] = 'hand'
        state.clear[b] = False
        state.holding['hand'] = b
        state.clear[c] = True
        return state

def a_50_unstack(state, b, c):
    return a_unstack(state, b, c)
def a_0_unstack(state, b, c):
    return a_unstack(state, b, c)

def a_putdown(state, b):
    if state.pos[b] == 'hand':
        state.pos[b] = 'table'
        state.clear[b] = True
        state.holding['hand'] = False
        return state

def a_50_putdown( state, b ):
    return a_putdown( state, b )
def a_0_putdown( state, b ):
    return a_putdown( state, b )

def a_stack(state, b, c):
    if state.pos[b] == 'hand' and state.clear[c] == True:
        state.pos[b] = c
        state.clear[b] = True
        state.holding['hand'] = False
        state.clear[c] = False
        return state

def a_50_stack(state, b, c):
    return a_stack(state, b, c)
def a_0_stack(state, b, c):
    return a_stack(state, b, c)

# Create a IPyHOP Actions object. An Actions object stores all the actions defined for the planning domain.
actions = Actions()
actions.declare_actions(
    [*reversed( [
        a_pickup,
        a_50_pickup,
        a_0_pickup,
        a_unstack,
        a_50_unstack,
        a_0_unstack,
        a_putdown,
        a_50_putdown,
        a_0_putdown,
        a_stack,
        a_50_stack,
        a_0_stack,
    ]) ] )

action_probability = {
    "a_pickup":[1,0],
    "a_50_pickup":[0.5,0.5],
    "a_0_pickup":[0,1],
    "a_unstack": [1, 0],
    "a_50_unstack": [0.5, 0.5],
    "a_0_unstack": [0, 1],
    "a_putdown": [1, 0],
    "a_50_putdown": [0.5, 0.5],
    "a_0_putdown": [0, 1],
    "a_stack": [1, 0],
    "a_50_stack": [0.5, 0.5],
    "a_0_stack": [0, 1],
}

action_cost = {
    "a_pickup":1,
    "a_50_pickup":1,
    "a_0_pickup":1,
    "a_unstack": 1,
    "a_50_unstack": 1,
    "a_0_unstack": 1,
    "a_putdown": 1,
    "a_50_putdown": 1,
    "a_0_putdown": 1,
    "a_stack": 1,
    "a_50_stack": 1,
    "a_0_stack": 1,
}
print(len(actions.action_dict.keys()))
print(len(action_probability.keys()))
print(len(action_cost.keys()))
actions.declare_action_models(action_probability, action_cost)

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError("Test run / Demo routine for Blocks World Actions isn't implemented.")

"""
Author(s): Yash Bansod
Repository: https://github.com/YashBansod/IPyHOP
Organization: University of Maryland at College Park
"""
