#!/usr/bin/env python
"""
File Description: File used for definition of IPyHOP Class.
"""

# ******************************************    Libraries to be imported    ****************************************** #
from __future__ import print_function, division
from itertools import count
from typing import List, Tuple, Union, Optional, Dict

import networkx as nx

from ipyhop.methods import Methods
from ipyhop.actions import Actions
from ipyhop.state import State
from ipyhop.mulitgoal import MultiGoal
from networkx import DiGraph, dfs_preorder_nodes, descendants, is_tree, ancestors, bfs_successors
from copy import deepcopy
import re
import keyword

# ******************************************    Class Declaration Start     ****************************************** #
class IPyHOP(object):
    """
    IPyHOP uses HTN methods to decompose tasks into smaller and smaller subtasks, until it finds tasks that
    correspond directly to actions.

    *   planner = IPyHOP(methods, actions) tells IPyHOP to create a IPyHOP planner object.
        To plan using the planner, you should use planner.plan(state, task_list).
    """

    def __init__(self, methods: Methods, actions: Actions ):
        """
        IPyHOP Constructor.

        :param methods: An instance of Methods class containing the collection of methods in the planning domain.
        :param actions: An instance of Actions class containing the collection of actions in the planning domain.
        """
        self.methods = methods
        self.actions = actions
        self.state = None
        self.task_list = []
        self.sol_plan = []
        self.sol_tree = DiGraph()
        self.blacklist = set()
        self.iterations = None
        self.id_counter = 0
        self.node_expansions = 0
        self.depth_step_size=None
        self.max_depth = None
        self._verbose = 0
        # when True will perform branch cycle checking, when False will not
        self.branch_cycle_check_flag = True

    _t_type = List[Tuple[str]]
    _m_type = Optional[Methods]
    _op_type = Optional[Actions]
    _p_type = Union[List[Tuple[str]], bool]
    # group 0: node id
    # group 1: task/action string
    # group 2: child node ids
    re_shop_top_level = re.compile( r"^([0-9]+?)\s\((.+?)\)(\s->\s)?(.*?)$",
                                    flags=re.MULTILINE | re.DOTALL )
    # group 5
    re_task = re.compile( r"(.+?)(?=\s|$)",
                          flags=re.MULTILINE | re.DOTALL )



    # ******************************        Class Method Declaration        ****************************************** #
    def plan(self, state: State, task_list: _t_type, methods: _m_type = None, actions: _op_type = None,
             verbose: Optional[int] = 0, initial_max_depth: Optional[int]=None,
             depth_step_size: Optional[int]=None) -> Union[_p_type,bool]:
        """
        IPyHOP.plan(state_1, tasks) tells IPyHOP to find a plan for accomplishing the task_list (a list of tasks)
        *tasks*, starting from an initial state *state_1*, using whatever methods and actions IPyHOP was constructed
        with.

        Optionally, instances of Methods class and/or Actions class can be passed into methods and actions
        respectively to replace the methods and actions IPyHOP uses for solving the planning problem.

        Additionally, you can add an optional argument called 'verbose' that tells IPyHOP how much debugging printout
        it should provide:
            * if verbose = 0 (the default), IPyHOP returns the solution but prints nothing;
            * if verbose = 1, it prints the initial parameters and the answer;
            * if verbose = 2, it also prints a message on each iteration;
            * if verbose = 3, it also prints info about what it's computing.

        :param state: An instance of State class containing the collection of variable bindings representing
            the current/initial state in the planning problem.
        :param task_list: A list of tasks that need to be accomplished in the planning problem.
        :param methods: [Optional] An instance of Methods class containing the collection of methods in the
            planning domain.
        :param actions: [Optional] An instance of Actions class containing the collection of actions in the
            planning domain.
        :param verbose: [Optional] An integer specifying the level of verbosity for IPyHOP.
        :return:
        """
        self.state = state.copy()
        self.task_list = deepcopy(task_list)
        self.methods = self.methods if methods is None else methods
        self.actions = self.actions if actions is None else actions
        self._verbose = verbose
        self.depth_step_size=depth_step_size
        self.max_depth=initial_max_depth if initial_max_depth is not None else depth_step_size
        self.iterations = 0

        if self._verbose > 0:
            run_info = '**IPyHOP, verbose = {verbosity}: **\n\tstate = {state}\n\ttasks/goals = {task_list}.'
            print(run_info.format(verbosity=self._verbose, state=self.state.__name__, task_list=task_list))

        self.sol_plan = []
        self.sol_tree = DiGraph()

        _id = 0
        parent_node_id = _id
        self.sol_tree.add_node(_id, info=('root',), type='D', status='NA', depth=0)
        _id = self._add_nodes_and_edges(_id, self.task_list)
        # save original task id list for plan failure check
        original_task_list = [*self.sol_tree.successors(0)]

        while True:
            _iter, _ = self._planning(parent_node_id)
            self.iterations += _iter
            assert is_tree(self.sol_tree), "Error! Solution graph is not a tree."

            # Store the planning solution as a list of actions to be executed.
            for node_id in dfs_preorder_nodes(self.sol_tree, source=0):
                if self.sol_tree.nodes[node_id]['type'] == 'A':
                    self.sol_plan.append( self.sol_tree.nodes[node_id]['info'] )
            # if only root remains we need to increase max depth and try again
            if len(self.sol_tree.nodes) > 1 or depth_step_size is None:
                break
            elif verbose>0:
                print( "No solution for max depth of " + str(self.max_depth))
                print( "Increasing max depth to " + str( self.max_depth + self.depth_step_size ) )
            _id = self._add_nodes_and_edges( 0, self.task_list )


            self.max_depth += self.depth_step_size

        # print(self.sol_tree.nodes[0])
        # check for plan failure
        new_task_list = [*self.sol_tree.successors(0)]
        print(self.sol_plan)
        if new_task_list != original_task_list:
            if verbose > 0:
                print("No Plan Possible")
            return False
        else:
            return self.sol_plan

    # ******************************        Class Method Declaration        ****************************************** #
    def _planning(self, sub_graph_root_node_id: int, verbose: Optional[int]=0):

        _iter = 0
        parent_node_id = sub_graph_root_node_id
        marked_node_id = None
        for _iter in count(0):
            # root of subtree has been reached, stop
            if parent_node_id in ancestors( self.sol_tree, sub_graph_root_node_id ):
                break
            curr_node_id = None
            # Get the first Open node from the immediate successors of parent node. (using BFS)
            for node_id in self.sol_tree.successors( parent_node_id ):
                if self.sol_tree.nodes[node_id]['status'] == 'O':
                    curr_node_id = node_id
                    if marked_node_id is None:
                        marked_node_id = curr_node_id
                    if self._verbose > 1:
                        print('Iteration {}, Refining node {}.'.format(
                            _iter, repr(self.sol_tree.nodes[node_id]['info'])))
                        # print( str( [ self.sol_tree.nodes[node_id]['info'] for node_id in dfs_preorder_nodes(self.sol_tree) if self.sol_tree.nodes[node_id]["type"] == "A"] ) )
                    break
            # If Open node wasn't found from the immediate successors
            if curr_node_id is None:
                # stop iterations at sub graph root
                # print(parent_node_id ==sub_graph_root_node_id)
                if parent_node_id == sub_graph_root_node_id:
                    break
                # Set the parent_node_id as predecessor of parent_node_id if available.
                try:
                    parent_node_id = next( self.sol_tree.predecessors( parent_node_id ) )
                except StopIteration:  # if the parent_node_id has no predecessors (i.e. it is root) end refinement.
                    if self._verbose > 2:
                        print('Iteration {}, Planning Complete.'.format(_iter))
                    break
                if self._verbose > 2:
                    print('Iteration {}, Parent node modified to {}.'.format(
                        _iter, repr(self.sol_tree.nodes[parent_node_id]['info'])))
                    print('Iteration {}, Child nodes are now: {}.'.format(
                        _iter, repr([self.sol_tree.nodes[x]['info'] for x in self.sol_tree.successors(parent_node_id)])))
            # Else, it means that an Open node was found in the subgraph. Refine the node.
            else:
                curr_node_id, parent_node_id = self._node_refine( curr_node_id, parent_node_id, _iter )
            # if parent_node_id in ancestors( self.sol_tree, sub_graph_root_node_id ):
            #     break
        # return iteration count and reachable most bottom-left node in subtree
        return _iter, next( dfs_preorder_nodes( self.sol_tree, marked_node_id ) )

    # ******************************        Class Method Declaration        ****************************************** #
    def _node_refine(self, curr_node_id: int, parent_node_id: int, _iter: int ):
        self.node_expansions += 1
        curr_node = self.sol_tree.nodes[curr_node_id]
        if 'state' in curr_node:
            # If curr_node already has a value for state, it means that the algorithm backtracked to this node.
            if curr_node['state']:
                # Modify the current state as the saved state at that node.
                self.state.update(curr_node['state'].copy())
            # If curr_node doesn't have value for state, it means that the node is visited for the first time.
            else:
                # Save the current state in the node.
                curr_node['state'] = self.state.copy()
        curr_node_info = curr_node['info']

        # If current node is a Task
        if curr_node['type'] == 'T':

            subtasks = None
            # consider failure if next decomposition would exceed max depth
            # print(curr_node["depth"], self.max_depth)
            if self.max_depth is None or curr_node["depth"] < self.max_depth:
                # If methods are available for refining the task, use them.
                while curr_node[ 'available_methods' ] != [ ]:
                    # get method instance
                    if curr_node[ 'selected_method_instances' ] is None:
                        method = curr_node[ 'available_methods' ][ 0 ]
                        curr_node[ 'selected_method' ] = method
                        # create method instance generator
                        curr_node[ 'selected_method_instances' ] = method( self.state, *curr_node_info[ 1: ] )
                    try:
                        subtasks = next( curr_node[ 'selected_method_instances' ] )
                    # exhausted all instances of selected method select new method
                    except StopIteration:
                        # get next method
                        curr_node[ 'available_methods' ].pop( 0 )
                        if len( curr_node[ 'available_methods' ] ) > 0:
                            method = curr_node[ 'available_methods' ][ 0 ]
                            curr_node[ 'selected_method' ] = method
                            # create method instance generator
                            curr_node[ 'selected_method_instances' ] = method( self.state, *curr_node_info[ 1: ] )
                    if subtasks is not None:
                        curr_node[ 'status' ] = 'C'
                        _id = self._add_nodes_and_edges( curr_node_id, subtasks )
                        parent_node_id = curr_node_id
                        if self._verbose > 2:
                            print( 'Iteration {}, Task {} successfully refined'.format( _iter,
                                                                                        repr( curr_node_info ) ) )
                            print( 'Iteration {}, Parent node modified to {}.'.format(
                                _iter, repr( self.sol_tree.nodes[ parent_node_id ][ 'info' ] ) ) )
                        break
            if subtasks is None:
                parent_node_id, curr_node_id = self._backtrack(parent_node_id, curr_node_id)
                if self._verbose > 2:
                    print('Iteration {}, Task {} refinement failed'.format(_iter, repr(curr_node_info)))
                    print('Iteration {}, Backtracking to {}.'.format(
                        _iter, repr(self.sol_tree.nodes[curr_node_id]['info'])))

        # If current node is an Action
        elif curr_node['type'] == 'A':
            new_state = None
            # If the Action is not blacklisted
            if curr_node_info not in self.blacklist:
                new_state = curr_node['action'](self.state.copy(), *curr_node_info[1:])
                if new_state is None or self.branch_cyclic( new_state, curr_node_id ):
                    new_state = None
                # If Action was successful, update the state.
                if new_state is not None:
                    curr_node['status'] = 'C'
                    self.state.update(new_state)
                    if self._verbose > 2:
                        print('Iteration {}, Action {} successful.'.format(_iter, repr(curr_node_info)))
            if new_state is None:

                parent_node_id, curr_node_id = self._backtrack(parent_node_id, curr_node_id)
                if self._verbose > 2:
                    print('Iteration {}, Action {} failed.'.format(_iter, repr(curr_node_info)))
                    print('Iteration {}, Backtracking to {}.'.format(
                        _iter, repr(self.sol_tree.nodes[curr_node_id]['info'])))

        # If current node is a Goal
        elif curr_node['type'] == 'G':
            subgoals = None
            state_var, arg, desired_val = curr_node_info
            # Skip goal refinement if already achieved
            if self.state.__dict__[state_var][arg] == desired_val:
                curr_node['status'] = 'C'
                subgoals = []
                if self._verbose > 2:
                    print('Iteration {}, Goal {} already achieved'.format(_iter, repr(curr_node_info)))
            else:
                # consider failure if next decomposition would exceed max depth
                if self.max_depth is None or curr_node[ "depth" ] < self.max_depth:
                    # If methods are available for refining the goal, use them.
                    while curr_node[ 'available_methods' ] != [ ]:
                        # get method instance
                        if curr_node[ 'selected_method_instances' ] is None:
                            method = curr_node[ 'available_methods' ][ 0 ]
                            curr_node[ 'selected_method' ] = method
                            # create method instance generator
                            curr_node[ 'selected_method_instances' ] = method( self.state, *curr_node_info[ 1: ] )
                        try:
                            subgoals = next( curr_node[ 'selected_method_instances' ] )
                        # exhausted all instances of selected method select new method
                        except StopIteration:
                            # get next method
                            curr_node[ 'available_methods' ].pop( 0 )
                            if len( curr_node[ 'available_methods' ] ) > 0:

                                method = curr_node[ 'available_methods' ][ 0 ]
                                curr_node[ 'selected_method' ] = method
                                # create method instance generator
                                curr_node[ 'selected_method_instances' ] = method( self.state, *curr_node_info[ 1: ] )
                        if subgoals is not None:
                            curr_node[ 'status' ] = 'C'
                            _id = self._add_nodes_and_edges( curr_node_id, subgoals )
                            parent_node_id = curr_node_id
                            if self._verbose > 2:
                                print( 'Iteration {}, Goal {} successfully refined'.format( _iter,
                                                                                                 repr( curr_node_info ) ) )
                                print( 'Iteration {}, Parent node modified to {}.'.format(
                                    _iter, repr( self.sol_tree.nodes[ parent_node_id ][ 'info' ] ) ) )
                            break
            if subgoals is None:
                parent_node_id, curr_node_id = self._backtrack(parent_node_id, curr_node_id)
                if self._verbose > 2:
                    print('Iteration {}, Goal {} refinement failed'.format(_iter, repr(curr_node_info)))
                    print('Iteration {}, Backtracking to {}.'.format(
                        _iter, repr(self.sol_tree.nodes[curr_node_id]['info'])))

        # If current node is a MultiGoal
        elif curr_node['type'] == 'M':
            subgoals = None
            unachieved_goals = self._goals_not_achieved(curr_node_id)
            if not unachieved_goals:
                curr_node['status'] = "C"
                subgoals = []
                if self._verbose > 2:
                    print('Iteration {}, MultiGoal {} already achieved'.format(_iter, repr(curr_node_info)))
            else:
                # consider failure if next decomposition would exceed max depth
                if self.max_depth is None or curr_node[ "depth" ] < self.max_depth:
                    # If methods are available for refining the multigoal, use them.
                    while curr_node[ 'available_methods' ] != [ ]:
                        # get method instance
                        if curr_node[ 'selected_method_instances' ] is None:
                            # get next method
                            method = curr_node[ 'available_methods' ][ 0 ]
                            # print( method )
                            curr_node[ 'selected_method' ] = method
                            # create method instance generator
                            curr_node[ 'selected_method_instances' ] = method( self.state, curr_node_info )
                        try:
                            # print( curr_node[ 'selected_method_instances' ] )
                            subgoals = next( curr_node[ 'selected_method_instances' ] )
                            # print( subgoals )
                        # exhausted all instances of selected method select new method
                        except StopIteration:
                            # get next method
                            curr_node[ 'available_methods' ].pop( 0 )
                            if len( curr_node[ 'available_methods' ] ) > 0:

                                method = curr_node[ 'available_methods' ][ 0 ]
                                # print(method)
                                curr_node[ 'selected_method' ] = method
                                # create method instance generator
                                curr_node[ 'selected_method_instances' ] = method( self.state, curr_node_info )
                                # print( method( self.state, curr_node_info ) )
                                # print( [  *curr_node[ 'selected_method_instances' ] ] )
                        if subgoals is not None:
                            curr_node[ 'status' ] = 'C'
                            _id = self._add_nodes_and_edges( curr_node_id, subgoals )
                            parent_node_id = curr_node_id
                            if self._verbose > 2:
                                print( 'Iteration {}, MultiGoal {} successfully refined'.format( _iter,
                                                                                            repr( curr_node_info ) ) )
                                print( 'Iteration {}, Parent node modified to {}.'.format(
                                    _iter, repr( self.sol_tree.nodes[ parent_node_id ][ 'info' ] ) ) )
                            break
            if subgoals is None:
                parent_node_id, curr_node_id = self._backtrack(parent_node_id, curr_node_id)
                if self._verbose > 2:
                    print(
                        'Iteration {}, MultiGoal {} refinement failed'.format(_iter, repr(curr_node_info)))
                    print('Iteration {}, Backtracking to {}.'.format(
                        _iter, repr(self.sol_tree.nodes[curr_node_id]['info'])))

        elif curr_node['type'] == 'VG':
            state_var, arg, desired_val = self.sol_tree.nodes[parent_node_id]['info']
            if self.state.__dict__[state_var][arg] == desired_val:
                curr_node['status'] = "C"
            else:
                parent_node_id, curr_node_id = self._backtrack(parent_node_id, curr_node_id)
                if self._verbose > 2:
                    curr_node_info = self.sol_tree.nodes[curr_node_id]['info']
                    print('Iteration {}, Goal {} Verification failed.'.format(_iter, repr(curr_node_info)))
                    print('Iteration {}, Backtracking to {}.'.format(_iter, repr(curr_node_info)))

        elif curr_node['type'] == 'VM':
            unachieved_goals = self._goals_not_achieved(parent_node_id)
            if not unachieved_goals:
                curr_node['status'] = "C"
            else:
                parent_node_id, curr_node_id = self._backtrack(parent_node_id, curr_node_id)
                if self._verbose > 2:
                    curr_node_info = self.sol_tree.nodes[curr_node_id]['info']
                    print('Iteration {}, MultiGoal {} Verification failed.'.format(_iter,
                                                                                   repr(curr_node_info)))
                    print('Iteration {}, Backtracking to {}.'.format(_iter, repr(curr_node_info)))
        return curr_node_id, parent_node_id

    # ******************************        Class Method Declaration        ****************************************** #
    def replan(self, state: State, action_position: int, verbose: Optional[int] = 0,
               depth_step_size: Optional[int]=None) -> Union[Tuple[_p_type,int],bool]:
        """
        repairs stored solution tree for a failure occuring at action_position given
        the state of the world
        Parameters
        ----------
        state           :   State
                        world state after failure
        action_position :   int
                        index immediately after last successful plan action
        verbose         :   int
                        higher verbosity increases detail of output in stdout valid for {0,1,2,3}
        depth_step_size :   Optional[int]
                        if set will limit how deep the solution tree may expand, otherwise unlimited

        Returns
        -------
        Union[_p_type,bool]
                        if planning succeeds return a tuple with the plan at index 0 and the index where
                        execution should resume at index 1, if planning fails returns False

        """
        sol_tree = self.sol_tree
        # get root children for plan success validation
        original_task_list = [*sol_tree.successors(0)]
        # get node id of action
        dfs_node_ids = [*dfs_preorder_nodes( sol_tree )]
        # print(dfs_node_ids)
        dfs_action_node_ids = [ *filter( lambda x: sol_tree.nodes[ x ][ "type" ] == "A", dfs_node_ids ) ]
        # print(dfs_action_node_ids)
        fail_node_id = dfs_action_node_ids[ action_position ]
        # fail node should always be action so move up to parent node before start

        node_id_stack = [ next( sol_tree.predecessors( fail_node_id ) ) ]
        state_stack = [ state.copy() ]
        node_id = node_id_stack[ 0 ]
        plan = []
        exec_preorder_index = action_position
        exec_plan_index = exec_preorder_index
        # problem state and node are stored in stacks
        # if node cannot be repaired try reparing parent
        # once repair complete simulate until problem or success
        # if problem place on stack and repeat repair process
        # if at any point in repair process the previous node on the stack is descendant of current problem,
        # pop from stack and continue repair procedure from previous node
        # print(exec_plan_index )
        while node_id_stack != []:

            if verbose >= 3:
                print("Loop Head, Node Stack is: " + str( node_id_stack ) )
            # get top of stack
            node_id = node_id_stack[ 0 ]
            # root has no parent we have exhausted all methods
            if node_id == 0:
                break
            true_state = state_stack[ 0 ]
            # get parent id
            parent_id = next( sol_tree.predecessors( node_id ) )
            parent_node = sol_tree.nodes[ parent_id ]

            # print( parent_id )
            # unexpand node
            sol_tree.remove_nodes_from( descendants( sol_tree, node_id ) )
            node = sol_tree.nodes[ node_id ]
            # print(node["info"])
            node[ "status" ] = "O"
            node[ 'available_methods' ] = [ *node[ 'methods' ] ] # CHANGE
            node[ "selected_method" ] = None
            node[ "state" ] = None
            # node[ "state" ] = true_state
            node[ "selected_method_instances" ] = None # CHANGE

            # replace child with parent on stack
            node_id_stack[ 0 ] = parent_id
            # print(self.sol_tree.nodes[parent_id]["info"])

            # there exists relevant methods we have not tried
            # propagate expansion downward, backtracking if needed but never higher than current node
            if node[ "available_methods" ] != []:
                self.state = true_state.copy()
                _iter, exec_id = self._planning(parent_id ,verbose=verbose)
                self.iterations += _iter
                if node[ "status" ] == "O":
                    continue
            # deadend move up
            else:
                # check if root is reached
                # if parent_id == 0:
                #     break
                # if so return to previous node on stack else continue traversing up
                prev_node = node_id_stack[ 1 ] if len( node_id_stack ) > 1 else None
                grandparent_id = next( sol_tree.predecessors( parent_id ) )
                # do this to prevent altering precondition guarantees
                # that is we have gone far enough up the tree that the previous node will be orphaned by repair
                if prev_node in descendants( sol_tree, grandparent_id ):
                    node_id_stack.pop(0)
                    state_stack.pop(0)
                continue
            # print("HERE_0")
            # get new plan
            # plan is actions in left-right order, ignore previously executed commands unless
            # needed to complete immediate goal/task

            # don't reexecute tree branches prior to current failure point parent
            preorder_nodes = [*dfs_preorder_nodes( sol_tree )]
            exec_preorder_index = preorder_nodes.index( exec_id )
            # we care only about actions that still need to be executed
            plan = [*filter(lambda x: sol_tree.nodes[x]["type"] == "A", preorder_nodes)]
            plan_node_indices = [ *map( lambda x: preorder_nodes.index( x ), plan ) ]
            for i in range( len( plan_node_indices ) ):
                if plan_node_indices[ i ] >= exec_preorder_index:
                    exec_plan_index = i
                    break
            # print(exec_plan_index)

            # plan going forward is stored in PyHOP object

            # simulate new plan from current point
            # print("STATE")
            # print(true_state)
            act_plan = [ sol_tree.nodes[ x ][ "info" ] for x in plan ]
            sim_state, sim_index, sim_success = self.simulate_no_copy( true_state, act_plan, exec_plan_index )
            # if a problem occurs put state at failure and attempted node on stack

            if not sim_success:
                state_stack.insert( 0, sim_state )
                node_id_stack.insert( 0, next( sol_tree.predecessors( plan[ sim_index ] ) ) )
                continue
            # print( "HERE_2" )
            # plan worked
            break
        # check for solution failure
        new_task_list = [*sol_tree.successors(0)]
        # plan repair failure
        if new_task_list != original_task_list:
            return False
        # plan repair success
        else:
            self.sol_plan = act_plan
            return act_plan, exec_plan_index
        # return self.sol_plan

    # ******************************        Class Method Declaration        ****************************************** #
    def _add_nodes_and_edges(self, parent_node_id: int, children_node_info_list: List[Tuple[str]]):
        _id = None
        parent_depth=self.sol_tree.nodes[parent_node_id]["depth"]
        for child_node_info in children_node_info_list:
            _id = self.get_next_id()
            if isinstance(child_node_info, MultiGoal):  # equivalent to type(child_node_info) == MultiGoal
                relevant_methods = self.methods.multigoal_method_dict[child_node_info.goal_tag]
                self.sol_tree.add_node(_id, info=child_node_info, type='M', status='O', state=None,
                                       selected_method=None, available_methods=[*relevant_methods],
                                       methods=relevant_methods, selected_method_instances=None, depth=parent_depth+1 )
                self.sol_tree.add_edge(parent_node_id, _id)
            elif child_node_info[0] in self.methods.task_method_dict:
                relevant_methods = self.methods.task_method_dict[child_node_info[0]]
                self.sol_tree.add_node(_id, info=child_node_info, type='T', status='O', state=None,
                                       selected_method=None, available_methods=[*relevant_methods],
                                       methods=relevant_methods, selected_method_instances=None, depth=parent_depth+1)
                self.sol_tree.add_edge(parent_node_id, _id)
            elif child_node_info[0] in self.actions.action_dict:
                action = self.actions.action_dict[child_node_info[0]]
                self.sol_tree.add_node(_id, info=child_node_info, type='A', status='O', action=action, depth=parent_depth+1)
                self.sol_tree.add_edge(parent_node_id, _id)
            elif child_node_info[0] in self.methods.goal_method_dict:
                relevant_methods = self.methods.goal_method_dict[child_node_info[0]]
                self.sol_tree.add_node(_id, info=child_node_info, type='G', status='O', state=None,
                                       selected_method=None, available_methods=[*relevant_methods],
                                       methods=relevant_methods, selected_method_instances=None, depth=parent_depth+1)
                self.sol_tree.add_edge(parent_node_id, _id)

        if self.sol_tree.nodes[parent_node_id]['type'] == 'G':
            _id = self.get_next_id()
            self.sol_tree.add_node(_id, info='VerifyGoal', type='VG', status='O', depth=parent_depth+1)
            self.sol_tree.add_edge(parent_node_id, _id)
        elif self.sol_tree.nodes[parent_node_id]['type'] == 'M':
            _id = self.get_next_id()
            self.sol_tree.add_node(_id, info='VerifyMultiGoal', type='VM', status='O', depth=parent_depth+1)
            self.sol_tree.add_edge(parent_node_id, _id)

        return _id

    # ******************************        Class Method Declaration        ****************************************** #
    # NO LONGER NEEDED
    # def _post_failure_modify(self, fail_node_id):
    #
    #     rev_pre_ord_nodes = reversed(list(dfs_preorder_nodes(self.sol_tree, source=0)))
    #
    #     for node_id in rev_pre_ord_nodes:
    #
    #         c_node = self.sol_tree.nodes[node_id]
    #         c_node['status'] = 'O'
    #
    #         if node_id == fail_node_id:
    #             break
    #
    #         c_type = c_node['type']
    #         if c_type == 'T' or c_type == 'G' or c_type == 'M':
    #             c_node['state'] = None
    #             c_node['selected_method'] = None
    #             c_node['available_methods'] = [*c_node['methods']]
    #             c_node[ "selected_method_instances" ] = None
    #             descendant_list = list(descendants(self.sol_tree, node_id))
    #             self.sol_tree.remove_nodes_from(descendant_list)
    #
    #     max_id = -1
    #     for node_id in self.sol_tree.nodes:
    #         if node_id >= max_id:
    #             max_id = node_id + 1
    #         if 'state' in self.sol_tree.nodes[node_id]:
    #             if self.sol_tree.nodes[node_id]['status'] == 'C':
    #                 self.sol_tree.nodes[node_id]['state'] = self.state.copy()
    #             else:
    #                 self.sol_tree.nodes[node_id]['state'] = None
    #
    #     return max_id

    # ******************************        Class Method Declaration        ****************************************** #
    def _backtrack(self, p_node_id: int, c_node_id: int, verbose: Optional[int] = 0 ):

        c_node = self.sol_tree.nodes[c_node_id]
        c_type = c_node['type']
        # reset c_node
        if c_type == 'T' or c_type == 'G' or c_type == 'M':
            c_node['state'] = None
            c_node['selected_method'] = None
            c_node['available_methods'] = [*c_node['methods']]
            c_node[ "selected_method_instances" ] = None
        # mark succesive preorder nodes as open
        dfs_list = list(dfs_preorder_nodes(self.sol_tree, source=p_node_id))
        for node_id in reversed(dfs_list):
            node = self.sol_tree.nodes[node_id]
            if node['status'] == 'C':
                node['status'] = 'O'
                # unexpand subtree rooted at c_node
                descendant_list = list(descendants(self.sol_tree, node_id))
                if descendant_list:
                    self.sol_tree.remove_nodes_from(descendant_list)
                    p_node_id = next(self.sol_tree.predecessors(node_id))
                    return p_node_id, node_id
                if 'state' in node:
                    node['state'] = None
        # we have backtracked to root node
        self.sol_tree.remove_nodes_from(list(descendants(self.sol_tree, 0)))
        return 0, 0



    # ******************************        Class Method Declaration        ****************************************** #
    def _goals_not_achieved(self, multigoal_node_id):
        # insure that all subgoal of multigoal are achieved
        multigoal = self.sol_tree.nodes[multigoal_node_id]['info']
        unachieved = {}
        for name in vars(multigoal):
            if name == '__name__' or name == 'goal_tag':
                continue
            for arg in vars(multigoal).get(name):
                val = vars(multigoal).get(name).get(arg)
                if val != vars(self.state).get(name).get(arg):
                    # want arg_value_pairs.name[arg] = val
                    if not unachieved.get(name):
                        unachieved.update({name: {}})
                    unachieved.get(name).update({arg: val})
        return unachieved

    # ******************************        Class Method Declaration        ****************************************** #
    def simulate(self, state: State, start_ind=0) -> List:
        """
        Simulates the generated plan on the given state

        :param state: An instance of State class containing the collection of variable bindings representing
            the current in the planning problem.
        :param start_ind: An integer specifying the index of the command in the generated plan to simulate from.
        :return: A list of states that the system transitions through during simulation of the plan.
        """
        state_list = [state.copy()]
        state_copy = state.copy()
        plan = self.sol_plan[start_ind:]
        for action in plan:
            self.actions.action_dict[action[0]](state_copy, *action[1:])
            state_list.append(state_copy.copy())
        return state_list

    def simulate_no_copy(self, state: State, act_plan: List[Tuple], start_ind=0) -> List:
        """
        Simulates the generated plan on the given state without keeping intermediate states

        :param state: An instance of State class containing the collection of variable bindings representing
            the current in the planning problem.
        :param start_ind: An integer specifying the index of the command in the generated plan to simulate from.
        :return: last state not None and index in plan where this occured
        """
        prev_state = state.copy()
        curr_state = prev_state
        # print("SIMULATE")
        for i, action in enumerate( act_plan[ start_ind: ] ):
            curr_state = self.actions.action_dict[action[0]](prev_state, *action[1:])
            if curr_state is None:
                return ( prev_state, start_ind + i, False )
            prev_state = curr_state
        return ( curr_state, len( act_plan ) - 1, True )
    # ******************************        Class Method Declaration        ****************************************** #
    def blacklist_command(self, command: Tuple):
        """
        Blacklists a provided command. Blacklisted commands will fail during planning.

        :param command: A tuple representing a command instance that should be blacklisted.
        """
        self.blacklist.add(command)

    # ******************************        Class Method Declaration        ****************************************** #
    def get_next_id(self):
        """
                Used to ensure all nodes have unique id

        """
        self.id_counter += 1
        return self.id_counter

    # ******************************        Class Method Declaration        ****************************************** #
    # returns true if the new state for the node at node_id is equal to any state on the path from that node
    # to the root, else returns false
    def branch_cyclic( self, new_state: State, node_id: int ) -> bool:
        if self.branch_cycle_check_flag:
            sol_tree = self.sol_tree
            sol_tree_nodes = sol_tree.nodes
            node_ancestors = ancestors( sol_tree, node_id )
            # do want to look at root which is stateless
            node_ancestors.remove(0)
            ancestor_states = map( lambda x: sol_tree_nodes[x]["state"], node_ancestors )

            return any( new_state == a_node_state for a_node_state in ancestor_states )
        else:
            return False

    # # ******************************        Class Method Declaration        ****************************************** #
    # def is_dependency_of(self, node_1_id: int, node_2_id: int) -> bool:
    #     """
    #             returns True if node_2 occurs after node_1 in preorder sort or is node_1
    #
    #     """
    #     # get tree in preorder
    #     preorder_nodes = dfs_preorder_nodes( 0 )
    #     if preorder_nodes.index( node_1_id ) <= preorder_nodes.index( node_1_id ):
    #         return True

    # ******************************        Class Method Declaration        ****************************************** #
    #
    def hddl_plan_str( self, name_mapping: Optional[Dict[str,str]]=None ) -> str:
        """
        Get IPC plan solution tree str representation of IPyHOPPER solution tree

        Parameters
        ----------
        name_mapping    :   Optional[Dict[str,str]]
                        dictionary mapping every IPyHOPPER term to desired string representation

        Returns
        -------
        str
                        IPyHOPPER plan in IPC format using str
        """
        sol_tree = self.sol_tree
        # output header
        output_str = "==>\n"
        # plan
        preorder_node_ids = [ *dfs_preorder_nodes( sol_tree ) ]
        dfs_action_node_ids = [ *filter( lambda x: sol_tree.nodes[ x ][ "type" ] == "A", preorder_node_ids ) ]
        dfs_action_nodes = [*map( lambda x: (x, sol_tree.nodes[ x ]), dfs_action_node_ids )]
        # unpack each action into string
        for node_id, node in dfs_action_nodes:
            # id
            output_str += str(node_id) + " "
            # action name and arguements
            for arg in node[ "info" ]:
                arg_str = str( arg )
                if name_mapping is not None:
                    arg_str = name_mapping[ arg_str ]
                output_str += arg_str + " "
            output_str += "\n"
        # decomposition
        output_str += "\n"
        dfs_nonaction_node_ids = [ *filter( lambda x: sol_tree.nodes[ x ][ "type" ] != "A", preorder_node_ids ) ]
        dfs_nonaction_nodes = [ *map( lambda x: (x, sol_tree.nodes[ x ]), dfs_nonaction_node_ids ) ]
        # unpack each method into string
        for node_id, node in dfs_nonaction_nodes:
            node_info = node[ "info" ]
            # root node
            if node_id == 0:
                output_str += node_info[ 0 ] + " "
                # top level children
                for child_id in nx.neighbors(sol_tree,node_id):
                    output_str += str(child_id) + " "

            else:
                # all other nonprimitive nodes
                # id
                output_str += str( node_id ) + " "
                # method name and arguments
                for arg in node[ "info" ]:
                    arg_str = str( arg )
                    if name_mapping is not None:
                        arg_str = name_mapping[ arg_str ]
                    output_str += arg_str + " "
                # subtasks
                output_str += "-> "
                # method of decomposition
                try:
                    decomp_str = node["selected_method"].func.__name__
                except AttributeError:
                    decomp_str = node[ "selected_method" ].__name__
                if name_mapping is not None:
                    decomp_str = name_mapping[ decomp_str ]
                output_str += decomp_str + " "
                # child node ids
                for child_id in nx.neighbors(sol_tree,node_id):
                    output_str += str(child_id) + " "
            output_str += "\n"
        output_str += "<==\n"
        return output_str


    def read_SHOP( self, SHOP_sol_tree_path: str, initial_state: State ):
        """
        Replicate SHOP solution tree in IPyHOPPER

        Parameters
        ----------
        SHOP_sol_tree_path      :   str
                                file path to SHOP tree (IPC format)
        initial_state           :   State
                                state at start of the plan

        Returns
        -------

        """
        re_shop_top_level = self.re_shop_top_level
        re_task= self.re_task
        # make empty solution tree
        self.plan( { }, [ ] )
        # read in SHOP tree
        with open( SHOP_sol_tree_path, "r" ) as f:
            shop_str = f.read()
        # list of match tuples
        top_level = re_shop_top_level.findall( shop_str )
        # add nodes first
        sol_tree = self.sol_tree
        info_dict = dict()
        info_dict[ 0 ] = { "info": ("root",), "type": "D", "status": 'NA', "depth": 0 }
        # get child node ids
        child_id_set = set()
        # for each tuple add node and edges
        for str_tuple in top_level:
            # id as int
            task_id = int( str_tuple[ 0 ] )
            # find name (grab first match for name) and clean
            parameter_list = [ ]
            for i, parameter in enumerate( re_task.findall( str_tuple[ 1 ] ) ):
                parameter_list.append( clean_string( parameter ) )
            task_name = parameter_list[ 0 ]
            # build node dict
            case = "A" if str_tuple[ 2 ] == "" else "T"
            info_dict[ task_id ] = {
                "info": tuple( parameter_list ),
                "type": case,
                "status": 'C',
                "state": None,
                "depth": None
            }
            methods = self.methods
            actions = self.actions
            # make tree skeleton
            if case == "T":
                # attach correct methods
                child_ids = str_tuple[ 3 ].split()
                method_name = clean_string( child_ids.pop( 0 ) )
                # allows for partial function currying
                try:
                    # partial case
                    selected_method_name = [ *filter( lambda x: x.func.__name__ == method_name, methods.task_method_dict[ task_name ] ) ][0]
                except AttributeError:
                    # normal case
                    selected_method_name = [ *filter( lambda x: x.__name__ == method_name, methods.task_method_dict[ task_name ] ) ][0]
                except KeyError:
                    # method not found
                    raise KeyError("Input tree contains method, " + method_name +
                                   ", but no method of this name was found in the domain definition")
                except IndexError:
                    # method not found
                    raise KeyError("Input tree contains method, " + method_name +
                                   ", but no method of this name was found in the domain definition")

                info_dict[ task_id ].update(
                    {
                        "selected_method": selected_method_name,
                        "available_methods": [ *methods.task_method_dict[ task_name ] ],
                        "methods": [ *methods.task_method_dict[ task_name ] ],
                        "selected_method_instances": None,
                    }
                )
                # add children
                child_id_list = [ *map( int, child_ids ) ]
                task_from_edge_list = [ *map( lambda x: (task_id, x), child_id_list ) ]
                sol_tree.add_edges_from( task_from_edge_list )
                # any task that is a child may never be an
                child_id_set |= { *child_id_list }
            # name action
            else:
                action_func = actions.action_dict[ task_name ]
                info_dict[ task_id ].update(
                    {
                        "action": action_func,
                    }
                )
        # get all top level tasks and add as root children
        top_level_task_id_list = [ *sorted( { *sol_tree.nodes } - child_id_set - { 0 } ) ]
        root_child_edges = map( lambda x: (0, x), top_level_task_id_list )
        sol_tree.add_edges_from( root_child_edges )
        # fill in node info for all nodes
        for node_id in sol_tree.nodes:
            node = sol_tree.nodes[ node_id ]
            for k, v in info_dict[ node_id ].items():
                node[ k ] = v
        # get plan node ids
        sol_tree_nodes = sol_tree.nodes
        plan_node_ids = [*filter( lambda x: sol_tree_nodes[x]["type"] == "A", dfs_preorder_nodes(sol_tree) )]
        sol_plan = [*map( lambda x: sol_tree_nodes[x]["info"], plan_node_ids )]
        self.sol_plan = sol_plan
        # simulate state progression
        state_list = self.simulate( initial_state, start_ind=0 )
        # in reverse order assign states to ancestors
        for act_id, act_state in zip(reversed(plan_node_ids), reversed(state_list[:-1])):
            ancestor_id_set = ({*ancestors(sol_tree,act_id)} - {0}) | {act_id}
            for ancestor_id in ancestor_id_set:
                sol_tree_nodes[ancestor_id]["state"] = act_state.copy()
        # for methods without action descendants copy from left
        preorder_node_ids = [*dfs_preorder_nodes(sol_tree)]
        rev_preorder_node_ids = [*reversed(preorder_node_ids)]
        for i in range(len(preorder_node_ids)):
            node_id = rev_preorder_node_ids[i]
            # method without action descendant
            if node_id != 0 and ( "state" not in sol_tree_nodes[ node_id ].keys() or sol_tree_nodes[ node_id ][ "state" ] is None ):
                # tail end
                if node_id == rev_preorder_node_ids[0]:
                    sol_tree_nodes[ node_id ][ "state" ] = state_list[-1].copy()
                # base case
                else:
                    next_node_id = rev_preorder_node_ids[i-1]
                    sol_tree_nodes[ node_id ][ "state" ] = sol_tree_nodes[ next_node_id ][ "state" ].copy()
        # set depth
        node_depth = 0
        for parent_id, child_id_list in bfs_successors(sol_tree,0):
            node_depth = sol_tree_nodes[parent_id]["depth"] + 1
            for node_id in child_id_list:
                sol_tree_nodes[ node_id ][ "depth" ] = node_depth
        self.id_counter = max( sol_tree.nodes )
        return







# takes PDDL strings and makes them assignable to python boundVars
def clean_string( input_str: str ) -> str:
    # ? must be removed
    new_str = input_str.replace( "?", "" )
    # ! must be removed
    new_str = new_str.replace( "!", "" )
    # - must be replaced with _
    new_str = new_str.replace( "-", "__" )
    # PDDL is case insensitive so we are going to make everything lower case
    new_str = new_str.lower()
    # we cannot allow keywords
    if keyword.iskeyword(new_str):
        new_str += "___"
    # remove leading and trailing
    new_str = new_str.strip()
    return new_str

# ******************************************    Class Declaration End       ****************************************** #
# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError("Test run / Demo routine for IPyHOP isn't implemented.")

"""
Author(s): Yash Bansod and Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
"""
