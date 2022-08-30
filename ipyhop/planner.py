#!/usr/bin/env python
"""
File Description: File used for definition of IPyHOP Class.
"""

# ******************************************    Libraries to be imported    ****************************************** #
from __future__ import print_function, division
from itertools import count
from typing import List, Tuple, Union, Optional
from ipyhop.methods import Methods
from ipyhop.actions import Actions
from ipyhop.state import State
from ipyhop.mulitgoal import MultiGoal
from networkx import DiGraph, dfs_preorder_nodes, descendants, is_tree, ancestors
from copy import deepcopy


# ******************************************    Class Declaration Start     ****************************************** #
class IPyHOP(object):
    """
    IPyHOP uses HTN methods to decompose tasks into smaller and smaller subtasks, until it finds tasks that
    correspond directly to actions.

    *   planner = IPyHOP(methods, actions) tells IPyHOP to create a IPyHOP planner object.
        To plan using the planner, you should use planner.plan(state, task_list).
    """

    def __init__(self, methods: Methods, actions: Actions):
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

        self._verbose = 0

    _t_type = List[Tuple[str]]
    _m_type = Optional[Methods]
    _op_type = Optional[Actions]
    _p_type = Union[List[Tuple[str]], bool]

    # ******************************        Class Method Declaration        ****************************************** #
    def plan(self, state: State, task_list: _t_type, methods: _m_type = None, actions: _op_type = None,
             verbose: Optional[int] = 0) -> _p_type:
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

        if self._verbose > 0:
            run_info = '**IPyHOP, verbose = {verbosity}: **\n\tstate = {state}\n\ttasks/goals = {task_list}.'
            print(run_info.format(verbosity=self._verbose, state=self.state.__name__, task_list=task_list))

        self.sol_plan = []
        self.sol_tree = DiGraph()

        _id = 0
        parent_node_id = _id
        self.sol_tree.add_node(_id, info=('root',), type='D', status='NA')
        _id = self._add_nodes_and_edges(_id, self.task_list)

        self.iterations, _ = self._planning(parent_node_id)
        assert is_tree(self.sol_tree), "Error! Solution graph is not a tree."

        # Store the planning solution as a list of actions to be executed.
        for node_id in dfs_preorder_nodes(self.sol_tree, source=0):
            if self.sol_tree.nodes[node_id]['type'] == 'A':
                self.sol_plan.append( self.sol_tree.nodes[node_id]['info'] )

        # print(self.sol_tree.nodes[0])
        return self.sol_plan

    # ******************************        Class Method Declaration        ****************************************** #
    def _planning(self, sub_graph_root_node_id: int, verbose: Optional[int]=0):

        _iter = 0
        parent_node_id = sub_graph_root_node_id
        marked_node_id = None
        for _iter in count(0):
            if parent_node_id in ancestors( self.sol_tree, sub_graph_root_node_id ):
                break
            curr_node_id = None
            # Get the first Open node from the immediate successors of parent node. (using BFS)
            for node_id in self.sol_tree.successors( parent_node_id ):
                if self.sol_tree.nodes[node_id]['status'] == 'O':
                    curr_node_id = node_id
                    if marked_node_id == None:
                        marked_node_id = curr_node_id
                    if self._verbose > 1:
                        print('Iteration {}, Refining node {}.'.format(
                            _iter, repr(self.sol_tree.nodes[node_id]['info'])))
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
        # return iteration count and reachable most bottom-left node
        # print("THIS")
        # print(curr_node_id)
        # print(parent_node_id)
        # print(self.sol_tree.nodes[parent_node_id]["info"])
        # print( self.sol_tree.nodes[ next( dfs_preorder_nodes(self.sol_tree,source=parent_node_id) ) ][ "info"])
        # print( [ (x,self.sol_tree.nodes[x]["info"]) for x in self.sol_tree.successors(parent_node_id) ] )
        # print("THIS END")
        # print( "ID_EXAMINE")
        # print( prev_node_id )
        # print( curr_node_id )
        # print( [*dfs_preorder_nodes( self.sol_tree )] )
        # print( sub_graph_root_node_id )
        # print( parent_node_id )
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
            # If methods are available for refining the task, use them.
            while curr_node[ 'available_methods' ] != [ ]:
                # get method instance
                if curr_node[ 'selected_method_instances' ] == None:
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
                # If methods are available for refining the goal, use them.
                while curr_node[ 'available_methods' ] != [ ]:
                    # get method instance
                    if curr_node[ 'selected_method_instances' ] == None:
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
                # If methods are available for refining the multigoal, use them.
                while curr_node[ 'available_methods' ] != [ ]:
                    # get method instance
                    if curr_node[ 'selected_method_instances' ] == None:
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
    def replan(self, state: State, action_position: int, verbose: Optional[int] = 0) -> _p_type:
        sol_tree = self.sol_tree
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

        self.sol_plan = act_plan
        return act_plan, exec_plan_index
        # return self.sol_plan

    # ******************************        Class Method Declaration        ****************************************** #
    def _add_nodes_and_edges(self, parent_node_id: int, children_node_info_list: List[Tuple[str]]):
        _id = None
        for child_node_info in children_node_info_list:
            _id = self.get_next_id()
            if isinstance(child_node_info, MultiGoal):  # equivalent to type(child_node_info) == MultiGoal
                relevant_methods = self.methods.multigoal_method_dict[child_node_info.goal_tag]
                self.sol_tree.add_node(_id, info=child_node_info, type='M', status='O', state=None,
                                       selected_method=None, available_methods=[*relevant_methods],
                                       methods=relevant_methods, selected_method_instances=None )
                self.sol_tree.add_edge(parent_node_id, _id)
            elif child_node_info[0] in self.methods.task_method_dict:
                relevant_methods = self.methods.task_method_dict[child_node_info[0]]
                self.sol_tree.add_node(_id, info=child_node_info, type='T', status='O', state=None,
                                       selected_method=None, available_methods=[*relevant_methods],
                                       methods=relevant_methods, selected_method_instances=None)
                self.sol_tree.add_edge(parent_node_id, _id)
            elif child_node_info[0] in self.actions.action_dict:
                action = self.actions.action_dict[child_node_info[0]]
                self.sol_tree.add_node(_id, info=child_node_info, type='A', status='O', action=action)
                self.sol_tree.add_edge(parent_node_id, _id)
            elif child_node_info[0] in self.methods.goal_method_dict:
                relevant_methods = self.methods.goal_method_dict[child_node_info[0]]
                self.sol_tree.add_node(_id, info=child_node_info, type='G', status='O', state=None,
                                       selected_method=None, available_methods=[*relevant_methods],
                                       methods=relevant_methods, selected_method_instances=None)
                self.sol_tree.add_edge(parent_node_id, _id)

        if self.sol_tree.nodes[parent_node_id]['type'] == 'G':
            _id = self.get_next_id()
            self.sol_tree.add_node(_id, info='VerifyGoal', type='VG', status='O')
            self.sol_tree.add_edge(parent_node_id, _id)
        elif self.sol_tree.nodes[parent_node_id]['type'] == 'M':
            _id = self.get_next_id()
            self.sol_tree.add_node(_id, info='VerifyMultiGoal', type='VM', status='O')
            self.sol_tree.add_edge(parent_node_id, _id)

        return _id

    # ******************************        Class Method Declaration        ****************************************** #
    def _post_failure_modify(self, fail_node_id):

        rev_pre_ord_nodes = reversed(list(dfs_preorder_nodes(self.sol_tree, source=0)))

        for node_id in rev_pre_ord_nodes:

            c_node = self.sol_tree.nodes[node_id]
            c_node['status'] = 'O'

            if node_id == fail_node_id:
                break

            c_type = c_node['type']
            if c_type == 'T' or c_type == 'G' or c_type == 'M':
                c_node['state'] = None
                c_node['selected_method'] = None
                c_node['available_methods'] = [*c_node['methods']]
                c_node[ "selected_method_instances" ] = None
                descendant_list = list(descendants(self.sol_tree, node_id))
                self.sol_tree.remove_nodes_from(descendant_list)

        max_id = -1
        for node_id in self.sol_tree.nodes:
            if node_id >= max_id:
                max_id = node_id + 1
            if 'state' in self.sol_tree.nodes[node_id]:
                if self.sol_tree.nodes[node_id]['status'] == 'C':
                    self.sol_tree.nodes[node_id]['state'] = self.state.copy()
                else:
                    self.sol_tree.nodes[node_id]['state'] = None

        return max_id

    # ******************************        Class Method Declaration        ****************************************** #
    def _backtrack(self, p_node_id: int, c_node_id: int, verbose: Optional[int] = 0 ):
        c_node = self.sol_tree.nodes[c_node_id]
        c_type = c_node['type']
        if c_type == 'T' or c_type == 'G' or c_type == 'M':
            c_node['state'] = None
            c_node['selected_method'] = None
            c_node['available_methods'] = [*c_node['methods']]
            c_node[ "selected_method_instances" ] = None

        dfs_list = list(dfs_preorder_nodes(self.sol_tree, source=p_node_id))
        for node_id in reversed(dfs_list):
            node = self.sol_tree.nodes[node_id]
            if node['status'] == 'C':
                node['status'] = 'O'
                descendant_list = list(descendants(self.sol_tree, node_id))
                if descendant_list:
                    self.sol_tree.remove_nodes_from(descendant_list)
                    p_node_id = next(self.sol_tree.predecessors(node_id))
                    return p_node_id, node_id
                if 'state' in node:
                    node['state'] = None

        self.sol_tree.remove_nodes_from(list(descendants(self.sol_tree, 0)))
        return 0, 0



    # ******************************        Class Method Declaration        ****************************************** #
    def _goals_not_achieved(self, multigoal_node_id):
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
            if curr_state == None:
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
    def is_dependency_of(self, node_1_id: int, node_2_id: int) -> bool:
        """
                returns True if node_2 occurs after node_1 in preorder sort or is node_1

        """
        # get tree in preorder
        preorder_nodes = dfs_preorder_nodes( 0 )
        if preorder_nodes.index( node_1_id ) <= preorder_nodes.index( node_1_id ):
            return True


# ******************************************    Class Declaration End       ****************************************** #
# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError("Test run / Demo routine for IPyHOP isn't implemented.")

"""
Author(s): Yash Bansod and Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
"""
