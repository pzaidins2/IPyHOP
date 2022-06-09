from ipyhop.actor import Actor
from ipyhop.mc_executor import MonteCarloExecutor
from ipyhop import IPyHOP, planar_plot
from examples.rescue.domain.rescue_methods import methods
from examples.rescue.domain.rescue_actions import actions
from examples.rescue.problem.rescue_problem_1 import init_state, task_list_1, task_list_2
import numpy as np

if __name__ == '__main__':
    # runs actions with chance designated chance of failure
    # seed = np.random.randint(0,1000)
    seed = 52
    mc_executor = MonteCarloExecutor(actions,seed=seed)
    # HTN/HGN planner
    planner = IPyHOP(methods, actions)
    # simulated agent that caries out tasks and achieves goals until all items completed or success deemed impossible
    actor = Actor( planner, mc_executor )
    history = actor.complete_to_do(init_state,task_list_2,verbose=3)
    print( "Seed is: " + str( seed ) )
    graph = actor.planner.sol_tree
    planar_plot(actor.planner.sol_tree,0)