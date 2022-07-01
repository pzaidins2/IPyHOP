
from examples.satellite.domain.actions import actions
from examples.satellite.domain.methods import methods
from examples.satellite.domain.deviations import deviation_handler
from ipyhop import IPyHOP
from ipyhop.actor import Actor
from ipyhop.mc_executor import MonteCarloExecutor
import os
from examples.satellite.satellite_example import init_sat
import numpy as np
import time
import matplotlib.pyplot as plt
from multiprocessing import Pool, cpu_count
import random

def run_experiment( i, j, problem_file_path ):
    problem_file = open( problem_file_path, "r" )
    problem_str = problem_file.read()
    problem_file.close()

    # init actor
    planner = IPyHOP( methods, actions )
    state_0, goal_a = init_sat( problem_str )
    mc_executor = MonteCarloExecutor( actions, deviation_handler )
    actor = Actor( planner, mc_executor )

    # planning and acting
    start_time = time.process_time()
    history = actor.complete_to_do( state_0, [ goal_a ] )
    time_elapsed = time.process_time() - start_time

    # metrics
    iteration_count = planner.iterations
    cpu_time = time_elapsed
    action_count = len( history )
    print( ( ( i, j ), ( iteration_count, cpu_time, action_count )  ) )
    return ( ( i, j ), ( iteration_count, cpu_time, action_count )  )

def main():
    problem_file_names = filter( lambda x: "pddl" in x, os.listdir( "problems" ) )
    problem_paths = [ "problems/" + x for x in problem_file_names ]
    N = 1000
    M = len( problem_paths )
    metrics = np.ndarray( ( N, M, 3 ) )

    print( metrics.shape )
    args = []
    for i in range( N ):
        for j in range( M ):
            args.append( ( i, j, problem_paths[ j ] ) )

    with Pool( processes=cpu_count()// 2 ) as pool:
        output = pool.starmap( run_experiment, args, chunksize=1 )
    print( output )
    for exp in output:

        metrics[ exp[ 0 ] ] = np.asarray( exp[1] )

    iteration_count = metrics[ :, :, 0 ]
    cpu_time = metrics[ :, :, 1 ]
    action_count = metrics[ :, :, 2 ]

    # save to csv
    np.savetxt( "new_satellite_iteration_count.csv", iteration_count, delimiter="," )
    np.savetxt( "new_satellite_cpu_time.csv", cpu_time, delimiter="," )
    np.savetxt( "new_satellite_action_count.csv", action_count, delimiter="," )

    # load csv
    iteration_count = np.genfromtxt( "new_satellite_iteration_count.csv", delimiter="," )
    cpu_time = np.genfromtxt( "new_satellite_cpu_time.csv", delimiter="," )
    action_count = np.genfromtxt( "new_satellite_action_count.csv", delimiter="," )

    N, M = iteration_count.shape
    # for i in range( N ):
    #     for j in range( M ):
    #         # read problem
    #         problem_file_path = problem_paths[ j ]
    #         problem_file = open( problem_file_path, "r" )
    #         problem_str = problem_file.read()
    #         problem_file.close()
    #
    #         # init actor
    #         planner = IPyHOP( methods, actions )
    #         state_0, goal_a =  init_sat( problem_str )
    #         mc_executor = MonteCarloExecutor( actions, deviation_handler )
    #         actor = Actor( planner, mc_executor )
    #
    #         # planning and acting
    #         start_time = time.perf_counter()
    #         history = actor.complete_to_do( state_0, [ goal_a ] )
    #         time_elapsed = time.perf_counter() - start_time
    #
    #         # metrics
    #         iteration_count[ i, j ] = planner.iterations
    #         cpu_time[ i, j ] = time_elapsed
    #         action_count[ i, j ] = len( history )

    # mean
    mean_iteration_count = np.mean( iteration_count, axis=0 )
    mean_cpu_time = np.mean( cpu_time, axis=0 )
    mean_action_count = np.mean( action_count, axis=0 )

    # standard error
    err_iteration_count = np.std( iteration_count, axis=0 ) / np.sqrt( N )
    err_cpu_time = np.std( cpu_time, axis=0 ) / np.sqrt( N )
    err_action_count = np.std( action_count, axis=0 ) / np.sqrt( N )

    x = [ i + 1 for i in range( M ) ]

    print( iteration_count )
    plt.figure(0)
    plt.subplot( 3, 1, 1 )
    plt.bar( x, mean_iteration_count, yerr=err_iteration_count )
    # plt.xlabel( "Problem #" )
    plt.ylabel( "Iteration Count")
    plt.yscale( "log" )

    plt.subplot( 3, 1, 2 )
    plt.bar( x, mean_cpu_time, yerr=err_cpu_time )
    # plt.xlabel( "Problem #" )
    plt.ylabel( "CPU Time (s)" )
    plt.yscale( "log" )

    plt.subplot( 3, 1, 3 )
    plt.bar( x, mean_action_count, yerr=err_action_count )
    plt.xlabel( "Problem #" )
    plt.ylabel( "Action Count" )
    plt.show()



if __name__ == '__main__':
    try:
        main()
        print('\nFile executed successfully!\n')
    except KeyboardInterrupt:
        print('\nProcess interrupted by user. Bye!')

"""
Author(s): Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
Organization: University of Maryland at College Park
"""