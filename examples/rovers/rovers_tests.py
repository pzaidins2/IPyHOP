#!/usr/bin/env python
"""
File Description: Rovers testing file. Run this file to create Rovers test data.
"""

# ******************************************    Libraries to be imported    ****************************************** #
from examples.rovers.domain.actions import actions
from examples.rovers.domain.methods import methods
from examples.rovers.domain.deviations import deviation_handler
from ipyhop import IPyHOP
from ipyhop.planner_old import IPyHOP_Old
from ipyhop.actor import Actor
from ipyhop.mc_executor import MonteCarloExecutor
import os
from examples.rovers.rovers_example import init_rovers
import numpy as np
import time
import matplotlib.pyplot as plt
from multiprocessing import Pool, cpu_count
import random
import cProfile

def run_experiment( i, j, k, problem_file_path ):
    problem_file = open( problem_file_path, "r" )
    problem_str = problem_file.read()
    problem_file.close()

    # init actor
    planner_type = [ IPyHOP, IPyHOP_Old ]
    planner_type = planner_type[ i ]
    planner = planner_type( methods, actions )
    state_0, goal_a, dev_hand = init_rovers( problem_str, actions, methods, deviation_handler )
    mc_executor = MonteCarloExecutor( actions, dev_hand )
    actor = Actor( planner, mc_executor )

    # planning and acting
    try:
        start_time = time.process_time()
        history = actor.complete_to_do( state_0, [ goal_a ] )
        time_elapsed = time.process_time() - start_time
        # metrics
        iteration_count = planner.iterations
        cpu_time = time_elapsed
        action_count = len( history )
        print( ( ( i, j, k ), ( iteration_count, cpu_time, action_count )  ) )
        return ( ( i, j, k ), ( iteration_count, cpu_time, action_count )  )
    except:
        print( "\nEXCEPTION OCCURRED: " + str( ( i, j, k ) ) + "\n" )
        return ( ( i, j, k ), ( -1, -1, -1 ) )
def main():
    problem_file_names = filter( lambda x: "pddl" in x, os.listdir( "problems" ) )
    problem_paths = [ "problems/" + x for x in problem_file_names ]
    N = 1000
    M = len( problem_paths )
    P = 2
    metrics = np.ndarray( ( P, N, M, 3 ) )

    print( metrics.shape )
    args = []
    for i in range( P ):
        for j in range( N ):
            for k in range( M ):
                args.append( ( i, j, k, problem_paths[ k ] ) )
    # i = 0
    # for j in range( N ):
    #     for k in range( M ):
    #         args.append( (i, j, k, problem_paths[ k ]) )
    # with cProfile.Profile() as pr:
    #     for exp_set in args:
    #         run_experiment( *exp_set )
    #
    # pr.print_stats()

    with Pool( processes=cpu_count() ) as pool:
        output = pool.starmap_async( run_experiment, args, chunksize=1 )
        while True:
            if output.ready():
                break
            print( str( round( 100 - 100 * output._number_left / len( args ), 3 ) ) + " %" )
            time.sleep( 60 )
    for exp in output.get():
        metrics[ exp[ 0 ] ] = np.asarray( exp[1] )

    new_iteration_count = metrics[ 0, :, :, 0 ]
    new_cpu_time = metrics[ 0, :, :, 1 ]
    new_action_count = metrics[ 0, :, :, 2 ]
    old_iteration_count = metrics[ 1, :, :, 0 ]
    old_cpu_time = metrics[ 1, :, :, 1 ]
    old_action_count = metrics[ 1, :, :, 2 ]

    # save to csv
    np.savetxt( "new_rovers_iteration_count.csv", new_iteration_count, delimiter="," )
    np.savetxt( "new_rovers_cpu_time.csv", new_cpu_time, delimiter="," )
    np.savetxt( "new_rovers_action_count.csv", new_action_count, delimiter="," )
    np.savetxt( "old_rovers_iteration_count.csv", old_iteration_count, delimiter="," )
    np.savetxt( "old_rovers_cpu_time.csv", old_cpu_time, delimiter="," )
    np.savetxt( "old_rovers_action_count.csv", old_action_count, delimiter="," )

    # load csv
    new_iteration_count = np.genfromtxt( "new_rovers_iteration_count.csv", delimiter="," )
    new_cpu_time = np.genfromtxt( "new_rovers_cpu_time.csv", delimiter="," )
    new_action_count = np.genfromtxt( "new_rovers_action_count.csv", delimiter="," )
    old_iteration_count = np.genfromtxt( "old_rovers_iteration_count.csv", delimiter="," )
    old_cpu_time = np.genfromtxt( "old_rovers_cpu_time.csv", delimiter="," )
    old_action_count = np.genfromtxt( "old_rovers_action_count.csv", delimiter="," )

    N, M = new_iteration_count.shape

    # mean
    new_mean_iteration_count = np.mean( new_iteration_count, axis=0 )
    new_mean_cpu_time = np.mean( new_cpu_time, axis=0 )
    new_mean_action_count = np.mean( new_action_count, axis=0 )
    old_mean_iteration_count = np.mean( old_iteration_count, axis=0 )
    old_mean_cpu_time = np.mean( old_cpu_time, axis=0 )
    old_mean_action_count = np.mean( old_action_count, axis=0 )

    # standard error
    new_err_iteration_count = np.std( new_iteration_count, axis=0 ) / np.sqrt( N )
    new_err_cpu_time = np.std( new_cpu_time, axis=0 ) / np.sqrt( N )
    new_err_action_count = np.std( new_action_count, axis=0 ) / np.sqrt( N )
    old_err_iteration_count = np.std( old_iteration_count, axis=0 ) / np.sqrt( N )
    old_err_cpu_time = np.std( old_cpu_time, axis=0 ) / np.sqrt( N )
    old_err_action_count = np.std( old_action_count, axis=0 ) / np.sqrt( N )

    x = np.asarray( [ i + 1 for i in range( M ) ] )

    print( new_iteration_count )
    bar_width = 0.2
    c_0 = "red"
    c_1 = "blue"
    c_2 = "black"
    plt.figure(0)
    ax = plt.subplot( 3, 1, 1 )
    title = "80% Failure Rate in Rovers Domain"
    plt.title( title )
    # plt.bar( x, new_mean_iteration_count, yerr=new_err_iteration_count, width=bar_width, label="new" )
    # plt.bar( x + bar_width, old_mean_iteration_count, yerr=old_err_iteration_count, width=bar_width, label="old"  )
    bp_0 = ax.boxplot( new_iteration_count, positions=x - 1.1 * bar_width / 2, widths=bar_width,
                        notch=True, labels=x, patch_artist=True,
                        boxprops=dict( facecolor=c_0, color=c_0 ),
                        capprops=dict( color=c_0 ),
                        whiskerprops=dict( color=c_0 ),
                        flierprops=dict( color=c_0, markeredgecolor=c_0 ),
                        medianprops=dict( color=c_2 ),
                 )
    bp_1 = ax.boxplot( old_iteration_count, positions=x + 1.1 * bar_width / 2, widths=bar_width,
                        notch=True, labels=x, patch_artist=True,
                        boxprops=dict( facecolor=c_1, color=c_1 ),
                        capprops=dict( color=c_1 ),
                        whiskerprops=dict( color=c_1 ),
                        flierprops=dict( color=c_1, markeredgecolor=c_1 ),
                        medianprops=dict( color=c_2 ),
                 )
    # plt.xlabel( "Problem #" )
    plt.ylabel( "Iteration Count")
    plt.yscale( "log" )
    ax.set_xticks( x )
    ax.legend( [ bp_0[ "boxes" ][ 0 ], bp_1[ "boxes" ][ 0 ] ], [ "new", "old" ] )

    ax = plt.subplot( 3, 1, 2 )
    # plt.bar( x, new_mean_cpu_time, yerr=new_err_cpu_time, width=bar_width, label="new" )
    # plt.bar( x + bar_width, old_mean_cpu_time, yerr=old_err_cpu_time, width=bar_width, label="old" )
    bp_0 = ax.boxplot( new_cpu_time, positions=x - 1.1 * bar_width / 2, widths=bar_width,
                       notch=True, labels=x, patch_artist=True,
                       boxprops=dict( facecolor=c_0, color=c_0 ),
                       capprops=dict( color=c_0 ),
                       whiskerprops=dict( color=c_0 ),
                       flierprops=dict( color=c_0, markeredgecolor=c_0 ),
                       medianprops=dict( color=c_2 ),
                       )
    bp_1 = ax.boxplot( old_cpu_time, positions=x + 1.1 * bar_width / 2, widths=bar_width,
                       notch=True, labels=x, patch_artist=True,
                       boxprops=dict( facecolor=c_1, color=c_1 ),
                       capprops=dict( color=c_1 ),
                       whiskerprops=dict( color=c_1 ),
                       flierprops=dict( color=c_1, markeredgecolor=c_1 ),
                       medianprops=dict( color=c_2 ),
                       )
    # plt.xlabel( "Problem #" )
    plt.ylabel( "CPU Time (s)" )
    plt.yscale( "log" )
    ax.set_xticks( x )

    ax = plt.subplot( 3, 1, 3 )
    # plt.bar( x, new_mean_action_count, yerr=new_err_action_count, width=bar_width, label="new" )
    # plt.bar( x + bar_width, old_mean_action_count, yerr=old_err_action_count, width=bar_width, label="old" )
    bp_0 = ax.boxplot( new_action_count, positions=x - 1.1 * bar_width / 2, widths=bar_width,
                       notch=True, labels=x, patch_artist=True,
                       boxprops=dict( facecolor=c_0, color=c_0 ),
                       capprops=dict( color=c_0 ),
                       whiskerprops=dict( color=c_0 ),
                       flierprops=dict( color=c_0, markeredgecolor=c_0 ),
                       medianprops=dict( color=c_2 ),
                       )
    bp_1 = ax.boxplot( old_action_count, positions=x + 1.1 * bar_width / 2, widths=bar_width,
                       notch=True, labels=x, patch_artist=True,
                       boxprops=dict( facecolor=c_1, color=c_1 ),
                       capprops=dict( color=c_1 ),
                       whiskerprops=dict( color=c_1 ),
                       flierprops=dict( color=c_1, markeredgecolor=c_1 ),
                       medianprops=dict( color=c_2 ),
                       )
    plt.xlabel( "Problem #" )
    plt.ylabel( "Action Count" )
    ax.set_xticks( x )
    plt.legend()
    plt.savefig( title + ".png")
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