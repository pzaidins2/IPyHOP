#!/usr/bin/env python
"""
File Description: robosubs testing file. Run this file to create robosub test data.
"""

# ******************************************    Libraries to be imported    ****************************************** #
from examples.robosub.domain.robosub_mod_actions import actions
from examples.robosub.domain.robosub_mod_methods import methods
from examples.robosub.problem.robosub_mod_prob_gen import StateSampler
from ipyhop import IPyHOP
from ipyhop.planner_old import IPyHOP_Old
from ipyhop.actor import Actor
from ipyhop.mc_executor import MonteCarloExecutor
import os
import numpy as np
import time
import matplotlib.pyplot as plt
from multiprocessing import Pool, cpu_count
import random
import cProfile

def run_experiment( i, j, k ):

    # init actor
    planner_type = [ IPyHOP, IPyHOP_Old ]
    planner_type = planner_type[ i ]
    planner = planner_type( methods, actions )
    state_sampler = StateSampler( seed_val=j )
    state_0, task_list = state_sampler.sample( actions, methods, state_name="state_0" )
    mc_executor = MonteCarloExecutor( actions, seed=k )
    actor = Actor( planner, mc_executor )

    # planning and acting
    try:
        start_time = time.process_time()
        history = actor.complete_to_do( state_0, task_list )
        time_elapsed = time.process_time() - start_time
        # metrics
        iteration_count = planner.iterations
        cpu_time = time_elapsed
        action_count = len( history )
        print( ( ( i, j, k ), ( action_count, cpu_time, iteration_count )  ) )
        return ( ( i, j, k ) , ( action_count, cpu_time, iteration_count )  )
    except:
        print( "\nEXCEPTION OCCURRED: " + str( ( i, j, k ) ) + "\n" )
        return ( ( i, j, k ) , ( -1, -1, -1 ) )

if __name__ == '__main__':
    # N = 1000
    # P = 2
    # M = 1000
    # metrics = np.ndarray( (P, N, M, 3) )
    #
    # print( metrics.shape )
    # args = [ ]
    # for i in range( P ):
    #     for j in range( N ):
    #         for k in range( M ):
    #             args.append( (i, j, k) )
    #
    # # with cProfile.Profile() as pr:
    # # for exp_set in args:
    # #     run_experiment( *exp_set )
    #
    # # pr.print_stats()
    #
    # with Pool( processes=cpu_count() ) as pool:
    #     output = pool.starmap_async( run_experiment, args, chunksize=1 )
    #     while True:
    #         if output.ready():
    #             break
    #         print( str( round( 100 - 100 * output._number_left / len( args ), 3 ) ) + " %" )
    #         time.sleep( 60 )
    # for exp in output.get():
    #     metrics[ exp[ 0 ] ] = np.asarray( exp[ 1 ] )
    #
    # new_robosub_action_count = metrics[ 0, :, :, 0 ]
    # new_robosub_cpu_time = metrics[ 0, :, :, 1 ]
    # new_robosub_iteration_count = metrics[ 0, :, :, 2 ]
    # old_robosub_action_count = metrics[ 1, :, :, 0 ]
    # old_robosub_cpu_time = metrics[ 1, :, :, 1 ]
    # old_robosub_iteration_count = metrics[ 1, :, :, 2 ]
    #
    # # save to csv
    # np.savetxt( "new_robosub_action_count.csv", new_robosub_action_count, delimiter="," )
    # np.savetxt( "new_robosub_cpu_time.csv", new_robosub_cpu_time, delimiter="," )
    # np.savetxt( "new_robosub_iteration_count.csv", new_robosub_iteration_count, delimiter="," )
    # np.savetxt( "old_robosub_action_count.csv", old_robosub_action_count, delimiter="," )
    # np.savetxt( "old_robosub_cpu_time.csv", old_robosub_cpu_time, delimiter="," )
    # np.savetxt( "old_robosub_iteration_count.csv", old_robosub_iteration_count, delimiter="," )

    # load csv
    new_robosub_action_count = np.genfromtxt( "new_robosub_action_count.csv", delimiter="," )
    new_robosub_cpu_time = np.genfromtxt( "new_robosub_cpu_time.csv", delimiter="," )
    new_robosub_iteration_count = np.genfromtxt( "new_robosub_iteration_count.csv", delimiter="," )
    old_robosub_action_count = np.genfromtxt( "old_robosub_action_count.csv", delimiter="," )
    old_robosub_cpu_time = np.genfromtxt( "old_robosub_cpu_time.csv", delimiter="," )
    old_robosub_iteration_count = np.genfromtxt( "old_robosub_iteration_count.csv", delimiter="," )

    P = 2
    N, M = new_robosub_iteration_count.shape


    plt.scatter( np.mean( old_robosub_action_count, axis=1 ), np.mean( new_robosub_action_count, axis=1 ) )
    x = np.asarray( [ *range( int( min( np.min( np.mean( old_robosub_action_count, axis=1 ) ),
                                        np.min( np.mean( new_robosub_action_count, axis=1 ) ) ) ) - 1,
                              int( max( np.max( np.mean( old_robosub_action_count, axis=1 ) ),
                                        np.max( np.mean( new_robosub_action_count, axis=1 ) ) ) ) + 2 ) ] )
    plt.plot( x, x, color="black", linestyle="dashed" )
    plt.gca().set_aspect( 'equal', adjustable='box' )
    plt.xlabel( "Mean Old Action Count" )
    plt.ylabel( "Mean New Action Count" )
    plt.title( "Robosub Domain" )
    plt.show()

    plt.scatter( np.mean( old_robosub_cpu_time, axis=1 ), np.mean( new_robosub_cpu_time, axis=1 ) )
    x = np.asarray( [ *range( 0, 7 ) ] ) / 100
    plt.plot( x, x, color="black", linestyle="dashed" )
    plt.gca().set_aspect( 'equal', adjustable='box' )
    plt.xlabel( "Mean Old CPU Time (s)" )
    plt.ylabel( "Mean New CPU Time (s)" )
    plt.title( "Robosub Domain" )
    plt.show()


    plt.scatter( np.mean( old_robosub_iteration_count, axis=1 ), np.mean( new_robosub_iteration_count, axis=1 ) )
    x = np.asarray( [ *range( int( min( np.min( np.mean( old_robosub_iteration_count, axis=1 ) ), np.min( np.mean( new_robosub_iteration_count, axis=1 ) ) ) ) - 1,
                              int( max( np.max( np.mean( old_robosub_iteration_count, axis=1 ) ), np.max( np.mean( new_robosub_iteration_count, axis=1 ) ) ) ) + 2 ) ] )
    plt.plot( x, x, color="black", linestyle="dashed" )
    plt.gca().set_aspect( 'equal', adjustable='box' )
    plt.xlabel( "Mean Old Iteration Count" )
    plt.ylabel( "Mean New Iteration Count" )
    plt.title( "Robosub Domain")
    plt.show()




"""
Author(s): Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
Organization: University of Maryland at College Park
"""