#!/usr/bin/env python
"""
File Description: data analysis and plotting
"""

import numpy as np
import os
import re
import matplotlib.pyplot as plt
import itertools

if __name__ == '__main__':
    # error rates tested
    error_rates = [ *range( 0, 110, 10 ) ]
    E = len( error_rates )
    # planner types used
    planner_types = [ "old", "new" ]
    P = len( planner_types )
    # metrics recorded
    metrics = [ "action_count", "cpu_time", "iteration_count" ]
    M = len( metrics )
    # number of tests per setting
    N = 1000
    # number of distinct problems
    Q = 20
    Qs = [ i + 1 for i in range( Q + 1 )]

    # data tensor
    results = -1 * np.ones( ( E, P, M, N, Q ) )

    # read in data
    data_dir = "satellite/data"

    coord_re = re.compile( "([a-z]+)_([a-z]+)_([\w]+)\.csv" )


    for error_dir in os.listdir( data_dir ):
        for file_name in os.listdir( data_dir + "/" + error_dir ):
            if ".csv" in file_name:
                file_path = data_dir + "/" + error_dir + "/" + file_name
                planner_type, domain, metric = coord_re.search( file_name ).group( 1, 2, 3 )
                coord = (
                    error_rates.index( int( error_dir ) ),
                    planner_types.index( planner_type ),
                    metrics.index( metric )
                )
                results[ coord ] = np.genfromtxt( file_path, delimiter="," )

    plt.figure( 0 )
    plt.title( "Old Replanning CPU Time vs Action Count" )
    for q in range( Q ):
        plt.scatter( np.mean( results[ :, 0, 0, :, q ], axis=(1) ) , np.mean( results[ :, 0, 1, :, q ], axis=(1) ) )
    plt.legend( Qs )
    plt.xlabel("Action Count")
    plt.ylabel("CPU Time (s)")
    # plt.savefig( "old_cpu_action.png" )
    plt.show()
    plt.figure( 1 )
    plt.title( "New Replanning CPU Time vs Action Count" )
    for q in range( Q ):
        plt.scatter( np.mean( results[ :, 1, 0, :, q ], axis=(1) ), np.mean( results[ :, 1, 1, :, q ], axis=(1) ) )
    plt.legend( Qs )
    plt.xlabel( "Action Count" )
    plt.ylabel( "CPU Time (s)" )
    # plt.savefig( "new_cpu_action.png" )
    plt.show()

    plt.figure( 2 )
    plt.title( "Old Replanning CPU Time vs Iteration Count" )
    for q in range( Q ):
        plt.scatter( np.mean( results[ :, 0, 2, :, q ], axis=(1) ), np.mean( results[ :, 0, 1, :, q ], axis=(1) ) )
    plt.legend( Qs )
    plt.xlabel( "Iteration Count" )
    plt.ylabel( "CPU Time (s)" )
    # plt.savefig( "old_cpu_action.png" )
    plt.show()
    plt.figure( 3 )
    plt.title( "New Replanning CPU Time vs Iteration Count" )
    for q in range( Q ):
        plt.scatter( np.mean( results[ :, 1, 2, :, q ], axis=(1) ), np.mean( results[ :, 1, 1, :, q ], axis=(1) ) )
    plt.legend( Qs )
    plt.xlabel( "Iteration Count" )
    plt.ylabel( "CPU Time (s)" )
    # plt.savefig( "new_cpu_action.png" )
    plt.show()

    plt.figure( 4 )
    plt.title( "Old Replanning Iteration Count vs Action Count" )
    for q in range( Q ):
        plt.scatter( np.mean( results[ :, 0, 0, :, q ], axis=(1) ), np.mean( results[ :, 0, 2, :, q ], axis=(1) ) )
    plt.legend( Qs )
    plt.xlabel( "Action Count" )
    plt.ylabel( "Iteration Count" )
    # plt.savefig( "old_cpu_action.png" )
    plt.show()
    plt.figure( 5 )
    plt.title( "New Replanning Iteration Count vs Action Count" )
    for q in range( Q ):
        plt.scatter( np.mean( results[ :, 1, 0, :, q ], axis=(1) ), np.mean( results[ :, 1, 2, :, q ], axis=(1) ) )
    plt.legend( Qs )
    plt.xlabel( "Action Count" )
    plt.ylabel( "Iteration Count" )
    # plt.savefig( "new_cpu_action.png" )
    plt.show()


"""
Author(s): Paul Zaidins
Repository: https://github.com/pzaidins2/IPyHOP
Organization: University of Maryland at College Park
"""