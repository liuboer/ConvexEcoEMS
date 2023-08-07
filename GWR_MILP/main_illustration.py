"""@file main_illustration.py
@brief The file contains main to solve the routing optimization problem.

@author: Adrian Caspari, Steffen Fahr
@copyright: Copyright 2021
@version: 1.0
@data: February 24, 2021
@maintainer: Adrian Caspari, Steffen Fahr
@email: adrian.caspari@rwth-aachen.de
@license: MIT-license
          Copyright 2021 Adrian Caspari, Steffen Fahr, Alexander Mitsos (AVT Process Systems Engineering, RWTH Aachen University, 52074 Aachen, Germany, amitsos@alum.mit.edu).
          Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
          The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
          THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""
import src.solveRoutingOptProb as routing
import src.plotRoutingResults as plot
import time 
import scipy.io

## @param file_nodes
#  cvs file containing the nodes for the routing problem
file_nodes = 'case_illustration/nodes.csv'
## @param file_edges
#  cvs file containing the edges for the routing problem
file_edges = 'case_illustration/edges.csv'

## @param fileName_file_start_end
#  string, name of file to ID of start and end node
file_start_end = 'case_illustration/start_and_destination.txt'

## @param fileName_result
#  string, name of file to store the results
fileName_result = 'case_illustration/result.txt'
## @param fileName_log
#  string, name of file to store the log
fileName_log = 'case_illustration/log.txt'

## @param constrainPathTime
#  bool, True if path time is constrained in routing problem
constrainPathTime = False
pathTime_fastestPath = 841.873  # s
## @param pathTimeUpperBound
#  real, upper bound for path time
pathTimeUpperBound = pathTime_fastestPath * 1.2

## @param flagPath
#  string, path to solve
#  'shortestPath' routing problem uses the path length as objective function
#  'ecoPath' routing problem uses the path fuel demand as objective function
#  'fastestPath' routing problem uses the path time as objective function
flagPath = 'ecoPath'

## @param dirNameFigs
#  string, name of the directory to store the figures
dirNameFigs = 'case_illustration'

tc_start_1 = time.time()
tc_start_2 = time.clock()
tc = {}

## solve routing problem
routing.solveRoutingOptProb(file_nodes,file_edges,file_start_end,fileName_result,fileName_log,constrainPathTime,pathTimeUpperBound,flagPath)

tc_end_1 = time.time()
tc_end_2 = time.clock()

tc['tc'] = tc_end_1 - tc_start_1
tc['tc_CPU'] = tc_end_2 - tc_start_2

scipy.io.savemat('tc.mat',mdict={'tc':tc,})

## plot routing results
plot.plotRoutingResults(file_nodes,file_edges,file_start_end,fileName_result,dirNameFigs)