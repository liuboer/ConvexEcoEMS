"""@file solveRoutingOptProb.py
@brief The file contains the routing optimization problem function.

Procedure:
The data for the graph is loaded.
The graph is generated.
The routing optimization problem is formulated in pyomo.
Gurobi is called to solve the optimization problem.
The routing optimization results and log is stored as .txt file.

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
import networkx as nx
from pyomo.environ import *
import csv
import pyomo.environ as pyo
import numpy as np


## @fn solveRoutingOptProb
#
# formulates and solves routing problem
# @param file_nodes csv file containing the node information
# @param file_edges csv file containing the edge information
# @param first_node string, name of the first node
# @param last_node string, name of the end node
# @param fileName_result string, name of the results file
# @param fileName_log string, name of the log file
# @param constrainPathTime bool, True if path time to be constrained
# @param pathTimeUpperBound Real, upper bound for path time
# @param dirNameFigs
#        string, name of the directory to store the figures
def solveRoutingOptProb(file_nodes,file_edges,file_start_end ,fileName_result,fileName_log,constrainPathTime,pathTimeUpperBound,flagPath):



    file_nodes = file_nodes
    file_edges = file_edges
    file_start_end = file_start_end
    fileName_result = fileName_result
    fileName_log = fileName_log


    #######################################################################################################
    ##

    ## @var socStartingNode
    #  state of charge at starting node
    socStartingNode = 0.6
    # @var: state of charge at end node
    socEndNode = socStartingNode
    HHV = 120e6  # in J/kg
    etaM = 0.9 #Guzella 2013, p. 91
    P_0m = 0 #Guzella 2013, p. 91
    TMaxM = 200  # , in Nm DUMMY compare Problem 4.38 page 160
    TMinM = -200  # , in Nm  DUMMY
    omegaMaxM = 10000/(30/np.pi)  # , in rad/s DUMMY
    gamma = 6.67 #previously 3.5 # DUMMY
    r_w = 0.282  # in m   p. 160 problem 4.38
    # eta_coulomb = 0.95  # from page 275, Guzzella
    pIce_max = 50e3 # W
    pM_min = -75e3 # W
    pM_max = 75e3 # W
    pB_min = -85e3 # W
    pB_max = 85e3 # W
    mFuel_max = 40000 # 1e-3 kg
    TMaxBrake = 1000 # in Nm


    fitParamsIce = [-0.000251074740566653, 0.000241896048457684]
    parametersBattery = [-0.000251074740566653, 0.000241896048457684, -2.31994904002174e-08]

    ## @fn resistance_force
    #   Calculates the resistance force pushing backwards against the car
    # @param l
    #   length of respective edge, horizontal projection (open street map measures distances as horizontal projection)
    # @param v
    #   float velocity of the car on respective edge
    # @param h
    #   elevation difference of end node minus starting node
    def resistance_force(l, v, h):
        v = v / 3.6  # [m/s]
        rho = 1.2  # [kg/m^3]
        c_d = 0.335  # [-]
        A = 2.0  # [m^2]
        M = 1380  # [kg]
        g = 9.81  # [m/s^2]
        f_res = 0.009  # [-]
        alpha = atan(h / l) #slope angle
        #@var F_air
        # air resistance, >0, [N]
        F_air = 0.5 * c_d * A * rho * v * v
        #@var F_slope
        # gravity, >0, =0 or <0 [N]
        F_slope = g * M * sin(alpha)
        #@var F_roll
        # rolling resistance, >0 and independent of velocity [N]
        F_roll = f_res * g * M * cos(alpha)
        #@var F_res
        # overall resistance force, >0 in negative driving direction [N]
        F_res = F_air + F_slope + F_roll
        return F_res

    ## @fn calc_time
    #   calculate time spent on edge [s]
    def calc_time(l, v, h):
        v = v / 3.6  # [m/s]
        t = sqrt(h ** 2 + l ** 2) / v  # in s
        return t

    ## @fn arriving_edges
    #   create table of edges e (rows) and nodes n (columns); Entry for e and n is 1 if e ends at n, else entry is 0.
    def arriving_edges(edges, nodes):
        rows = []
        for row in edges:
            new_row = []
            new_row.extend([1 if (node[0] == row[-1]) else 0 for node in nodes])
            rows.append(new_row)
        return rows

    ## @fn leaving_edges
    #   create table of edges e (rows) and nodes n (columns); Entry for e and n is 1 if e starts at n, else entry is 0.
    def leaving_edges(edges, nodes):
        rows = []
        for row in edges:
            new_row = []
            new_row.extend([1 if (node[0] == row[-2]) else 0 for node in nodes])
            rows.append(new_row)
        return rows


    #############################################################################################################

    ## @var G
    #   networkx graph of edges and nodes (for visualization purposes)
    G = nx.Graph()
    ## @var edge_list
    #   list of strings: all edge names
    edge_list = []
    ## @var node_list
    #   list of strings: all node names
    node_list = []

    with open(file_start_end) as f:
        f = f.read()
        first_node = f.split()[0]
        last_node = f.split()[1]

    with open(file_nodes) as f:
        rows = csv.reader(f, delimiter=",")
        rows = [row for row in rows][1:]
        for row in rows:
            G.add_node(row[0], pos=(float(row[2]), float(row[3])),
                       elevation=float(row[1]))  # pos is important for map display
            node_list.append([row[0]])

    ## @var elevations
    #   dictionary of node elevations [m]
    elevations = nx.get_node_attributes(G, "elevation")
    ## @var pos
    #   dictionary of node positions (for visualization purposes)
    pos = nx.get_node_attributes(G, "pos")  # for map display

    with open(file_edges) as f:
        rows = csv.reader(f, delimiter=",")
        rows = [row for row in rows][1:]
        ## @var matrixForce
        # list of doubles: resistance force in N for each edge
        matrixForce = []
        ## @var matrixLength
        # list of doubles: edge length in m for each edge
        matrixLength = []
        ## @var matrixVelocity
        # list of doubles: velocity in m/s for each edge
        matrixVelocity = []
        ## @var matrixOmegaWheel
        # list of doubles: rotational frequency of wheel in rad/s for each edge
        matrixOmegaWheel = []
        ## @var matrixTime
        # list of doubles: time in s for each edge
        matrixTime = []  # t in s
        for row in rows:
            force = round(resistance_force(l=float(row[1]), v=float(row[2]), h=elevations[row[4]] - elevations[row[3]]), 1)
            G.add_edge(row[3], row[4], force=force * float(row[1]), weight=force * float(row[1]), name=row[0],
                       velocity=row[2])
            matrixForce.append(force)
            matrixLength.append(float(row[1]))
            matrixVelocity.append(float(row[2]) / 3.6)
            matrixOmegaWheel.append(float(row[2]) / 3.6 * gamma/r_w)
            matrixTime.append(
                round(calc_time(l=float(row[1]), v=float(row[2]), h=elevations[row[4]] - elevations[row[3]]), 3))
            edge_list.append([row[0], row[3], row[4]])
    ## @var matrixA
#   list of lists / matrix: a(i,j)= 1 if edge i ends at node j, 0 otherwise
    matrixA = arriving_edges(edge_list, node_list)
    ## @var matrixL
#   list of lists / matrix: l(i,j)= 1 if edge i starts at node j, 0 otherwise
    matrixL = leaving_edges(edge_list, node_list)

    timeEdges = np.array(matrixTime)



    # # Display graph
    # weights = nx.get_edge_attributes(G, "weight")  # for map image
    # names = nx.get_edge_attributes(G, "name")  # for map image
    # plt.cla()
    # plt.close()
    # nx.draw(G, pos=pos, edge_labels=weights, with_labels=True)
    # nx.draw_networkx_edge_labels(G, pos, edge_labels=names)
    # plt.draw()
    # plt.show()

    ######################################################################################################################


    model = ConcreteModel()

    model.numEdges = len(matrixA)
    model.numNodes = len(matrixA[0])
    print("edges: " + str(model.numEdges))
    print("nodes: " + str(model.numNodes))

    ## @var matrixS
    #list of integers, for each edge: s(n) n is start node: -1, n is end node: 1, else: 0
    matrixS = [0] * model.numNodes
    index_StartNode =0
    index_EndNode = 0
    for i in range(0,model.numNodes):
        if node_list[i][0]==first_node:
            index_StartNode = i
            matrixS[i] = -1 # start node
        if node_list[i][0]==last_node:
            matrixS[i] = 1 # end node
            index_EndNode = i



    # sets
    model.Edges = RangeSet(0, model.numEdges - 1)
    model.Nodes = RangeSet(0, model.numNodes - 1)


    ## variables
    #@var x
    #binary: x(e) = 1 if edge is is used, else 0
    model.x = Var(model.Edges, domain=Binary)
    ##@var Beta
    #binary: Beta(e) = 1 if electric motor drives power train, Beta(e) = 0 if electric motor recuperates
    model.Beta = Var(model.Edges, domain=Binary)
    ##@var mFuel
    #real: amount of fuel burnt on edge e [1e-3 kg]
    model.mFuel = Var(model.Edges, domain=NonNegativeReals,bounds=(0,mFuel_max))
    ##@var z_mFuel_x
    #real: =mFuel*x; mFuel if edge is used, else 0 [kg]
    model.z_mFuel_x = Var(model.Edges, domain=Reals)  # fuel on edge in kg
    ##@var z_TEdgesW_x
    #real: TEdgesW if edge is used, else 0 [Nm]
    model.z_TEdgesW_x = Var(model.Edges, domain=Reals)
    ##@var socNodes
    #real: state of charge (SOC) at node in [0, 1]
    model.socNodes = Var(model.Nodes, domain=NonNegativeReals, bounds=(0, 1))
    ##@var socEdgeStart
    #real: SOC at start node of edge in [0, 1]
    model.socEdgeStart = Var(model.Edges, domain=NonNegativeReals, bounds=(0, 1))
    ##@var socEdgeEnd
    #real: SOC at end node of edge in [0, 1]
    model.socEdgeEnd = Var(model.Edges, domain=NonNegativeReals, bounds=(0, 1))
    ##@var z_socEdgeEnd_x
    #real: socEdgeEnd*x [0, 1]
    model.z_socEdgeEnd_x = Var(model.Edges, domain=Reals)
    ##@var z_beta_pEdgesM
    #real: =beta*pEdgesM; pM (electric power) if pM is positive, else 0
    model.z_beta_pEdgesM = Var(model.Edges, domain=Reals)
    ## @var TEdgesW
    # real: =Torque at Wheels [Nm]
    model.TEdgesW = Var(model.Edges, domain=Reals, bounds=(-TMaxM, TMaxM))
    ## @var TEdgesM
    # real: =Torque or electric motor [Nm]
    model.TEdgesM = Var(model.Edges, domain=Reals, bounds=(TMinM, TMaxM))
    ## @var TEdgesBrake
    # real: =Torque of brake is engaged [Nm]
    model.TEdgesBrake = Var(model.Edges, domain=NonNegativeReals, bounds=(0, TMaxBrake))
    ## @var omegaEdgesM
    # real: = rotational velocity of electric motor  [rad/s]
    model.omegaEdgesM = Var(model.Edges, domain=NonNegativeReals, bounds=(0,omegaMaxM)) #
    ## @var pEdgesM
    # real: = electric motor power or recuperated power  [kW]
    model.pEdgesM = Var(model.Edges, domain=Reals, bounds=(pM_min, pM_max))
    ## @var pEdgesIce
    # real: = combustion engine power in terms of fuel enthalpy flow [kW]
    model.pEdgesIce = Var(model.Edges, domain=NonNegativeReals, bounds=(0, pIce_max))
    ## @var pEdgesB
    # real: = battery power [kW]
    model.pEdgesB = Var(model.Edges, domain=Reals, bounds=(pB_min, pB_max))
    ## @var pathCost
    # real: = overall fuel consumption [kg]
    model.pathCost = Var(domain=NonNegativeReals,bounds=(0,100))
    ## @var pathTime
    # real: = overall time for trip [s]
    model.pathTime = Var(domain=NonNegativeReals,bounds=(0,10*3600))
    ## @var pathLength
    # real: = overall path length [m]
    model.pathLength = Var(domain=NonNegativeReals,bounds=(0,1000*1e3))



    # objective

    def obj(model):
        """@fn obj

        Defines objective function of routing problem.
        @param model the pyomo model
        @return objective function of routing
        """
        if flagPath == 'ecoPath':
            return pyo.summation(model.z_mFuel_x)
        if flagPath == 'shortestPath':
            return model.pathLength
        if flagPath == 'fastestPath':
            return model.pathTime

    def constraintPathCost(model): #continuous path

        return model.pathCost == pyo.summation(model.z_mFuel_x)/1e3

    def constraintPathTime(model): #continuous path

        return model.pathTime == sum(matrixTime[i] * model.x[i] for i in model.Edges)

    def constraintPathLength(model): #continuous path

        return model.pathLength == sum(matrixLength[i] * model.x[i] for i in model.Edges)

    model.OBJ = Objective(rule=obj, doc='objective function')

    # constraints

    def constraint_z_mFuel_x_1(model,e):
        bigM = mFuel_max*10
        return model.z_mFuel_x[e] <= bigM*model.x[e]

    def constraint_z_mFuel_x_2(model,e):
        bigM = mFuel_max*10
        return model.z_mFuel_x[e] >= -bigM*model.x[e] # big M

    def constraint_z_mFuel_x_3(model,e):
        bigM = mFuel_max*10
        return model.z_mFuel_x[e] <= model.mFuel[e] +bigM*(1-model.x[e]) # big M

    def constraint_z_mFuel_x_4(model,e):
        bigM = mFuel_max*10
        return model.z_mFuel_x[e] >= model.mFuel[e] - bigM*(1-model.x[e])


    def constraint_z_socEdgeEnd_x_1(model,e):
        bigM_z_socEdgeEnd_x = 10
        return model.z_socEdgeEnd_x[e] <= bigM_z_socEdgeEnd_x*model.x[e]

    def constraint_z_socEdgeEnd_x_2(model,e):
        bigM_z_socEdgeEnd_x = 10
        return model.z_socEdgeEnd_x[e] >= -bigM_z_socEdgeEnd_x*model.x[e] # big M

    def constraint_z_socEdgeEnd_x_3(model,e):
        bigM_z_socEdgeEnd_x = 10
        return model.z_socEdgeEnd_x[e] <= model.socEdgeEnd[e] +bigM_z_socEdgeEnd_x*(1-model.x[e])# big M

    def constraint_z_socEdgeEnd_x_4(model,e):
        bigM_z_socEdgeEnd_x = 10
        return model.z_socEdgeEnd_x[e] >= model.socEdgeEnd[e] - bigM_z_socEdgeEnd_x*(1-model.x[e])


    def constraint_z_TEdgesW_x_1(model,e):
        bigM_z_TEdgesW_x = (TMaxM)*10
        return model.z_TEdgesW_x[e] <= bigM_z_TEdgesW_x*model.x[e]

    def constraint_z_TEdgesW_x_2(model,e):
        bigM_z_TEdgesW_x = (TMaxM)*10
        return model.z_TEdgesW_x[e] >= -bigM_z_TEdgesW_x*model.x[e]

    def constraint_z_TEdgesW_x_3(model,e):
        bigM_z_TEdgesW_x = (TMaxM)*10
        return model.z_TEdgesW_x[e] <= model.TEdgesW[e] +bigM_z_TEdgesW_x*(1-model.x[e])

    def constraint_z_TEdgesW_x_4(model,e):
        bigM_z_TEdgesW_x = (TMaxM)*10
        return model.z_TEdgesW_x[e] >= model.TEdgesW[e] - bigM_z_TEdgesW_x*(1-model.x[e])


    def constraintContinuity(model, n): #continuous path
        ## @fn constraintContinuity
        # Make sure the vehicle drives a continuous path (cf. standard shortest path problem)
        return matrixS[n]== sum(model.x[i]*(matrixA[i][n] - matrixL[i][n]) for i in model.Edges)

    def constraintCycle(model, n): #Cannot drive cycles
        ## @fn constraintCycle
        # Prohibit driving in loops. Each node may only have 1 predecessor (see SOC calculations)
        return 2 >= sum((matrixA[i][n] + matrixL[i][n]) * model.x[i] for i in model.Edges)

    def constraintEnergyBalance1(model, n): #continuity of SOC from one path to next
            return model.socNodes[n] == sum(matrixA[i][n] * model.z_socEdgeEnd_x[i] for i in model.Edges)

    def constraintEnergyBalance2(model, e): #continuity of SOC from one path to next

        return model.socEdgeStart[e] == sum(matrixL[e][i] * model.socNodes[i] for i in model.Nodes)

    def constraintForceBalance(model, e): #Torque at wheel vs resistance force in Nm
        return model.TEdgesW[e] * gamma == matrixForce[e] * r_w + model.TEdgesBrake[e]

    def constraintTorqueCouple(model, e): # 3-way torque couple between axle, motor and engine in Nm
        return model.z_TEdgesW_x[e] == model.TEdgesM[e]

    def constraintPowerM(model, e): # calculate motor power including losses in W
        return model.TEdgesM[e] * matrixOmegaWheel[e] + P_0m == model.z_beta_pEdgesM[e]  * etaM + model.pEdgesM[e] / etaM  - model.z_beta_pEdgesM[e] /etaM

    def constraintBeta1(model, e): # If motor is driving car, beta must be 1
        return model.pEdgesM[e]-model.z_beta_pEdgesM[e]  <= 0

    def constraintBeta2(model, e): # If motor is recuperating, beta must be 0
        return -model.z_beta_pEdgesM[e]  <= 0

    def constraint_z_beta_pEdgesM_1(model,e):
        bigM_z_beta_pEdgesM = pM_max*10
        return model.z_beta_pEdgesM[e] <= bigM_z_beta_pEdgesM*model.Beta[e]

    def constraint_z_beta_pEdgesM_2(model,e):
        bigM_z_beta_pEdgesM = -pM_min*10
        return model.z_beta_pEdgesM[e] >= -bigM_z_beta_pEdgesM*model.Beta[e]

    def constraint_z_beta_pEdgesM_3(model,e):
        bigM_z_beta_pEdgesM = pM_max*10
        return model.z_beta_pEdgesM[e] <= model.pEdgesM[e] +bigM_z_beta_pEdgesM*(1-model.Beta[e])

    def constraint_z_beta_pEdgesM_4(model,e):
        bigM_z_beta_pEdgesM = -pM_min*10
        return model.z_beta_pEdgesM[e] >= model.pEdgesM[e] - bigM_z_beta_pEdgesM*(1-model.Beta[e])

    def constraintRPMCoupleM(model, e): # couple rotational velocities axle to motor in rad/s
        return matrixOmegaWheel[e] == model.omegaEdgesM[e]

    def constraintMFuel(model, e): #calculate amount of fuel as fake integral of engine enthalpy flow.
        return model.mFuel[e] * HHV/1e3 == matrixTime[e] * (fitParamsIce[0] + fitParamsIce[1] * model.pEdgesIce[e])  # deal with integral

    def constraintIcePowerMap(model, e): #calculate amount of fuel as fake integral of engine enthalpy flow.
        return model.pEdgesIce[e] == model.pEdgesM[e] - model.pEdgesB[e]

    def constraintBatteryIntegral(model, e): # calculate charge change as fake integral. Euler forward integral
        return model.socEdgeEnd[e] == -parametersBattery[0]/parametersBattery[1] - parametersBattery[2]/parametersBattery[1] * model.pEdgesB[e] + (model.socEdgeStart[e] + parametersBattery[0]/parametersBattery[1] + parametersBattery[2]/parametersBattery[1] * model.pEdgesB[e])*np.exp(timeEdges[e]*parametersBattery[1])



    model.constraintPathCost = pyo.Constraint(rule=constraintPathCost,   doc='constraintPathCost')
    model.constraintPathTime = pyo.Constraint(rule=constraintPathTime,   doc='constraintPathTime')
    model.constraintPathLength = pyo.Constraint(rule=constraintPathLength,   doc='constraintPathLength')
    model.constraintBalance = pyo.Constraint(model.Nodes, rule=constraintContinuity)
    model.constraintCycle = pyo.Constraint(model.Nodes, rule=constraintCycle)
    model.constraintEnergyBalance0 = pyo.Constraint(expr=model.socNodes[index_StartNode]==socStartingNode)
    model.constraintEnergyBalance1 = pyo.Constraint(model.Nodes-pyo.Set(initialize=[index_StartNode]), rule=constraintEnergyBalance1,   doc='energy balance constraints for every node')
    model.constraint_z_socEdgeEnd_x_1 = pyo.Constraint(model.Edges, rule=constraint_z_socEdgeEnd_x_1)
    model.constraint_z_socEdgeEnd_x_2 = pyo.Constraint(model.Edges, rule=constraint_z_socEdgeEnd_x_2)
    model.constraint_z_socEdgeEnd_x_3 = pyo.Constraint(model.Edges, rule=constraint_z_socEdgeEnd_x_3)
    model.constraint_z_socEdgeEnd_x_4 = pyo.Constraint(model.Edges, rule=constraint_z_socEdgeEnd_x_4)
    model.constraint_z_mFuel_x_1 = pyo.Constraint(model.Edges, rule=constraint_z_mFuel_x_1)
    model.constraint_z_mFuel_x_2 = pyo.Constraint(model.Edges, rule=constraint_z_mFuel_x_2)
    model.constraint_z_mFuel_x_3 = pyo.Constraint(model.Edges, rule=constraint_z_mFuel_x_3)
    model.constraint_z_mFuel_x_4 = pyo.Constraint(model.Edges, rule=constraint_z_mFuel_x_4)
    model.constraint_z_TEdgesW_x_1 = pyo.Constraint(model.Edges, rule=constraint_z_TEdgesW_x_1)
    model.constraint_z_TEdgesW_x_2 = pyo.Constraint(model.Edges, rule=constraint_z_TEdgesW_x_2)
    model.constraint_z_TEdgesW_x_3 = pyo.Constraint(model.Edges, rule=constraint_z_TEdgesW_x_3)
    model.constraint_z_TEdgesW_x_4 = pyo.Constraint(model.Edges, rule=constraint_z_TEdgesW_x_4)
    model.constraint_z_beta_pEdgesM_1 = pyo.Constraint(model.Edges, rule=constraint_z_beta_pEdgesM_1)
    model.constraint_z_beta_pEdgesM_2 = pyo.Constraint(model.Edges, rule=constraint_z_beta_pEdgesM_2)
    model.constraint_z_beta_pEdgesM_3 = pyo.Constraint(model.Edges, rule=constraint_z_beta_pEdgesM_3)
    model.constraint_z_beta_pEdgesM_4 = pyo.Constraint(model.Edges, rule=constraint_z_beta_pEdgesM_4)
    model.constraintEnergyBalance2 = pyo.Constraint(model.Edges, rule=constraintEnergyBalance2)
    model.constraintForceBalance = pyo.Constraint(model.Edges, rule=constraintForceBalance)
    model.constraintTorqueCouple = pyo.Constraint(model.Edges, rule=constraintTorqueCouple)
    model.constraintRPMCoupleM = pyo.Constraint(model.Edges, rule=constraintRPMCoupleM)
    model.constraintEndnode = pyo.Constraint(expr=model.socNodes[index_EndNode]>=socEndNode)
    model.constraintPowerM = pyo.Constraint(model.Edges, rule=constraintPowerM)
    model.constraintBeta1 = pyo.Constraint(model.Edges, rule=constraintBeta1)
    model.constraintBeta2 = pyo.Constraint(model.Edges, rule=constraintBeta2)
    model.constraintMFuel = pyo.Constraint(model.Edges, rule=constraintMFuel)
    model.constraintIcePowerMap = pyo.Constraint(model.Edges, rule=constraintIcePowerMap)
    model.constraintBatteryIntegral = pyo.Constraint(model.Edges, rule=constraintBatteryIntegral)

    if constrainPathTime:
        model.constraintPathTimeUpperBound = pyo.Constraint(expr = model.pathTime<=pathTimeUpperBound)

    instance = model  # .create_instance()


    opt = pyo.SolverFactory('gurobi')
    # opt = pyo.SolverFactory('mosek')
    opt.set_options("NonConvex=0")  # set =2 necessary if problem is nonconvex
    opt.set_options( "MIPFocus=2")  # The MIPFocus parameter allows you to modify your high-level solution strategy, depending on your goals. By default, the Gurobi MIP solver strikes a balance between finding new feasible solutions and proving that the current solution is optimal. If you are more interested in finding feasible solutions quickly, you can select MIPFocus=1. If you believe the solver is having no trouble finding good quality solutions, and wish to focus more attention on proving optimality, select MIPFocus=2. If the best objective bound is moving very slowly (or not at all), you may want to try MIPFocus=3 to focus on the bound.
    opt.set_options("TIME_LIMIT=10800")
    opt.set_options("Cuts=0")
    results = opt.solve(model, tee=True,  logfile=fileName_log)


    instance.display()


    instance.display(fileName_result)
    results.write()
    print('results -> ' + fileName_result)
    print('solved routing otpimization problem')
