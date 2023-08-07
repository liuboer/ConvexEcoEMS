"""@file plotRoutingResults.py
@brief The file contains the function to plot the routing results.

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
import matplotlib.pyplot as plt
import csv
from math import *
import numpy as np

## @fn plotRoutingResults
#
# formulates and solves routing problem
# @param file_nodes csv file containing the node information
# @param file_edges csv file containing the edge information
# @param first_node string, name of the first node
# @param last_node string, name of the end node
# @param fileName_result string, name of the results file
# @param dirNameFigs string, name of the directory to store the figures
def plotRoutingResults(file_nodes,file_edges, file_start_end,fileName_result,dirNameFigs):



    file_start_end = file_start_end
    filename= fileName_result
    file_nodes = file_nodes
    file_edges = file_edges

    plt.rcParams.update({'font.size':28,'lines.linewidth':4})
    plt.rcParams['figure.figsize'] = 8,6

    ## @fn resistance_force
    #   Calculates the resistance force pushing backwards against the car
    ## @var l
    #   length of respective edge, horizontal projection (open street map measures distances as horizontal projection)
    ## @var v
    #   float velocity of the car on respective edge
    ## @var h
    #   elevation difference of end node minus starting node
    def resistance_force(l, v, h):
        v = v / 3.6  # [m/s]
        rho = 1.2  # [kg/m^3]
        c_d = 0.335  # [-]
        A = 2.0  # [m^2]
        M = 1380  # [kg]
        g = 9.81  # [m/s^2]
        f_res = 0.009  # [-]
        ## @var alpha
        # slope angle of the edge
        alpha = atan(h / l)
        ## @var F_air
        # air resistance, >0, [N]
        F_air = 0.5 * c_d * A * rho * v * v
        ## @var F_slope
        # gravity, >0, =0 or <0 [N]
        F_slope = g * M * sin(alpha)
        ## @var F_roll
        # rolling resistance, >0 and independent of velocity [N]
        F_roll = f_res * g * M * cos(alpha)
        ## @var F_res
        # overall resistance force, >0 in negative driving direction [N]
        F_res = F_air + F_slope + F_roll
        return F_res

    def extract_variables(file):
        with open(file, "r") as f:
            text = f.read().split("\n  Objectives:")[0].split("Variables:\n")[1]
            blocks = text.split(" : Size=")
            tables = [block.split("\n")[2:-1] for block in blocks][1:]
            names = [block.split("\n")[-1].split()[0] for block in blocks[0:-1]]
            lists = [[float("".join(i.split()).split(":")[2]) for i in block] for block in tables]
            d = dict()
            for n in range(0, len(names)):
                d[names[n]] = lists[n]
        return d

    def get_OBJ(file):
        with open(file, "r") as f:
            obj = float(f.read().split("\n\n  Constraints:")[0].split("Objectives:\n")[1].split()[-1])
        return obj
    ###########################################################################################


    G = nx.Graph()




    z=get_OBJ(filename)



    states=extract_variables(filename)

    #retrieve start and end node
    with open(file_start_end) as f:
        f = f.read()
        first_node = f.split()[0]
        last_node = f.split()[1]

    # retrieve node position and elevation
    with open(file_nodes) as f:
        rows = csv.reader(f, delimiter=",")
        rows = [row for row in rows][1:]
        elevations = dict()
        i = 0
        for row in rows:
            G.add_node(row[0], pos=(float(row[2]), float(row[3])), elevation=float(row[1]))
            elevations[row[0]] = float(row[1])
            if row[0] == last_node:
                last_node_index = i
            i +=1
        pos = nx.get_node_attributes(G, "pos")

    SOC_f = states["socNodes"][last_node_index]

    with open(file_edges) as f:
        rows = csv.reader(f, delimiter=",")
        rows = [row for row in rows][1:]
        ## @var edgelist:
        # list of all edge ID's
        edge_list = []
        for row in rows:
            edge_list.append(row[0])

    """@var selected_states:
    list of the names of all variables from the optimization problem 
    to be retrieved from the results file"""
    selected_states=["socEdgeStart","TEdgesM","omegaEdgesM","pEdgesM", "pEdgesIce"]
    """@var edge_dict:
       keys: edge ID's 
       values: dictionaries containing edge information (by variable name as key)"""
    edge_dict=dict()
    for edge in range(0,len(edge_list)):
        x=states["x"]
        if x[edge] == 1:
            edge_id = edge_list[edge]
            edge_dict[edge_id] = dict()
            edge_dict[edge_id]["P_ICE_net"] = states["pEdgesIce"][edge]
            edge_dict[edge_id]["P_M_net"] = states["omegaEdgesM"][edge] * states["TEdgesM"][edge]
            for state in selected_states:
                edge_dict[edge_id][state] = states[state][edge]

    #add edge information from problem formulation
    with open(file_edges) as f:
        rows = csv.reader(f, delimiter=",")
        rows = [row for row in rows][1:]
        for row in rows:
            G.add_edge(row[3], row[4], name = row[0])
            if row[0] in edge_dict:
                edge_dict[row[0]]["start"] = row[3]
                edge_dict[row[0]]["end"] = row[4]
                edge_dict[row[0]]["length"] = float(row[1])
                edge_dict[row[0]]["velocity"] = float(row[2])
                force = round(resistance_force(l=float(row[1]), v=float(row[2]), h=elevations[row[4]] - elevations[row[3]]), 1)
                edge_dict[row[0]]["force"] = force
                edge_dict[row[0]]["P_total"] = force * float(row[2]) /3.6
        names= nx.get_edge_attributes(G, "name")
    selected_states.extend(["P_M_net", "P_ICE_net", "P_total"])

    ## @var path_nodes
    # list of node ID's along the path, in the order they are encountered by the verhicle
    path_nodes = [first_node]
    ## @var path_edges
    # list of edge ID's along the path, in the order they are encountered by the verhicle
    path_edges = []
    ## @var path
    # list of node-tuples representing path to be drawn in map (visualization)
    path=[]

    print("Begin finding connected path")
    while not (path_nodes[-1] == last_node):
        found = False
        for edge in edge_dict:
            if edge_dict[edge]["start"] == path_nodes[-1]: #next edge found
                path.append((path_nodes[-1], edge_dict[edge]["end"]))
                path_nodes.append(edge_dict[edge]["end"])
                path_edges.append(edge)
                path.append((path_nodes[-1], edge_dict[edge]["end"]))
                found = True
        if found==False:
            print("Failed finding connected path. Check start and end nodes.")
            exit()

    print("Done finding connected path")



    #plot map
    plt.figure()
    nx.draw(G, pos=pos, with_labels=False, node_size=10,node_color='black')
    nx.draw_networkx_edges(G, pos=pos, edge_color = "g", edgelist=path,width=5.0)
    nx.draw_networkx(G.subgraph(first_node), pos=pos, font_size=14, node_color='black',font_color='white',node_size=400,with_labels=True,labels={first_node:'S'},node_shape='h')
    nx.draw_networkx(G.subgraph(last_node), pos=pos, font_size=14, node_color='black',font_color='white',node_size=400,with_labels=True,labels={last_node:'E'},node_shape='h')
    plt.draw()
    plt.savefig(dirNameFigs + '/fig_map.png', format='png', bbox_inches='tight',transparent=True, dpi=1000)
    plt.savefig(dirNameFigs + '/fig_map.eps', format='eps', bbox_inches='tight',transparent=True)
    plt.show(block=False)

    """@ var series
    dictionary of edge states
    key: variable name
    value: list of state values for edges in the order they are encountered by the vehicle
    """
    series = dict()
    for state in selected_states:
        series[state] = [float(edge_dict[path_edges[0]][state])]

    ## var elevation
    # list of node elevations in the order they are encountered by the vehicle
    elevation= [elevations[edge_dict[path_edges[0]]["start"]]]
    ## var SOC
    # list of SOC at nodes in the order they are encountered by the vehicle
    SOC=[]
    ## var x_values
    # list of x values for the diagrams, representing distance driven
    x_values = [0]
    sum = 0
    for edge in path_edges:
        for state in selected_states:
            series[state].append(float(edge_dict[edge][state]))
        sum+=edge_dict[edge]["length"]
        x_values.append(sum)
        elevation.append(elevations[edge_dict[edge]["end"]])
        SOC.append(float(edge_dict[edge]["socEdgeStart"]))
    ## var min elevation
    # reference point for elevation in figures is set to minimum elevation
    min_elevation = min(elevation)
    SOC.append(SOC_f)

    #create first plot
    plt.figure()
    fig, ax1 = plt.subplots()
    plt.fill([0] + (np.array(x_values)/1000).tolist() + [x_values[-1]/1000], [0] + [i-min_elevation for i in elevation] + [0], color='lightsteelblue')
    ax1.set_ylim([np.array(elevation).min()-np.array(elevation).min(),np.array(elevation).max()-np.array(elevation).min()])
    plt.xlabel("distance / km")
    ax1.set_ylabel("height / m", color='lightsteelblue')
    ax1.tick_params(axis='y', labelcolor='lightsteelblue')

    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

    ax2.plot(np.array(x_values)/1000, SOC,color='red', linewidth = 3, label = 'routing')
    ax2.set_ylabel("SOC")
    plt.xlim([x_values[0],x_values[-1]/1000])

    plt.savefig(dirNameFigs+'/fig_SOC.png', format='png', bbox_inches='tight',transparent=True, dpi=1000)
    plt.savefig(dirNameFigs+'/fig_SOC.eps', format='eps', bbox_inches='tight',transparent=True)
    plt.show(block=False)

    #create second plot
    plt.figure()
    fig, ax1 = plt.subplots()
    plt.fill([0] + (np.array(x_values)/1000).tolist() + [x_values[-1]/1000], [0] + [i-min_elevation for i in elevation] + [0], color='lightsteelblue')
    plt.xlabel("distance / km")
    ax1.set_ylabel("height / m", color='lightsteelblue')
    ax1.tick_params(axis='y', labelcolor='lightsteelblue')
    ax1.set_ylim([np.array(elevation).min()-np.array(elevation).min(),np.array(elevation).max()-np.array(elevation).min()])

    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

    ax2.step(np.array(x_values)/1000, [i/1000 for i in series["P_ICE_net"]],color='black', linewidth = 3)
    ax2.step(np.array(x_values)/1000, [i/1000 for i in series["P_M_net"]],color='red',linestyle='--', linewidth = 3)
    ax2.plot([0, x_values[-1]/1000], [0, 0], linewidth=0.5, color="k")
    ax2.legend([r"$P_\mathrm{engine}$", r"$P_\mathrm{motor}$"])
    ax2.set_ylabel("power / kW")
    plt.xlim([x_values[0],x_values[-1]/1000])

    plt.savefig(dirNameFigs+'/fig_schedule.png', format='png', bbox_inches='tight',transparent=True, dpi=1000)
    plt.savefig(dirNameFigs+'/fig_schedule.eps', format='eps', bbox_inches='tight',transparent=True)
    plt.show(block=False)

    print('Done plot routing results')