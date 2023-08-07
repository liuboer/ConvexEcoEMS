"""@file graph_compressor.py
@brief The file contains function definitions to compress graphs.

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
import osmnx as ox
import networkx as nx
import math
import urllib



def compress(graph, cluster_size = 12, cluster_by="edge length", keep_nodes=[]):

    edge_list = list(graph.edges.items())
    new_edge_list = [] #create list of all relevant edge attributes
    e = 1
    for edge in edge_list:
        start = int(edge[0][0])
        end = int(edge[0][1])
        edge_id = "e" + str(e)
        e += 1
        maxspeed = int(edge[1]["maxspeed"])
        name = edge[1]["name"]
        osmid = edge[1]["osmid"]
        length = int(edge[1]["length"])
        oneway = edge[1]["oneway"]
        length = length
        d = dict()
        d["start"] = start
        d["end"] = end
        d["edge_id"] = edge_id
        d["maxspeed"] = maxspeed
        d["name"] = name
        d["osmid"] = osmid
        d["length"] = length
        d["oneway"] = oneway
        new_edge_list.append(d)

    # node list (list of dictionaries)
    node_dict = []
    n = 1
    for node in graph.nodes():
        d=dict()
        d["x"] = graph.nodes[node]["x"]
        d["y"] = graph.nodes[node]["y"]
        d["osmid"] = graph.nodes[node]["osmid"]
        graph.nodes[node]["x"] = graph.nodes[node]["x"]
        graph.nodes[node]["y"] = graph.nodes[node]["y"]
        graph.nodes[node]["pos"] = graph.nodes[node]["pos"]
        graph.nodes[node]["osmid"] = graph.nodes[node]["osmid"]
        graph.nodes[node]["name"] = "n" + str(n)
        n += 1
        node_dict.append(d)

    ################################ cluster #########################################


    if not cluster_by in ["edge length", "euclidean distance", "dont"]:
        cluster_by = input("\"cluster by\"-parameter has invalid value. Please type \"edge length\" or \"euclidian distance\" or press enter to skip this step.")
    if cluster_by == "edge length":
        graph2 = cluster_by_edge_length(new_edge_list, graph, cluster_size, keep_nodes)
    elif cluster_by == "euclidean distance":
        graph2 = cluster_by_euclidean_distance(new_edge_list, node_dict, graph, cluster_size, keep_nodes)
    else:
        graph2 = cluster_by_euclidean_distance(new_edge_list, node_dict, graph, 0, keep_nodes)
        print("Clustering step skipped.")
    if not cluster_by == "dont":
        print("number of edges after clustering close nodes: " + str(len(list(graph2.edges.items()))))
        print("number of nodes after clustering close nodes: " + str(len(list(graph2.nodes()))))
    # fig, ax = ox.plot_graph(graph2)
    return(graph2)

def cluster_by_edge_length(reduced_edge_list, graph, cluster_size, keep_nodes=[]):
    # find accumulations  / clusters of nodes and create list of clusters and list of all nodes that are in clusters:
    graph2 = nx.MultiDiGraph()
    graph2.graph = graph.graph
    node_clusters=[]
    close_nodes=[]
    for edge in reduced_edge_list:
        if edge["length"] < cluster_size:
            new_cluster=[edge["start"], edge["end"]]
            cluster_is_new = True
            for cluster in node_clusters:
                if edge["start"] in cluster or edge["end"] in cluster:
                    cluster.extend(new_cluster)
                    cluster = list(set(cluster))
                    cluster_is_new = False
            if cluster_is_new:
                node_clusters.append(new_cluster)
            close_nodes.append(edge["start"])
            close_nodes.append(edge["end"])
    close_nodes=set(close_nodes)

    for node in keep_nodes:
        for cluster in node_clusters:
            if node in cluster:
                cluster = [node] + cluster

    # add edge information, leave out edges inside of clusters
    e = 1
    for edge in reduced_edge_list:
        if not int(edge["length"]) < cluster_size:
            if not int(edge["start"]) in close_nodes:
                start = int(edge["start"])
            else:
                for cluster in node_clusters:
                    if edge["start"] in cluster:
                        start = cluster[0]
            if not int(edge["end"]) in close_nodes:
                end = int(edge["end"])
            else:
                for cluster in node_clusters:
                    if edge["end"] in cluster:
                        end = cluster[0]

            edge_id = "e" + str(e)
            e += 1
            maxspeed = int(edge["maxspeed"])
            name = edge["name"]
            osmid = edge["osmid"]
            length = int(edge["length"])
            oneway = edge["oneway"]
            graph2.add_edge(start, end, osmid=osmid, edge_id=edge_id, length=length, maxspeed=maxspeed, oneway=oneway, name=name)

    isolates = nx.Graph()
    isolates.add_nodes_from(nx.isolates(graph2))
    graph2.remove_nodes_from(isolates)

    #add node information
    n = 1
    for node in graph2.nodes():
        graph2.nodes[node]["x"] = graph.nodes[node]["x"]
        graph2.nodes[node]["y"] = graph.nodes[node]["y"]
        graph2.nodes[node]["pos"] = graph.nodes[node]["pos"]   ###
        graph2.nodes[node]["osmid"] = graph.nodes[node]["osmid"]
        # graph2.nodes[node]["elevation"] = graph.nodes[node]["elevation"]
        graph2.nodes[node]["name"] = "n" + str(n)
        n += 1

    return graph2

def cluster_by_euclidean_distance(reduced_edge_list, reduced_node_dict, graph, cluster_size, keep_nodes=[]):
    graph2 = nx.MultiDiGraph()
    graph2.graph = graph.graph
    next_neighbor = dict()
    end_nodes = [i for i in reduced_node_dict]
    aggregated_nodes=[]
    for node in reduced_node_dict:
        next_neighbor[node["osmid"]] = node["osmid"]

    if cluster_size > 0:
        for Startnode in reduced_node_dict:
            if Startnode["osmid"] in aggregated_nodes:
                continue
            aggregated = []
            for Endnode in end_nodes:
                dist = euclidian_distance_1(Startnode, Endnode)
                if dist < cluster_size:
                    next_neighbor[Endnode["osmid"]] = next_neighbor[Startnode["osmid"]]
                    aggregated.append(Endnode)
            aggregated_nodes.extend(i["osmid"] for i in aggregated)
            for i in aggregated:
                end_nodes.remove(i)

        for node in keep_nodes:
            next_neighbor[node] = node

    # add edge information, leave out edges inside of clusters
    e = 1
    for edge in reduced_edge_list:
        if not next_neighbor[edge["start"]] == next_neighbor[edge["end"]]:
            start = next_neighbor[edge["start"]]
            end = next_neighbor[edge["end"]]
            edge_id = "e" + str(e)
            e += 1
            maxspeed = int(edge["maxspeed"])
            name = edge["name"]
            osmid = edge["osmid"]
            length = int(edge["length"])
            oneway = edge["oneway"]
            graph2.add_edge(start, end, osmid=osmid, edge_id=edge_id, length=length, maxspeed=maxspeed, oneway=oneway, name=name)

    isolates = nx.Graph()
    isolates.add_nodes_from(nx.isolates(graph2))
    graph2.remove_nodes_from(isolates)

    #add node information
    n = 1
    for node in graph2.nodes():
        graph2.nodes[node]["x"] = graph.nodes[node]["x"]
        graph2.nodes[node]["y"] = graph.nodes[node]["y"]
        graph2.nodes[node]["pos"] = graph.nodes[node]["pos"]   ###
        graph2.nodes[node]["osmid"] = graph.nodes[node]["osmid"]
        # graph2.nodes[node]["elevation"] = graph.nodes[node]["elevation"]
        graph2.nodes[node]["name"] = "n" + str(n)
        n += 1

    return(graph2)




def euclidian_distance_1(Startnode, Endnode):
    R = 6371e3  # metres
    delta_x = (Startnode["x"] - Endnode["x"]) / 180 * math.pi
    delta_y = (Startnode["y"] - Endnode["y"]) / 180 * math.pi
    a = math.sin(delta_x / 2) * math.sin(delta_x / 2) + math.cos(delta_x) * math.cos(delta_y) * math.sin(
        delta_y / 2) * math.sin(delta_y / 2);
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    dist = R * c
    return dist



##### print out edge types and number of edges per type #####
def print_edge_types(graph):
    categories = dict()
    edge_list_raw = list(graph.edges.items())
    for edge in edge_list_raw:
        if str(edge[1]["highway"]) in categories:
            categories[str(edge[1]["highway"])] +=1
        else:
            categories[str(edge[1]["highway"])] = 1

    for i in categories.items():
        print(i)

##### calc euclidian distance between two coordinates #####
def euclidian_distance(Startnode, Endnode):
    R = 6371e3  # metres
    delta_x = (Startnode[0] - Endnode[0]) / 180 * math.pi
    delta_y = (Startnode[1] - Endnode[1]) / 180 * math.pi
    a = math.sin(delta_x / 2) * math.sin(delta_x / 2) + math.cos(delta_x) * math.cos(delta_y) * math.sin(
        delta_y / 2) * math.sin(delta_y / 2);
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    dist = R * c
    return dist


######find intersection between two streets ######
def find_corner(graph, s1, s2):
    edge_list  = list(graph.edges.items())

    street_1=[]
    street_2=[]
    for edge in edge_list:
        if edge[1]["name"] == s1:
            street_1.append(edge)
        if edge[1]["name"]  == s2:
            street_2.append(edge)

    st1_nodes=[edge[0][1] for edge in street_1]
    st2_nodes=[edge[0][1] for edge in street_2]

    results=[]
    for i in st1_nodes:
        if i in st2_nodes:
            results.append(i)
    return int(results[0])

#####get elevation data from airmap api: 30m grid, 0 decimals #######

def get_node_elevations(graph):
    nodes = list(graph.nodes().items())
    node_list = []
    list_of_lists = []
    elevations_list = []

    for node in list(nodes):
        node_list.append(node[0])
    for sublist in range(0, math.floor(len(nodes) / 500)):
        list_of_lists.append(nodes[500 * sublist: 500 * (sublist + 1)])
    list_of_lists.append(nodes[500 * math.floor(len(nodes) / 500):])
    for l in list_of_lists:
        coordinates = ""
        for node in l:
            coordinates += str(node[1]["y"]) + "," + str(node[1]["x"]) + ","
        url = "https://api.airmap.com/elevation/v1/ele/?points=" + coordinates[:-1]
        r = urllib.request.Request(url)
        output = str(urllib.request.urlopen(r).read().decode("utf-8"))
        elevations_list.extend(output.split("\"data\":[")[1].split("]")[0].split(","))
    node_elevation = dict()
    for i in range(0, len(nodes)):
        if not elevations_list[i] == "null":
            node_elevation[node_list[i]] = int(elevations_list[i])
        else:
            coordinates = str(nodes[i]["y"]) + "," + str(nodes[i]["x"])
            url = "https://api.airmap.com/elevation/v1/ele/?points=" + coordinates
            r = urllib.request.Request(url)
            output = str(urllib.request.urlopen(r).read().decode("utf-8"))
            elevations_list[i] = output.split("\"data\":[")[1].split("]")[0].split(",")
            if (elevations_list[i]=='null'):
                print('no elevation existing.')
    return node_elevation  # , failed_nodes





#####Extracts only relevant information of graph into new MultiDiGraph #######

def clean_up_graph(graph, irrelevant_edge_types=[], relevant_edge_types=[], exclude_irrelevant=True, include_only_relevant=False):
    graph2 = nx.MultiDiGraph()
    graph2.graph = graph.graph
    edge_list_raw = list(graph.edges.items())

    for edge in edge_list_raw:
        # if not "maxspeed" in edge[1]:
        #     edge[1]["maxspeed"] = 50
        irrelevant = False
        for i in irrelevant_edge_types:
            if i in edge[1]["highway"]:
                irrelevant = True
        if not exclude_irrelevant:
            irrelevant = False
        relevant = False
        for i in relevant_edge_types:
            if i in edge[1]["highway"]:
                relevant = True
        if not include_only_relevant:
            relevant = True
        if not int(edge[0][0]) == int(edge[0][1]) and relevant and not irrelevant:
            start = int(edge[0][0])
            end = int(edge[0][1])
            try:
                maxspeed = int(edge[1]["maxspeed"])
            except:
                try:
                    maxspeed = int(edge[1]["maxspeed"][0])
                except:
                    if edge[1]["highway"] == "motorway":
                        maxspeed = 130
                    else:
                        maxspeed = 50
            try:
                name = edge[1]["name"]
            except:
                name = "no_name"
            osmid = edge[1]["osmid"]
            length = int(edge[1]["length"])
            oneway = edge[1]["oneway"]
            try:
                geometry = edge[1]["geometry"]
                graph2.add_edge(start, end, osmid=osmid, length=length, maxspeed=maxspeed, oneway=oneway, name=name,
                                geometry=geometry)
            except:
                graph2.add_edge(start, end, osmid=osmid, length=length, maxspeed=maxspeed, oneway=oneway, name=name)

    # remove isolated nodes
    isolates = nx.Graph()
    isolates.add_nodes_from(nx.isolates(graph2))
    graph2.remove_nodes_from(isolates)
    for node in graph2.nodes():
        graph2.nodes[node]["x"] = graph.nodes[node]["x"]
        graph2.nodes[node]["y"] = graph.nodes[node]["y"]
        graph2.nodes[node]["pos"] = (float(graph.nodes[node]["y"]), float(graph.nodes[node]["x"]))
        graph2.nodes[node]["osmid"] = graph.nodes[node]["osmid"]
    return graph2



