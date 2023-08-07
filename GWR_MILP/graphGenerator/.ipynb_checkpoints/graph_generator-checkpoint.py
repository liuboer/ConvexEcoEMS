"""@file graph_generator.py
@brief The file contains main to generate the graph for the routing optimization problem.

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
import os
import csv
import src.graph_compressor as compressor
import geopandas as gp

###################################### user inputs  ############################################
# !!!!!!!!!! please read the next few lines :) !!!!!!!!!!
# This script will download map data from osm and save an osm file, a node file and an edges file.
# The following lines of code contain all relevant user input.
# Note that the download step and the compression step can take a while for large maps.
# If the download seems to take forever (high two-digit minutes) try again later (e.g. in the morning).
# If the node elevation update throws an error (usually cannot convert 'none' to float), try again later as well. This is an API issue.
# A few lines into the script, the function ox.graph_from_point is used. You can also save the "raw" graph or load a "raw" graph. We recommend this when playing with the compression level.
# You need an internet connection with this script even if loading a raw graph from a .osm file (to get elevation data).
# we recommend to save the console output in a txt file, as it might come in handy later, especially start and end node ID
# For information on road types see https://wiki.openstreetmap.org/wiki/Key:highway. Note that edges may have multiple types.

start_adress = "51, Forckenbeckstraße, 52074 Aachen"
destination_adress = "1, Schroufstraße , 52078 Aachen"
# A round map section will be downloaded with map_margin times the minimum necessary radius (aka bee-line distance of start and end node)
map_margin = 1.2
#cluster nodes that are close together to reduce map size
cluster_size = 200
#can be "edge length", which only clustered nodes that are actually connected, or "euclidian distance"
cluster_by = "euclidean distance"
#this is a custom filter for the osm download. Absolutely necessary for large maps, or download will take days.
# https://wiki.openstreetmap.org/wiki/Key:highway
osm_filter = '["highway"~"motorway|trunk|motorway_link|trunk_link|primary|secondary|tertiary|unclassified"]'
# Additional Filters
irrelevant_edge_types = ["service",  "pedestrian", "path", "cycleway", "footway", "steps", "residential", "living_street"]
exclude_irrelevant_edge_types = False
relevant_edge_types=[]
include_relevant_edge_types_only = False

###################################### start scipt ############################################

folder = os.path.dirname(os.path.realpath(__file__))
print("... Downloading OSM data ...")
start_coordinates = ox.geocode(start_adress)
end_coordinates = ox.geocode(destination_adress)
centerpoint = ((start_coordinates[0] + end_coordinates[0])/2,(start_coordinates[1] + end_coordinates[1])/2)
dist = compressor.euclidian_distance(start_coordinates, centerpoint) * map_margin
print("Distance between start and endpoint: " + str(round(dist/map_margin*2,1)) + " km bee-line")
graph = ox.graph_from_point(centerpoint, dist=dist, custom_filter=osm_filter)
# ox.io.save_graphml(graph, filepath=folder + "\\"  + "graph_raw.osm") # use these lines to work offline by saving "raw" graphs
# graph = ox.io.load_graphml(filepath=folder + "\\"  + "graph_raw.osm")
fig, ax = ox.plot_graph(graph)
print("Number of edges, raw graph: " + str(len(graph.edges.items())))
print("Number of nodes, raw graph: " + str(len(graph.nodes.items())))

# print_edge_types(graph)  #get overview of which types of roads the network consists of.


# graph cleanup (compression by type of road and unifying attribute lists of edges)
print("... Cleanup: removing irrelevant edges ...")
graph2=compressor.clean_up_graph(graph, irrelevant_edge_types=irrelevant_edge_types, relevant_edge_types=[], exclude_irrelevant=exclude_irrelevant_edge_types, include_only_relevant=include_relevant_edge_types_only)
print("Number of edges after cleanup: " + str(len(graph2.edges.items())))
print("Number of nodes after cleanup: " + str(len(graph2.nodes.items())))

# Find specified start and end nodes
print("Searching for specified start and end nodes")
start_node = ox.get_nearest_node(graph2, start_coordinates) #find_corner(graph2, "Eilendorfer Straße", "Freunder Landstraße")
end_node = ox.get_nearest_node(graph2, end_coordinates) # find_corner(graph2, "Campus-Boulevard", "Forckenbeckstraße")
print("identified nodes:\nstart node: " + str(start_node) + "\nend node: " + str(end_node))

# graph compression by trivial nodes in the middle of edges and by node clusters
print("... Compressing graph ...")
graph3 = compressor.compress(graph2, cluster_size = cluster_size, cluster_by = cluster_by, keep_nodes=[start_node, end_node])
fig, ax = ox.plot_graph(graph3)
print("Final number of edges: " + str(len(list(graph3.edges.items()))))
print("Final number of nodes: " + str(len(list(graph3.nodes.items()))))


ox.io.save_graphml(graph3, filepath=folder + "\\" + "graph_intermediate.osm")
# update node elvations from API
print("... Updating node elevations ...")
node_elevation = compressor.get_node_elevations(graph3)
for node in graph3.nodes():
    graph3.nodes[node]["elevation"]=node_elevation[node]

#prepare nodes csv
table_of_nodes=[["node name", "node elevation", "x_pos", "y_pos"]]
for node in graph3.nodes():
    table_of_nodes.append([graph.nodes[node]["osmid"], node_elevation[node], graph.nodes[node]["x"], graph.nodes[node]["y"]])

#prepare edges csv
table_of_edges=[["edge_id", "length", "speed", "start", "end", "name", "oneway"]]
edge_list=list(graph3.edges.items())
e=1
for edge in edge_list:
    edge_id = "e" + str(e)
    e += 1
    start = int(edge[0][0])
    end = int(edge[0][1])
    maxspeed = int(edge[1]["maxspeed"])
    length = int(edge[1]["length"])
    name = edge[1]["name"]
    oneway = edge[1]["oneway"]
    table_of_edges.append([edge_id, length, maxspeed, start, end, name, oneway])


################################ save csv and osm files ##################################################

with open (folder + "\\nodes" + ".csv", "w", encoding="utf-8", newline="") as write_file:
    writer = csv.writer(write_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    for row in table_of_nodes:
        writer.writerow(row)

with open (folder + "\\edges" + ".csv", "w", encoding="utf-8", newline="") as write_file:
    writer = csv.writer(write_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    for row in table_of_edges:
        writer.writerow(row)

with open (folder + "\\start_and_destination" + ".txt", "w") as write_file:
    write_file.write(str(start_node) + "\n" + str(end_node))

file_name = folder + "\\" + ".osm"
ox.io.save_graphml(graph3, filepath= file_name )

outF = open(folder + "\\case_info" + ".txt", "w")