"""Road network model

This file contains the main road network model class


Classes:
    RoadNetworkModel

"""

import xml.etree.ElementTree as ET
import networkx as nx
import re, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from model.base_components import *
from model.system_components import *
# import sumolib

class RoadNetworkModel():
    """Read an xml file and construct a representation of the road network"""
    def __init__(self, fileroot, name, shortest_paths = False):

        # initialize components and parse file
        self.name = name
        self.shortest_paths = shortest_paths

        self.junctions = {}
        self.edges = {}
        self.edge_systems = {}
        self.bounds = {}
        self.graph = {}
        self.path_systems = {}
        self.custom_systems = {}

        # read low-level edges
        self.read_model(os.path.join(fileroot, name))

        # represent as directed graph
        self.construct_graph()

        # get graph entrances/exits (not of these types) and paths btw. them
        paths = self.get_paths(self.graph,
            {'highway.residential', 'highway.service'}) #rev why exclude these

        self.path_systems = self.get_path_systems(paths)

        # self.add_custom_system("Entire network", self.edges.keys())

        self.add_intersections()


    def read_model(self, filename):
        """Parse a road network model from xml format to dictionary."""

        # parse file
        root = ET.parse(filename).getroot()

        # net = sumolib.net.readNet(filename)
        
        # iterate over network edges
        for edge_xml in root.findall('edge'):

            # create edge and edge system objects store, them keyed by id
            lanes = [lane_xml.attrib for lane_xml in edge_xml.findall('lane')]
            edge = Edge(edge_xml.attrib, lanes)
            self.edges[edge.id] = edge
            self.edge_systems[edge.id] = EdgeSystem(edge)


        # iterate over network junctions
        for junction_xml in root.findall('junction'):

            # create junction objects and store keyed by id (when possible)
            if junction_xml.attrib['type'] != "internal":
                junction = Junction(junction_xml.attrib)
                self.junctions[junction.id] = junction


        # get boundaries of converted and original coordinates
        self.convBoundary = [float(coord) for coord in
            root.findall('location')[0].attrib['convBoundary'].split(',')]
        self.origBoundary = [float(coord) for coord in
            root.findall('location')[0].attrib['origBoundary'].split(',')]
        

    def construct_graph(self):
        """Create a directed graph representation fo the system."""

        self.graph = nx.DiGraph([(edge.from_id, edge.to_id, {'edge': edge})
            for edge in self.edges.values()])

        nx.set_node_attributes(self.graph, {junction.id:{"type":junction.type} for junction in self.junctions.values()})

    def get_paths(self, G, excluded_types={}):
        """Get all simple paths from entrances to exits of a given graph."""

        all_entrances = {node for node, in_degree in self.graph.in_degree()
            if in_degree == 0}
        all_exits = {node for node, out_degree in self.graph.out_degree()
            if out_degree == 0}

        # Nodes of certain types are not allowed to be entrances or exits, so
        # we determine the type of the junction based on its edges and then
        # we discard node if it's only connected to edges with excluded types
        filtered_entrances = {node for node in all_entrances 
            if not ({edge[2]['edge'].type
                for edge in self.graph.out_edges(node,True)} # edge types
                .issubset(excluded_types))}    # only connected to excl. types 

        # we repeat the same provess for exits
        filtered_exits = {node for node in all_exits 
            if not ({edge[2]['edge'].type
                for edge in self.graph.in_edges(node,True)}  # edge types
                .issubset(excluded_types))}    # only connected to excl. types

        # get all combinations of entrances and exits
        routes = ((source, target) for source in filtered_entrances
            for target in filtered_exits)
            
        # find all paths
        if not self.shortest_paths:
            
            # get all simple paths, groupped by route, as sequences of nodes
            pathlist = (nx.all_simple_paths(G, source, target)
                for (source, target) in routes)

        # find only shortest paths
        else:
            pathlist = ([nx.shortest_path(G, source, target)]
                for (source, target) in routes
                if nx.has_path(G, source, target)) #rev why not in above if statement

        
        # flatten list and key paths by source-target pair 
        # (for paths with same source and target, also use incremental id)
        paths = {}
        for route in pathlist:
            for index, path in enumerate(route):

                # get first and last edge in path
                source = self.graph.get_edge_data(path[0], path[1])['edge'].id
                target = self.graph.get_edge_data(
                    path[len(path)-2], path[len(path)-1])['edge'].id

                paths[source + "->" + target.replace("#","_") + "|" + str(index)] = path


        # return flattenned list of paths, no longer groupped by route
        return paths

        
    def get_path_systems(self, paths):
        """Construct path system objects based on a set of paths."""

        path_systems = {}

        # for each path, get both normal and internal edges
        for path_id, path in paths.items():
            
            # get internal edges in each junction, flatten list
            junctions = (self.junctions[node] for node in path)
            internal_edges = {edge for junction in junctions
                for edge in junction.internal_edges}

            # get path as sequence of edges, make edge tuples to Edge objects
            normal_edges = [self.graph.get_edge_data(u,v)['edge'].id
                for u,v in nx.utils.pairwise(path)]

            # get ids of all normal and internal edges actually in the graph 
            valid_edges = set(self.edges).intersection(
                internal_edges.union(normal_edges))
            
            # create PathSystem object and add it to the list
            path_systems[path_id] = (PathSystem( path_id,
                (self.edge_systems[edge_id] for edge_id in valid_edges)))

        return path_systems


    def add_custom_system(self, name, edges):
        """Add a user-created multi-edge system to the model."""

        # create system, assign incremental id based on custom systems count
        system = CustomSystem( len(self.custom_systems),
                (self.edge_systems[edge_id] for edge_id in set(edges)), name)

        self.custom_systems[system.id] = system

    def add_intersections(self):
        junctions = [x for x,y in self.graph.nodes(data=True) if "type" in y and y['type']=="traffic_light" and len(self.graph.out_edges(x))>2]
        i=0
        for junction in junctions:
            in_edges = [edge[2]['edge'].id for edge in self.graph.in_edges(junction,data=True)]
            out_edges = [edge[2]['edge'].id for edge in self.graph.out_edges(junction,data=True)]
            i+=1
            self.add_custom_system("inter_"+str(i), out_edges+in_edges)

            # if(i == 8):
            #     g = self.graph
            #     # g = self.graph.edge_subgraph([i for i in self.graph.out_edges(junction)]+[i for i in self.graph.in_edges(junction)])
            #     plt.figure(num=None, figsize=(20, 20), dpi=80)
            #     fig = plt.figure(1)
            #     ax=fig.gca()
            #     # ax=f.add_subplot(111)
            #     nx.draw(g, ax=ax, node_size = 200)
            #     fig.savefig("graph.png",bbox_inches="tight",pad_inches = 0.1)


            entrances = [edge[0] for edge in self.graph.in_edges(junction)]
            exits = [edge[1] for edge in self.graph.out_edges(junction)]

            routes = ((source, target) for source in entrances for target in exits)

            # get all simple paths, groupped by route, as sequences of nodes
            pathlist = (nx.all_simple_paths(self.graph.subgraph(entrances+exits+[junction]), source, target) for (source, target) in routes)

            # flatten list and key paths by source-target pair 
            # (for paths with same source and target, also use incremental id)
            paths = {}
            for route in pathlist:
                for index, path in enumerate(route):

                    # get first and last edge in path
                    source = self.graph.get_edge_data(path[0], path[1])['edge'].id
                    target = self.graph.get_edge_data(
                        path[len(path)-2], path[len(path)-1])['edge'].id

                    paths[source + "->" + target.replace("#","_") + "|" + str(index)] = path

            
            path_systems = {}
            j = 0
            # for each path, get both normal and internal edges
            for path_id, path in paths.items():
                
                # get internal edges in each junction, flatten list
                # junctions = (self.junctions[node] for node in path)
                # internal_edges = {edge for junction in junctions
                #     for edge in junction.internal_edges}

                # get path as sequence of edges, make edge tuples to Edge objects
                normal_edges = [self.graph.get_edge_data(u,v)['edge'].id
                    for u,v in nx.utils.pairwise(path)]

                # get ids of all normal and internal edges actually in the graph 
                # valid_edges = set(self.edges).intersection(
                #     internal_edges.union(normal_edges))
                
                # create PathSystem object and add it to the list
                # path_systems[path_id] = (PathSystem( path_id,
                #     (self.edge_systems[edge_id] for edge_id in valid_edges)))

                j+=1
                self.add_custom_system("inter_"+str(i)+"_"+str(j), normal_edges)