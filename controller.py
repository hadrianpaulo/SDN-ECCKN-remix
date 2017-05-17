from copy import deepcopy

import networkx as nx
import pandas as pd

from sensor import Sensor
from utils import get_dupes, complete_graph_from_list

"""
SDN based architecture is applied to each
node, which has the following advantages:
1) All the decisions are made by the controller which is usually connected
with constant power supply;
2) Each node transmits its beacon data via certain route in the initial full
topology and transmits its main data only to the next-hop node based on
the decision made by the controller.
3) There is no data exchange (broadcasting) between nodes to calculate its
own status during the whole interval. Hence, there is no broadcasting during
the entire network lifetime, which dramatically reduces the total transmission
times during the network lifetime.
4) Controller only transmits decision packet to the nodes whose sleep status
or next-hop nodes will change
"""


class Controller:
    def __init__(self, n_nodes=150):
        """
        :param n_nodes: Number of Sensor nodes that this controller initiates
        """
        self.node = Sensor(position=(100, 100))
        self.name = str(self.node.get_position())
        self.n_nodes = n_nodes
        self.shortest_path = {}
        self.orig_topology = None
        self.current_topology = None

        self.df_E = pd.DataFrame()

        while True:
            self.sensor_nodes = [Sensor() for _ in range(self.n_nodes)]
            self.sensor_node_pos = {node.name: node.get_position()
                                    for node in self.sensor_nodes}

            self.sensor_nodes.append(self.node)
            self.sensor_node_pos[self.node.name] = self.node.get_position()

            self.orig_topology = complete_graph_from_list(self.sensor_nodes)
            if len(list(get_dupes(self.sensor_nodes))) == 0:
                break

        self.sensor_nodes.sort(reverse=True)
        node_names_list = [node.name for node in self.sensor_nodes]
        self.node_attr = dict(zip(node_names_list, self.sensor_nodes))

        # Each node transmits its beacon data via certain route in the initial full topology
        # Just do has_path to Controller node with nearest neighbor node to determine target
        # for beacon data transmission
        print(nx.edges(self.orig_topology))

    def update_topology_shortest_path(self, new_graph=None):
        """
        Edges are discarded if E rank of sensor
        nodes are insufficient to reach other nodes
        """
        res_graph = deepcopy(new_graph)
        edge_iter = list(res_graph.edges_iter(data=True))
        for u, v, w in edge_iter:
            node_1 = self.node_attr[u]
            node_2 = self.node_attr[v]
            # E_rank_u_requirement = node_1.E_elec * node_1.l + \
            #                        node_1.eps_amp * node_1.l * w['weight'] ** 2
            # if E_rank_u_requirement >= node_1.E_rank_u or node_1.is_asleep() or node_2.is_asleep():
            #     res_graph.remove_edge(node_1.name, node_2.name)
        shortest_path = {
            'path': nx.all_pairs_dijkstra_path(res_graph),
            'weight': nx.all_pairs_dijkstra_path_length(res_graph)}

        return res_graph, shortest_path
