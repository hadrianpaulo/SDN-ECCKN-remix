from copy import deepcopy

import networkx as nx
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt

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
        self.node = Sensor(position=(100, 100), controller=True)
        self.name = self.node.get_name()
        self.n_nodes = n_nodes
        self.shortest_path = {}
        self.orig_topology = None
        self.current_topology = None

        self.df_E = pd.DataFrame()

        while True:
            self.sensor_nodes = [Sensor() for _ in range(self.n_nodes)]
            self.sensor_node_pos = {node.get_name(): node.get_position()
                                    for node in self.sensor_nodes}

            self.sensor_nodes.append(self.node)
            self.sensor_node_pos[self.node.get_name(
            )] = self.node.get_position()

            self.orig_topology = complete_graph_from_list(self.sensor_nodes)
            if len(list(get_dupes(self.sensor_nodes))) == 0:
                break

        self.sensor_nodes.sort(reverse=True)
        node_names_list = [node.get_name() for node in self.sensor_nodes]
        self.node_attr = dict(zip(node_names_list, self.sensor_nodes))

        # INITIALIZE AND SET BEACON TARGETS
        # Each node transmits its beacon data via certain route in the initial
        # full topology

        #
        # # Just do has_path to Controller node with nearest neighbor node to determine target
        # # for beacon data transmission
        # #       ASSUME: 5 nearest to controller node will target controller node,
        # #       others target nearest neighbor
        # nearest_five_controller = sorted(self.orig_topology[self.node.get_name(
        # )].items(), key=lambda edge: edge[1]['weight'])[:5]
        # for node_name, d in nearest_five_controller:
        #     self.node_attr[node_name].set_target(
        #         self.node, d['weight'], main=False)
        #
        # remaining_uninitialized_nodes = [x for x in node_names_list if x not in set(
        #     [idx for idx, val in nearest_five_controller])]
        # for node_name in remaining_uninitialized_nodes:
        #     temp, d = sorted(self.orig_topology[node_name].items(
        #     ), key=lambda edge: edge[1]['weight'])[0]
        #     self.node_attr[node_name].set_target(
        #         self.node_attr[temp], d['weight'], main=False)
        #
        # for node in self.sensor_nodes:
        #     print(node.get_name())
        #     print(node.E_rank_u_neighbors_beacon)
        #     node.transmit(main=False)

        # Assume all beacon targets are direct to controller
        sensor_nodes_controller = sorted(self.orig_topology[self.node.get_name(
        )].items(), key=lambda edge: edge[1]['weight'])
        for node_name, d in sensor_nodes_controller:
            self.node_attr[node_name].set_target(
                self.node, d['weight'], main=False)

        # transmits its main data only to the next-hop node based on
        # the decision made by the controller
        # 'Initial Topology and Shortest Path calculations')
        self.orig_topology, self.shortest_path = self.update_topology_shortest_path(
            init=True)
        # 'Removing untraversable edges')
        self.current_topology, self.shortest_path = self.update_topology_shortest_path()
        # Setting initial sensor node targets')
        self.update_sensor_node_targets()

    def update_topology_shortest_path(self, new_graph=None, init=False):
        """
        Edges are discarded if E rank of sensor
        nodes are insufficient to reach other nodes
        """
        if new_graph is None:
            new_graph = self.orig_topology

        shortest_path = {'path': nx.all_pairs_dijkstra_path(
            new_graph), 'weight': nx.all_pairs_dijkstra_path_length(new_graph)}

        res_graph = deepcopy(new_graph)

        # Runs at O(E log N) due to djikstra's and edge traversal
        # What if we finally have edges which can be added once E_rank_u is
        # replenished?
        if not init:
            edge_iter = list(res_graph.edges_iter(data=True))
            for u, v, w in edge_iter:
                node_1 = self.node_attr[u]
                node_2 = self.node_attr[v]
                E_rank_u_requirement = node_1.E_elec * node_1.l_main + \
                    node_1.eps_amp * node_1.l_main * w['weight'] ** 2
                E_rank_u_requirement_2 = node_2.E_elec * node_2.l_main + \
                    node_2.eps_amp * node_2.l_main * w['weight'] ** 2
                if E_rank_u_requirement >= node_1.E_rank_u or E_rank_u_requirement_2 >= node_2.E_rank_u \
                        or node_1.is_asleep() or node_2.is_asleep():
                    res_graph.remove_edge(node_1.get_name(), node_2.get_name())
            shortest_path = {
                'path': nx.all_pairs_dijkstra_path(res_graph),
                'weight': nx.all_pairs_dijkstra_path_length(res_graph)}

        return res_graph, shortest_path

    def update_sensor_node_targets(self):
        for node in self.sensor_nodes:
            try:
                try:
                    target = self.shortest_path['path'][self.node.get_name(
                    )][node.get_name()]
                    distance = self.shortest_path['weight'][self.node.get_name(
                    )][node.get_name()]
                    target.reverse()
                    target_node = self.node_attr[target[1]]
                except KeyError:
                    target_node = None
                    distance = 0
                node.set_target(target_node, distance)
                node.wake_up()
            except IndexError:
                # print('Unreachable! Path at ', target)
                if node.is_controller:
                    node.set_target(None, 0)

    def update_sensor_properties(self):
        for node in self.sensor_nodes:
            node.update_properties()

    def draw(self, fn=None):
        if fn is None:
            fn = "graph.png"
        f = plt.figure()
        nx.draw_networkx(
            self.current_topology,
            self.sensor_node_pos,
            node_size=10,
            with_labels=False,
            width=0.05,
            alpha=0.5, ax=f.add_subplot(111))
        f.savefig(fn)
        plt.close(f)

    def save_erank(self, col=0):
        self.df_E['time_' +
                  str(col)] = pd.Series(self.node.E_rank_u_neighbors_main)

    def export_erank(self):
        self.df_E.to_csv('df_erank.csv')

    def __repr__(self):
        return self.name

    def get_isolated_nodes(self):
        isolated = []
        for node in self.sensor_nodes:
            if node.isolated:
                isolated.append(node.get_name())
        return len(isolated), isolated

    def get_sleeping_nodes(self):
        sleeping = []
        for node in self.sensor_nodes:
            if node.is_asleep():
                sleeping.append(node.get_name())
        return len(sleeping), sleeping

    def get_dead_nodes(self):
        dead = []
        for node in self.sensor_nodes:
            if node.is_dead():
                dead.append(node.get_name())
        return len(dead), dead

    def get_alive_nodes(self):
        alive = []
        for node in self.sensor_nodes:
            if node.is_awake():
                alive.append(node.get_name())
        return len(alive), alive

        # TODO: ECCKN Algorithm :(
        # "1. Get the information of current remaining energy $E_{rank_u}$;\n",
        # "2. Broadcast $E_{rank_u}$ and receive the energy ranks
        #     of its currently awake neighbors $N_u$.
        #     Let $R_u$ be the set of these ranks.\n",
        # "3. Broadcast $R_u$ and receive $R_v$ from each $s_v$ ∈ $N_u$.\n",
        # "4. If |$N_u$| < k or |$N_v$| < k for any $s_v$ ∈ $N_v$, remain awake. Return.\n",
        # "5. Compute $E_u$ = {$s_v$|$s_v$ ∈ $N_u$ and $E_{rank_v}$ > $E_{rank_u}$};\n",
        # "6. Go to sleep if both the following conditions hold. Remain awake otherwise.\n",
        # "    * Any two nodes in $E_u$ are connected either directly themselves or indirectly
        #        through nodes which is in the $s_u$’s 2-hop neighborhood that have $E_{rank_v}$
        #        larger than $E_{rank_u}$;\n",
        # "    * Any node in $N_u$ has at least k neighbors from $E_u$.\n",
        # "7. Return.\n",
        # "\n",
        # "This algorithm gets run per node, where $s_u$ is the current node
        #  and $s_v$ is a neighbor node.\n",

    def run_ECCKN(self, k=3):
        for node in self.sensor_nodes:
            N_u = self.orig_topology.neighbors(node.get_name())
            N_u_awake = sum(
                [True if self.node_attr[node].is_awake() else False for node in N_u])
            E_u = {}
            # If |$N_u$| < k or |$N_v$| < k for any $s_v$ ∈ $N_v$, remain
            # awake. Return.

            # print(N_u_awake)
            if N_u_awake < k:
                # Remain awake
                node.wake_up()
                # print(node, " remains awake!")
                continue
            else:
                cont = False
                for neighbor_node in N_u:
                    N_v = self.orig_topology.neighbors(neighbor_node)
                    N_v_k_awake = sum(
                        [True if self.node_attr[node].is_awake() else False for node in N_v])
                    if N_v_k_awake < k:
                        # Remain awake
                        # print(node, " remains awake!")
                        node.wake_up()
                        cont = True
                        break
                if cont:
                    continue

                # Compute $E_u$ = {$s_v$|$s_v$ ∈ $N_u$ and $E_{rank_v}$ >
                # $E_{rank_u}$}",
                if self.node_attr[neighbor_node].E_rank_u > node.E_rank_u:
                    E_u[neighbor_node] = True

                # TODO: Go to sleep if both the following conditions hold.
                # Remain awake otherwise.\n",
                #    * Any two nodes in $E_u$ are connected either:
                #        * directly themselves
                #        * indirectly through nodes which is in the $s_u$’s 2-hop neighborhood
                #          that have E_rank_v larger than E_rank_u
                #
                #    * Any node in $N_u$ has at least k neighbors from $E_u$.\n",
                # AKA Any neighbor of node_u should have k neighbors in E_u

                cond1 = False
                for node_v1 in E_u.keys():
                    for node_v2 in E_u.keys():
                        hop_1 = self.current_topology.has_edge(
                            node_v1, node_v2)
                        hop_2_nodes = list(nx.common_neighbors(
                            self.current_topology, node_v1, node_v2))
                        hop_2_nodes_awake = sum([True if self.node_attr[node].is_awake()
                                                 else False for node in hop_2_nodes])
                        # node_u or node_v should have higher E_rank than node
                        if hop_1:
                            cond1 = True
                        if hop_2_nodes_awake > 1:
                            for node_2 in hop_2_nodes:
                                if self.node_attr[node_2].E_rank_u > node.E_rank_u and \
                                        not self.node_attr[node_2].is_controller:
                                    cond1 = True

                cond2 = False
                for neighbor in N_u:
                    res = 0
                    neighbor_neighbor = self.current_topology.neighbors(
                        neighbor)
                    for n in neighbor_neighbor:
                        try:
                            if E_u[n]:
                                res += 1
                        except KeyError:
                            # print('Not a neighbor of ', n)
                            continue
                    if res >= k:
                        cond2 = True
                        break

                if cond1 and cond2:
                    # print("node " + node.name + " sleeps!")
                    node.sleep()
                else:
                    node.wake_up()
