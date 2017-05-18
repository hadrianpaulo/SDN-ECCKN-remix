import itertools
from enum import Enum

import networkx as nx


def complete_graph_from_list(node_list):
    graph = nx.empty_graph(0)
    node_names_list = [node.get_name() for node in node_list]
    node_attr = dict(zip(node_names_list, node_list))
    if len(node_list) > 1:
        if graph.is_directed():
            edges = itertools.permutations(node_names_list, 2)
        else:
            edges = itertools.combinations(node_names_list, 2)
        edges = list(edges)
        edges_w = []
        for i in edges:
            edges_w.append(
                i + (calc_sensor_distance(node_attr[i[0]], node_attr[i[1]]),))
        graph.add_weighted_edges_from(edges_w)
        nx.set_node_attributes(graph, 'sensor', node_attr)
    return graph


def get_dupes(c):
    a, b = itertools.tee(sorted(c))
    next(b, None)
    r = None
    for k, g in zip(a, b):
        if k.get_name() != g.get_name():
            continue
        if k.get_name() != r.get_name():
            yield k
            r = k


def calc_sensor_distance(s1, s2):
    """
    :param s1: Sensor object 1
    :param s2: Sensor object 2
    Get distance from specified node
    """
    x1, y1 = s1.get_position()
    x2, y2 = s2.get_position()
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


class State(Enum):
    INIT = 1
    AWAKE = 2
    SLEEP = 3
    DEAD = 4

    @classmethod
    def is_valid(cls, initial_state, target_state):
        if initial_state == State.INIT:
            if target_state == State.AWAKE or target_state == State.SLEEP:
                return True
        elif initial_state == State.AWAKE or initial_state == State.SLEEP:
            if target_state == State.AWAKE or target_state == State.SLEEP or target_state == State.DEAD:
                return True
        elif initial_state == State.DEAD and target_state == State.DEAD:
            return True
        else:
            return False
