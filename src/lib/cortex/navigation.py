import heapq
import math
from typing import List, Optional, Tuple

# from scipy.interpolate import interp1d
# from src.lib.cortex import cubic_spline_planner
import networkx as nx
import numpy as np
from copy import deepcopy
from src.config import get_config

config = get_config()

# from src.config import config

target_idx = config["end_idx"]


def dijkstra(G, start, target):
    d = {start: 0}
    parent = {start: None}
    pq = [(0, start)]
    visited = set()
    while pq:
        du, u = heapq.heappop(pq)
        if u in visited:
            continue
        if u == target:
            break
        visited.add(u)
        for v in G.adj[u]:
            if v not in d or d[v] > du + 1:
                d[v] = du + 1
                parent[v] = u
                heapq.heappush(pq, (d[v], v))

    fp = [target]
    tg = target
    ptype = [("lk" if len([i for i in G.neighbors(tg)]) < 2 else "int")]

    while tg != start:
        fp.insert(0, parent[tg])
        tg = parent[tg]
        ptype.insert(0, ("lk" if len([i for i in G.neighbors(tg)]) < 2 else "int"))
        # print([i for i in G.neighbors(tg)])

    ptyperet = ptype.copy()
    for i in range(len(ptype)):
        if ptype[i] == "int":
            try:
                ptyperet[i - 1] = "int"
            except Exception as e:
                print("pathplanning.py[46]", e)

            try:
                ptyperet[i + 1] = "int"
                i += 1
            except Exception as e:
                print("pathplanning.py[52]", e)

    edgeret = []

    for i in range(len(fp) - 1):
        dt = G.get_edge_data(fp[i], fp[i + 1])
        edgeret.append(dt["dotted"])

    edgeret.append(None)

    return fp, ptyperet, edgeret


def add_yaw(G):
    node_dict = deepcopy(dict(G.nodes(data=True)))
    for current_node in node_dict:
        for v in G.adj[current_node]:
            c_data = node_dict[current_node]
            v_data = node_dict[v]
            dx = v_data["x"] - c_data["x"]
            dy = v_data["y"] - c_data["y"]
            # print(c_data, v_data, "\n", dx, dy)
            yaw = math.atan2(dy, dx)
            if "yaw" in node_dict[current_node]:
                node_dict[current_node]["yaw"] += [yaw]
            else:
                node_dict[current_node].update({"yaw": [yaw]})
    return node_dict


class Navigator:
    def __init__(self, test=False):
        if test:
            self.graph = nx.read_graphml(
                "./src/lib/cortex/path_data/test_track.graphml"
            )
        else:
            self.graph = nx.read_graphml(
                "./src/lib/cortex/path_data/comp_track.graphml"
            )

        self.node_dict = add_yaw(self.graph)
        self.cur_index = 0
        self.path = []
        self.ptype = []
        self.etype = []

    def plan_course(self, cofig_json):
        raise NotImplementedError

    def replan_course(self, completed_list, config_json):
        raise NotImplementedError

    def get_course_ahead(self, x, y, yaw):
        raise NotImplementedError

    def get_current_node(self, x, y, yaw):
        idx = self.get_nearest_node(self, x, y, yaw)
        return (
            self.path[idx],
            self.ptype[idx],
            self.etype[idx],
        )

    def get_nearest_node(self, x, y, yaw):
        dx = []
        dy = []
        for node in self.node_dict:
            dx.append(self.node_dict[node]["x"] - x)
            dy.append(self.node_dict[node]["y"] - y)

        d = np.hypot(dx, dy)
        idxs = np.argsort(d)
        for idx in idxs:
            try:
                dyaw = np.array(self.node_dict[str(idx)]["yaw"]) - yaw
            except KeyError as e:
                print(e)
                continue
            if (abs(dyaw) < 0.7).any():
                return idx  # , self.node_dict[str(idx)]

    def get_path_ahead(self, x, y, yaw):

        idx = self.get_nearest_node(self, x, y, yaw)
        return (
            self.path[idx:],
            self.ptype[idx:],
            self.etype[idx:],
        )

    # use this (if start idx is know pass it to kwarg start_idx)
    def plan_path(self, x, y, yaw, start_idx: Optional[int] = -1):
        """Given pos get update path"""
        if start_idx == -1:
            start_idx = self.get_nearest_node(x, y, yaw)

        path_list, _ptype, _edgret = dijkstra(self.graph, start_idx, target_idx)
        self.path = self._convert_nx_path2list(path_list)
        self.ptype = _ptype
        self.etype = _edgret

    # def get_path(self, start_idx: str, end_idx: str) -> Tuple[List[Tuple[int]], str]:
    #     path_list, _ptype, _edgret = dijkstra(self.graph, start_idx, target_idx)
    #     self.path = self._convert_nx_path2list(path_list)
    #     self.ptype = _ptype
    #     self.etype = _edgret

    def _convert_nx_path2list(self, path_list) -> List[Tuple[int]]:
        coord_list = []
        for i in path_list:
            data = self.node_dict[i]
            coord_list.append([data["x"], data["y"]])
        return coord_list