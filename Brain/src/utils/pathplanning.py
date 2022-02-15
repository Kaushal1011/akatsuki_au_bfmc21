import networkx as nx
from typing import *
import heapq

from src.templates.workerprocess import WorkerProcess
from threading import Thread
import numpy as np
import math

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

    while tg != start:
        fp.insert(0, parent[tg])
        tg = parent[tg]

    return fp


class PathPlanning:
    def __init__(self, test: bool = False) -> None:
        if test:
            self.graph = nx.read_graphml("/home/b0nzo/Documents/akatsuki_au_bfmc21/Brain/src/utils/path_data/test_track.graphml")
        else:
            self.graph = nx.read_graphml("/home/b0nzo/Documents/akatsuki_au_bfmc21/Brain/src/utils/path_data/comp_track.graphml")

        self.node_dict = self.graph.nodes(data=True)

    def get_path(self, start_idx: str, end_idx: str) -> List[Tuple[int]]:
        path_list = dijkstra(self.graph, start_idx, end_idx)
        return self._convert_nx_path2list(path_list)

    def _convert_nx_path2list(self, path_list) -> List[Tuple[int]]:
        coord_list = []
        for i in path_list:
            data = self.node_dict[i]
            coord_list.append([data["x"], data["y"]])
        return coord_list

class Purest_Pursuit:
    def __init__(self, coord_list):
        self.k = 0.01  # look forward gain
        self.Lfc = .50  # [m] look-ahead distance
        self.Kp = 1.0  # speed proportional gain
        self.WB = 0.3  # [m] wheel base of vehicle
        self.cx, self.cy = zip(*coord_list)
        self.old_nearest_point_index = None

    
    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = self.k * state.v + self.Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf
    
    def purest_pursuit_steer_control(self,state):
        ind, Lf = self.search_target_index(state)

        if ind < len(self.cx):
            tx = self.cx[ind]
            ty = self.cy[ind]
        else:  # toward goal
            tx = self.cx[-1]
            ty = self.cy[-1]
            ind = len(self.cx) - 1

        alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

        delta = math.atan2(2.0 * self.WB * math.sin(alpha) / Lf, 1.0)

        return delta

