import heapq
import math
from typing import List, Tuple

from scipy.interpolate import interp1d

import networkx as nx
import numpy as np


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
            except Exception:
                pass
            try:
                ptyperet[i + 1] = "int"
                i += 1
            except Exception:
                pass

    edgeret = []

    for i in range(len(fp) - 1):
        dt = G.get_edge_data(fp[i], fp[i + 1])
        edgeret.append(dt["dotted"])

    edgeret.append(None)

    return fp, ptyperet, edgeret


class PathPlanning:
    def __init__(self, test: bool = False) -> None:
        if test:
            self.graph = nx.read_graphml(
                "./src/lib/cortex/path_data/test_track.graphml"
            )
        else:
            self.graph = nx.read_graphml(
                "./src/lib/cortex/path_data/comp_track.graphml"
            )

        self.node_dict = self.graph.nodes(data=True)

    def get_path(self, start_idx: str, end_idx: str) -> Tuple[List[Tuple[int]], str]:
        
        path_list, _ptype, _edgret = dijkstra(self.graph, start_idx, end_idx)

        return self._smooth_point_list(self._convert_nx_path2list(path_list), _ptype,_edgret)
        

    def _convert_nx_path2list(self, path_list) -> List[Tuple[int]]:
        coord_list = []
        for i in path_list:
            data = self.node_dict[i]
            coord_list.append([data["x"], data["y"]])
        return coord_list

    def _smooth_point_list(self, coord_list, ptype,etype) -> List[Tuple[int]]:
        coordlist_new = []
        count = 0
        sizeincrease=0
        countfinal = len(coord_list)
        print(countfinal)
        ptype_new=ptype.copy()
        etype_new=etype.copy()

        while count < countfinal:
            if ptype[count] == "int":
                # append first point
                coordlist_new.append(coord_list[count])
                # find midpoint of intersection start and end
                xmidint = (coord_list[count][0] + coord_list[count + 2][0]) / 2
                ymidint = (coord_list[count][1] + coord_list[count + 2][1]) / 2

                xfinmid = (xmidint + coord_list[count + 1][0]) / 2
                yfinmid = (ymidint + coord_list[count + 1][1]) / 2

                pts=[coord_list[count],(xfinmid,yfinmid),coord_list[count+2]]

                x,y=zip(*pts)
                
                i = np.arange(len(x))

                # 5x the original number of points
                interp_i = np.linspace(0, i.max(), 5 * i.max())

                xi = interp1d(i, x, kind='quadratic')(interp_i)
                yi = interp1d(i, y, kind='quadratic')(interp_i)
                
                for i in range(len(xi)):
                    coordlist_new.append((xi[i],yi[i]))
                    ptype_new.insert(count+sizeincrease,"int")
                    etype_new.insert(count+sizeincrease,False)
                    sizeincrease+=1
                
                
                
                # coordlist_new.append((xfinmid,yfinmid))
                coordlist_new.append(coord_list[count+2])
                count+=3

                # coordlist_new.append((xfinmid, yfinmid))
                # coordlist_new.append(coord_list[count + 2])
                # count += 3
            else:
                coordlist_new.append(coord_list[count])
                count += 1

        return coordlist_new,ptype_new,etype_new


class Purest_Pursuit:
    def __init__(self, coord_list):
        self.k = 0.01  # look forward gain
        self.Lfc = 0.25  # [m] look-ahead distance
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
            distance_this_index = state.calc_distance(self.cx[ind], self.cy[ind])
            while True:
                try:
                    distance_next_index = state.calc_distance(
                        self.cx[ind + 1], self.cy[ind + 1]
                    )
                except IndexError:
                    distance_next_index = state.calc_distance(self.cx[-1], self.cy[-1])

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

    def purest_pursuit_steer_control(self, state, ind, Lf):
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
