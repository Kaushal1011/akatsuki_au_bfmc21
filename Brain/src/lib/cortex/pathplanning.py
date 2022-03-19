import heapq
import math
from typing import List, Tuple

from scipy.interpolate import interp1d
from src.lib.cortex import cubic_spline_planner
import networkx as nx
import numpy as np
from copy import deepcopy
from src.config import config

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
                print("pathplanning.py[46]",e)

            try:
                ptyperet[i + 1] = "int"
                i += 1
            except Exception as e:
                print("pathplanning.py[52]",e)
                

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

def give_perpendicular_park_pts(x,y,spot=1):
    # if spot==1:
    #     pt0x,pt0y=x+0.01,y+0.01
    #     pt1x,pt1y=x+0.5588,pt0y-0.5334
    #     pt2x,pt2y=pt1x+0.508,pt1y+0.3302
    #     pt3x,pt3y=pt2x-0.127,pt2y+0.66
    #     pt4x,pt4y=pt3x+0,pt3y+0.2032
    #     pt5x,pt5y=pt4x,pt4y+0.2032
    #     park_x_n,park_y_n=[pt0x,pt1x,pt2x,pt3x,pt4x,pt5x],[pt0x,pt1y,pt2y,pt3y,pt4y,pt5y]
    #     cx,cy,cyaw,rk,s=cubic_spline_planner.calc_spline_course(park_x_n,park_y_n,ds=0.15)
       
    #     return [i for i in zip(cx,cy)]
    #temporary:
    coord_list = [
        (2.1199999999999997, 2.1199999999999997),
        (2.2072555826316647, 1.9642686239127372),
        (2.2987501548820624, 1.8198978465696687),
        (2.398722706369926, 1.6982482667149883),
        (2.511412226713987, 1.6106804830928898),
        (2.6410577055329796, 1.5685550944475672),
        (2.7895393410766838, 1.581190302937868),
        (2.9415339509365492, 1.6430084931545883),
        (3.0740966676307324, 1.7418326938602926),
        (3.1642485373345357, 1.8654564196217684),
        (3.192618769936659, 2.002381602029436),
        (3.1697268696723078, 2.146978751551605),
        (3.1208636337525117, 2.2965185340365264),
        (3.171424620664199, 2.348292183875666),
        (3.1565564782777963, 2.499609817049711),
        (3.1594334941971126, 2.6494483048050387),
        (3.1507302101554803, 2.753755087166564),
    ]

    return coord_list

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

        self.node_dict = add_yaw(self.graph)

    def get_path(self, start_idx: str, end_idx: str) -> Tuple[List[Tuple[int]], str]:
        
        path_list, _ptype, _edgret = dijkstra(self.graph, start_idx, end_idx)

        return self._smooth_point_list(self._convert_nx_path2list(path_list), _ptype,_edgret)
        

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
    def __init__(self, coord_list:List[Tuple[float, float]], Lfc=0.125):
        self.k = 0.01  # look forward gain
        self.Lfc = Lfc  # [m] look-ahead distance
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
                except IndexError as e:
                    distance_next_index = state.calc_distance(self.cx[-1], self.cy[-1])
                    break

                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = self.k * abs(state.v) + self.Lfc  # update look ahead distance

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

        if state.v < 0:
            return delta

        return delta


    def reset_coord_list(self, coord_list, Lfc):
        self.cx, self.cy = zip(*coord_list)
        self.old_nearest_point_index = None
        self.Lfc = Lfc