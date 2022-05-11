import heapq
import math
from typing import List, Optional, Tuple

# from scipy.interpolate import interp1d
# from src.lib.cortex import cubic_spline_planner
import networkx as nx
import numpy as np
from copy import deepcopy
from src.config import get_config
from src.lib.cortex import cubic_spline_planner

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

    roundabout_pts = [
        "267",
        "268",
        "269",
        "270",
        "271",
        "302",
        "303",
        "304",
        "305",
        "306",
        "307",
    ]

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
            except:
                pass
            try:
                ptyperet[i + 1] = "int"
                i += 1
            except:
                pass

    for i in range(len(ptyperet)):
        if fp[i] in roundabout_pts:
            ptyperet[i] = "roundabout"

    edgeret = []

    for i in range(len(fp) - 1):
        dt = G.get_edge_data(fp[i], fp[i + 1])
        edgeret.append(dt["dotted"])

    edgeret.append(None)

    return fp, ptyperet, edgeret


def isLeft(A, B, isleftpoint):
    yd = math.atan2(B[1] - A[1], B[0] - A[0])
    yd2 = math.atan2(isleftpoint[1] - A[1], isleftpoint[0] - A[0])
    compute = yd - yd2

    if abs(compute) > math.pi:
        if compute > 0:
            compute = abs(compute) - 2 * math.pi
        else:
            compute = 2 * math.pi - abs(compute)
    # print(compute)
    if compute >= 0:
        # print("True")
        return True, compute
    else:
        # print("False")
        return False, compute


# TODO: smooth path
def _smooth_point_list(G, coord_list1, ptype, etype):
    node_dict = deepcopy(dict(G.nodes(data=True)))
    coord_list = []
    for i in coord_list1:
        data = node_dict[i]
        coord_list.append([data["x"], data["y"]])
    coordlist_new = []
    count = 0
    sizeincrease = 0
    countfinal = len(coord_list)
    # print(countfinal)
    ptype_new = ptype.copy()
    etype_new = etype.copy()

    while count < countfinal:
        if ptype[count] == "int":
            # append first point
            coordlist_new.append(coord_list[count])
            # find midpoint of intersection start and end
            xmidint = (coord_list[count][0] + coord_list[count + 2][0]) / 2
            ymidint = (coord_list[count][1] + coord_list[count + 2][1]) / 2

            flag, value = isLeft(
                coord_list[count], coord_list[count + 1], coord_list[count + 2]
            )

            if flag and abs(value) > 0.3:
                xfinmid = (xmidint + coord_list[count + 1][0] * 5) / 6
                yfinmid = (ymidint + coord_list[count + 1][1] * 5) / 6
                pts = [coord_list[count], (xfinmid, yfinmid), coord_list[count + 2]]
            elif not flag and abs(value) > 0.3:
                xfinmid = (xmidint * 2 + coord_list[count + 1][0]) / 3
                yfinmid = (ymidint * 2 + coord_list[count + 1][1]) / 3
                pts = [coord_list[count], (xfinmid, yfinmid), coord_list[count + 2]]

            else:
                pts = [coord_list[count], coord_list[count + 2]]

            x, y = zip(*pts)

            i = np.arange(len(x))

            cx, cy, _, _, _ = cubic_spline_planner.calc_spline_course(x, y, 0.15)

            for i in range(len(cx)):
                coordlist_new.append((cx[i], cy[i]))
                ptype_new.insert(count + sizeincrease, "int")
                etype_new.insert(count + sizeincrease, False)
                sizeincrease += 1

            coordlist_new.append(coord_list[count + 2])
            coordlist_new.append(coord_list[count + 2])
            count += 3
        elif (
            ptype[count] == "lk"
            and count < countfinal - 2
            and ptype[count + 1] == "lk"
            and isSmoothNeeded(
                coord_list[count], coord_list[count + 1], coord_list[count + 2]
            )
        ):
            pts = [coord_list[count], coord_list[count + 1], coord_list[count + 2]]
            x, y = zip(*pts)

            i = np.arange(len(x))

            cx, cy, _, _, _ = cubic_spline_planner.calc_spline_course(x, y, 0.15)

            for i in range(len(cx)):
                coordlist_new.append((cx[i], cy[i]))
                ptype_new.insert(count + sizeincrease, "lk")
                etype_new.insert(count + sizeincrease, False)
                sizeincrease += 1
            coordlist_new.append(coord_list[count + 2])
            coordlist_new.append(coord_list[count + 2])
            coordlist_new.append(coord_list[count + 2])
            count += 3
        else:
            coordlist_new.append(coord_list[count])
            count += 1

    return coordlist_new, ptype_new, etype_new


def _smooth_point_list_o(G, coord_list1, ptype, etype):
    node_dict = deepcopy(dict(G.nodes(data=True)))
    coord_list = []
    for i in coord_list1:
        data = node_dict[i]
        coord_list.append([data["x"], data["y"]])
    coordlist_new = []
    count = 0
    sizeincrease = 0
    countfinal = len(coord_list)
    # print(countfinal)
    ptype_new = ptype.copy()
    etype_new = etype.copy()

    while count < countfinal:
        if ptype[count] == "int":
            # append first point
            coordlist_new.append(coord_list[count])
            # find midpoint of intersection start and end
            xmidint = (coord_list[count][0] + coord_list[count + 2][0]) / 2
            ymidint = (coord_list[count][1] + coord_list[count + 2][1]) / 2

            flag, value = isLeft(
                coord_list[count], coord_list[count + 1], coord_list[count + 2]
            )

            if flag and abs(value) > 0.3:
                xfinmid = (xmidint + coord_list[count + 1][0] * 5) / 6
                yfinmid = (ymidint + coord_list[count + 1][1] * 5) / 6
                pts = [coord_list[count], (xfinmid, yfinmid), coord_list[count + 2]]
            elif not flag and abs(value) > 0.3:
                xfinmid = (xmidint * 2 + coord_list[count + 1][0]) / 3
                yfinmid = (ymidint * 2 + coord_list[count + 1][1]) / 3
                pts = [coord_list[count], (xfinmid, yfinmid), coord_list[count + 2]]

            else:
                pts = [coord_list[count], coord_list[count + 2]]

            x, y = zip(*pts)

            i = np.arange(len(x))

            cx, cy, _, _, _ = cubic_spline_planner.calc_spline_course(x, y, 0.15)

            for i in range(len(cx)):
                coordlist_new.append((cx[i], cy[i]))
                ptype_new.insert(count + sizeincrease, "int")
                etype_new.insert(count + sizeincrease, False)
                sizeincrease += 1

            coordlist_new.append(coord_list[count + 2])
            coordlist_new.append(coord_list[count + 2])
            count += 3
        elif ptype[count] == "lk" or ptype[count] == "roundabout":
            icount = count
            pts = []
            while True:
                if ptype[count] != "lk" and ptype[count] != "roundabout":
                    "broke"
                    break
                pts.append(coord_list[count])
                count += 1
                if count >= countfinal:
                    break
            # print(pts)

            if len(pts) == 1:
                coordlist_new.append(coord_list[icount])
                # ptype_new.insert(icount,"lk")
                # etype_new.insert(icount,False)
                continue

            x, y = zip(*pts)

            i = np.arange(len(x))

            cx, cy, _, _, _ = cubic_spline_planner.calc_spline_course(x, y, 0.15)

            for i in range(len(cx)):
                coordlist_new.append((cx[i], cy[i]))

            windowsize = len(cx) // (count - icount)

            incrementor = 0
            for i in range(len(cx) - (count - icount)):
                # add incrementor increment logic
                if sizeincrease > windowsize * (incrementor + 1) and incrementor != (
                    count - icount
                ):
                    incrementor += 1
                ptype_new.insert(icount + sizeincrease, "lk")
                if len(etype) < icount + incrementor - 1:
                    etype_new.insert(icount + sizeincrease, etype[icount + incrementor])
                else:
                    etype_new.insert(
                        icount + sizeincrease, etype[icount + incrementor - 1]
                    )
                sizeincrease += 1

        else:
            coordlist_new.append(coord_list[count])
            count += 1

    return coordlist_new, ptype_new, etype_new


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
    def __init__(self, config: dict, test=False):
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
        self.coords = []
        self.ptype = []
        self.etype = []
        self.activity = []
        self.yaw = []
        self.area_dict_list = []
        self.config = config
        self.create_path()

    def create_path(self):
        for i in range(len(self.config["nodes"])):
            # print(config["nodes"][i],config["activity"][i])
            c, p, e = _smooth_point_list_o(
                self.graph,
                *dijkstra(
                    self.graph,
                    str(self.config["nodes"][i][0]),
                    str(self.config["nodes"][i][1]),
                )
            )

            a = [self.config["activity"][i] for j in c]
            assert len(c) == len(e) == len(p) == len(a)
            pdict = {"c": c, "p": p, "e": e, "a": a}
            self.coords.extend(c)
            self.ptype.extend(p)
            self.etype.extend(e)
            self.activity.extend(a)
            self.area_dict_list.append(pdict)

        self.add_path_yaw()

    def add_path_yaw(self):
        for i in range(len(self.coords)):
            # print(i)
            try:
                y = self.coords[i + 1][1] - self.coords[i][1]
                x = self.coords[i + 1][0] - self.coords[i][0]
                self.yaw.append(math.atan2(y, x))
            except Exception as e:
                print("in exception", e)
                self.yaw.append(0)

    def plan_course(self, cofig_json):
        raise NotImplementedError

    def replan_course(self, completed_list, config_json):
        raise NotImplementedError

    def get_course_ahead(self, x, y, yaw):
        raise NotImplementedError

    def get_current_node(self, x, y, yaw):
        idx = self.get_nearest_node_incoords(x, y, yaw)
        return idx
        # return (
        #     self.coords[idx],
        #     self.ptype[idx],
        #     self.etype[idx],
        # )

    def get_nearest_node_incoords(self, x, y, yaw):

        dx = []
        dy = []

        for i in range(len(self.coords)):
            dx.append(self.coords[i][0] - x)
            dy.append(self.coords[i][1] - y)

        d = np.hypot(dx, dy)
        # print(dx,dy)
        idxs = np.argsort(d)
        # print(idxs)
        # for i in idxs:
        #     print(d[i])

        d = np.hypot(dx, dy)
        idxs = np.argsort(d)
        min_dyaw_idx = 0
        for idx in idxs[:10]:
            try:
                dyaw = self.yaw[idx] - yaw
                print(self.yaw[idx], yaw)

                if abs(dyaw) < 0.2:
                    return idx

            except KeyError as e:
                print(e)
                continue

        return min_dyaw_idx

    def get_nearest_node(self, x, y, yaw):
        dx = []
        dy = []
        # print(x,y)
        for node in self.node_dict:
            dx.append(self.node_dict[node]["x"] - x)
            dy.append(self.node_dict[node]["y"] - y)

        d = np.hypot(dx, dy)
        # print(dx, dy)
        idxs = np.argsort(d)
        # print(idxs)
        # for i in idxs:
        #     print(d[i])
        # min_dyaw = float("inf")
        min_dyaw_idx = -1
        for idx in idxs:
            try:
                dyaw: np.ndarray = abs(np.array(self.node_dict[str(idx)]["yaw"]) - yaw)
                # dyaw = dyaw.min()

                # if abs(dyaw) > 3.141592:
                #     if dyaw>0:
                #         dyaw=abs(dyaw)-2*math.pi
                #     else:
                #         dyaw=2*math.pi-abs(dyaw)

                # if dyaw > min_dyaw:
                #     min_dyaw = dyaw
                #     min_dyaw_idx = idx

                if (abs(dyaw) < 0.2).any():
                    return idx

            except KeyError as e:
                print(e)
                continue
        return min_dyaw_idx

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