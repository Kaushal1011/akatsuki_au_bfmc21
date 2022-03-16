import heapq
import math
from typing import List, Tuple

from scipy.interpolate import interp1d

import networkx as nx
import numpy as np
from copy import deepcopy
import cvxpy
from util import pi_2_pi,get_nparray_from_matrix,smooth_yaw,calc_speed_profile
import time

class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None



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
    def __init__(self, coord_list):
        self.k = 0.01  # look forward gain
        self.Lfc = 0.125  # [m] look-ahead distance
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


class MPC:

    def __init__(self,coord_list,initial_state) -> None:
        
        self.cx, self.cy = zip(*coord_list)
        self.old_nearest_point_index = None

        self.NX = 4  # x = x, y, v, yaw
        self.NU = 2  # a = [accel, steer]
        self.T = 5  # horizon length

        # mpc parameters
        self.R = np.diag([0.01, 0.01])  # input cost matrix
        self.Rd = np.diag([0.01, 1.0])  # input difference cost matrix
        self.Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
        self.Qf = self.Q  # state final matrix
        self.GOAL_DIS = .15  # goal distance
        self.STOP_SPEED = 0.5 / 3.6  # stop speed
        # self.MAX_TIME = 500.0  # max simulation time
        # iterative paramter
        self.MAX_ITER = 3  # Max iteration
        self.DU_TH = 0.1  # iteration finish param

        self.TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed
        self.N_IND_SEARCH = 10  # Search index number

        self.DT = 0.2  # [s] time tick

        self.dtcal=time.time()

        # Vehicle parameters
        self.LENGTH = 0.365  # [m]
        self.WIDTH = 0.19  # [m]
        self.BACKTOWHEEL = 0.1  # [m]
        self.WHEEL_LEN = 0.03  # [m]
        self.WHEEL_WIDTH = 0.02  # [m]
        self.TREAD = 0.07  # [m]
        self.WB = 0.26  # [m]

        self.MAX_STEER = math.radians(21.0)  # maximum steering angle [rad]
        self.MAX_DSTEER = math.radians(30.0)  # maximum steering speed [rad/s]
        self.MAX_SPEED = 0.3  # maximum speed [m/s]
        self.MIN_SPEED = -0.3  # minimum speed [m/s]
        self.MAX_ACCEL = 0.01  # maximum accel [m/ss]

        goal = [self.cx[-1], self.cy[-1]]

        self.state = initial_state

        # initial yaw compensation
        if self.state.yaw - self.cyaw[0] >= math.pi:
            self.state.yaw -= math.pi * 2.0
        elif self.state.yaw - self.cyaw[0] <= -math.pi:
            self.state.yaw += math.pi * 2.0

        self.target_ind, _ = self.calc_nearest_index(self.state, self.cx, self.cy, cyaw, 0)

        self.odelta, self.oa = None, None

        self.cyaw = smooth_yaw(self.cyaw)

    def get_linear_model_matrix(self,v, phi, delta):

        A = np.matrix(np.zeros((self.NX, self.NX)))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.DT * math.cos(phi)
        A[0, 3] = - self.DT * v * math.sin(phi)
        A[1, 2] = self.DT * math.sin(phi)
        A[1, 3] = self.DT * v * math.cos(phi)
        A[3, 2] = self.DT * math.tan(delta) / self.WB

        B = np.matrix(np.zeros((self.NX, self.NU)))
        B[2, 0] = self.DT
        B[3, 1] = self.DT * v / (self.WB * math.cos(delta) ** 2)

        C = np.zeros(self.NX)
        C[0] = self.DT * v * math.sin(phi) * phi
        C[1] = - self.DT * v * math.cos(phi) * phi
        C[3] = v * delta / (self.WB * math.cos(delta) ** 2)

        return A, B, C    
    
    def update_state(self,state, a, delta):

        # input check
        if delta >= self.MAX_STEER:
            delta = self.MAX_STEER
        elif delta <= -self.MAX_STEER:
            delta = -self.MAX_STEER

        state.x = state.x + state.v * math.cos(state.yaw) * self.DT
        state.y = state.y + state.v * math.sin(state.yaw) * self.DT
        state.yaw = state.yaw + state.v / self.WB * math.tan(delta) * self.DT
        state.v = state.v + a * self.DT

        if state. v > self.MAX_SPEED:
            state.v = self.MAX_SPEED
        elif state. v < self.MIN_SPEED:
            state.v = self.MIN_SPEED

        return state
    
    def update_state_new(self,state, a, delta):
        self.DT=time.time()-self.dtcal
        self.dtcal=time.time()
        # input check
        if delta >= self.MAX_STEER:
            delta = self.MAX_STEER
        elif delta <= -self.MAX_STEER:
            delta = -self.MAX_STEER

        self.state.x = state.x 
        self.state.y = state.y 
        self.state.yaw = state.yaw 
        self.state.v = state.v + a * self.DT

        if self.state. v > self.MAX_SPEED:
            self.state.v = self.MAX_SPEED
        elif self.state. v < self.MIN_SPEED:
            self.state.v = self.MIN_SPEED

        return state




    def calc_nearest_index(self,state, cx, cy, cyaw, pind):

        dx = [state.x - icx for icx in cx[pind:(pind + self.N_IND_SEARCH)]]
        dy = [state.y - icy for icy in cy[pind:(pind + self.N_IND_SEARCH)]]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind) + pind

        mind = math.sqrt(mind)

        dxl = cx[ind] - state.x
        dyl = cy[ind] - state.y

        angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind


    def predict_motion(self,x0, oa, od, xref):
        xbar = xref * 0.0
        for i in range(len(x0)):
            xbar[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for (ai, di, i) in zip(oa, od, range(1, self.T + 1)):
            state = self.update_state(state, ai, di)
            xbar[0, i] = state.x
            xbar[1, i] = state.y
            xbar[2, i] = state.v
            xbar[3, i] = state.yaw

        return xbar


    def iterative_linear_mpc_control(self,xref, x0, dref, oa, od):
        """
        MPC contorl with updating operational point iteraitvely
        """

        if oa is None or od is None:
            oa = [0.0] * self.T
            od = [0.0] * self.T

        for i in range(self.MAX_ITER):
            xbar = self.predict_motion(x0, oa, od, xref)
            poa, pod = oa[:], od[:]
            oa, od, ox, oy, oyaw, ov = self.linear_mpc_control(xref, xbar, x0, dref)
            du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
            if du <= self.DU_TH:
                break
        else:
            print("Iterative is max iter")

        return oa, od, ox, oy, oyaw, ov


    def linear_mpc_control(self,xref, xbar, x0, dref):
        """
        linear mpc control

        xref: reference point
        xbar: operational point
        x0: initial state
        dref: reference steer angle
        """

        x = cvxpy.Variable((self.NX, self.T + 1))
        u = cvxpy.Variable((self.NU, self.T))

        cost = 0.0
        constraints = []

        for t in range(self.T):
            cost += cvxpy.quad_form(u[:, t], self.R)

            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], self.Q)

            A, B, C = self.get_linear_model_matrix(
                xbar[2, t], xbar[3, t], dref[0, t])
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

            if t < (self.T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], self.Rd)
                constraints += [cvxpy.abs(u[1, t + 1] - u[1, t])
                                <= self.MAX_DSTEER * self.DT]

        cost += cvxpy.quad_form(xref[:, self.T] - x[:, self.T], self.Qf)

        constraints += [x[:, 0] == x0]
        constraints += [x[2, :] <= self.MAX_SPEED]
        constraints += [x[2, :] >= self.MIN_SPEED]
        constraints += [cvxpy.abs(u[0, :]) <= self.MAX_ACCEL]
        constraints += [cvxpy.abs(u[1, :]) <= self.MAX_STEER]

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.ECOS, verbose=False)

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ox = get_nparray_from_matrix(x.value[0, :])
            oy = get_nparray_from_matrix(x.value[1, :])
            ov = get_nparray_from_matrix(x.value[2, :])
            oyaw = get_nparray_from_matrix(x.value[3, :])
            oa = get_nparray_from_matrix(u.value[0, :])
            odelta = get_nparray_from_matrix(u.value[1, :])

        else:
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov


    def calc_ref_trajectory(self,state, cx, cy, cyaw, ck, sp, dl, pind):
        xref = np.zeros((self.NX, self.T + 1))
        dref = np.zeros((1, self.T + 1))
        ncourse = len(cx)

        ind, _ = self.calc_nearest_index(state, cx, cy, cyaw, pind)

        if pind >= ind:
            ind = pind

        xref[0, 0] = cx[ind]
        xref[1, 0] = cy[ind]
        xref[2, 0] = sp[ind]
        xref[3, 0] = cyaw[ind]
        dref[0, 0] = 0.0  # steer operational point should be 0

        travel = 0.0

        for i in range(self.T + 1):
            travel += abs(state.v) * self.DT
            dind = int(round(travel / dl))

            if (ind + dind) < ncourse:
                xref[0, i] = cx[ind + dind]
                xref[1, i] = cy[ind + dind]
                xref[2, i] = sp[ind + dind]
                xref[3, i] = cyaw[ind + dind]
                dref[0, i] = 0.0
            else:
                xref[0, i] = cx[ncourse - 1]
                xref[1, i] = cy[ncourse - 1]
                xref[2, i] = sp[ncourse - 1]
                xref[3, i] = cyaw[ncourse - 1]
                dref[0, i] = 0.0

        return xref, ind, dref


    def check_goal(self,state, goal, tind, nind):

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        d = math.sqrt(dx ** 2 + dy ** 2)

        if (d <= self.GOAL_DIS):
            isgoal = True
        else:
            isgoal = False

        if abs(tind - nind) >= 5:
            isgoal = False

        if (abs(state.v) <= self.STOP_SPEED):
            isstop = True
        else:
            isstop = False

        if isgoal and isstop:
            return True

        return False

    def MPC_steer_control(self, state, ind):


        xref, target_ind, dref = self.calc_ref_trajectory(
            state, self.cx, self.cy, self.cyaw, self.ck, self.sp, self.dl, ind)

        x0 = [state.x, state.y, state.v, state.yaw]  # current state

        oa, odelta, ox, oy, oyaw, ov = self.iterative_linear_mpc_control(
            xref, x0, dref, oa, odelta)

        if odelta is not None:
            di, ai = odelta[0], oa[0]

        #update state outside of this
        # state = self.update_state(state, ai, di)

        if self.check_goal(state, self.goal, target_ind, len(self.cx)):
            print("Goal")
            return None
        
        return di,ai

