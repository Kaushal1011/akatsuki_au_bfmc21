import heapq
import math
from typing import List, Tuple
import copy
import networkx as nx
import numpy as np
from scipy.optimize import minimize
import scipy.interpolate as scipy_interpolate

def interpolate_b_spline_path(x, y, n_path_points, degree=3):
    ipl_t = np.linspace(0.0, len(x) - 1, len(x))
    spl_i_x = scipy_interpolate.make_interp_spline(ipl_t, x, k=degree)
    spl_i_y = scipy_interpolate.make_interp_spline(ipl_t, y, k=degree)
    travel = np.linspace(0.0, len(x) - 1, n_path_points)
    return spl_i_x(travel), spl_i_y(travel)

def interpolate_path(path, sample_rate):
    choices = np.arange(0,len(path),sample_rate)
    if len(path)-1 not in choices:
            choices =  np.append(choices , len(path)-1)
    way_point_x = path[choices,0]
    way_point_y = path[choices,1]
    n_course_point = len(path)*3
    rix, riy = interpolate_b_spline_path(way_point_x, way_point_y, n_course_point)
    new_path = np.vstack([rix,riy]).T
    # new_path[new_path<0] = 0
    return new_path

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
        ptype.insert(
            0, ("lk" if len([i for i in G.neighbors(tg)]) < 2 else "int"))
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
        return (
            self._smooth_point_list(
                self._convert_nx_path2list(path_list), _ptype),
            _ptype,
        )

    def _convert_nx_path2list(self, path_list) -> List[Tuple[int]]:
        coord_list = []
        for i in path_list:
            data = self.node_dict[i]
            coord_list.append([data["x"], data["y"]])
        return coord_list

    def _smooth_point_list(self, coord_list, ptype) -> List[Tuple[int]]:
        coordlist_new = []
        count = 0
        countfinal = len(coord_list)
        print(countfinal)

        while count < countfinal:
            if ptype[count] == "int":
                # append first point
                coordlist_new.append(coord_list[count])
                # find midpoint of intersection start and end
                xmidint = (coord_list[count][0] + coord_list[count + 2][0]) / 2
                ymidint = (coord_list[count][1] + coord_list[count + 2][1]) / 2

                xfinmid = (xmidint + coord_list[count + 1][0]) / 2
                yfinmid = (ymidint + coord_list[count + 1][1]) / 2

                coordlist_new.append((xfinmid, yfinmid))
                coordlist_new.append(coord_list[count + 2])
                count += 3
            else:
                coordlist_new.append(coord_list[count])
                count += 1
        return coordlist_new


class Purest_Pursuit:
    def __init__(self, coord_list):
        self.k = 0.01  # look forward gain
        self.Lfc = 0.35  # [m] look-ahead distance
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
            distance_this_index = state.calc_distance(
                self.cx[ind], self.cy[ind])
            while True:
                try:
                    distance_next_index = state.calc_distance(
                        self.cx[ind + 1], self.cy[ind + 1]
                    )
                except IndexError:
                    print("index error")
                    distance_next_index = state.calc_distance(
                        self.cx[-1], self.cy[-1])

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


class MPC_Controller:
    def __init__(self):
        self.horiz = None
        self.horiz_limit = 5
        self.R = np.diag([0.01, 0.01])                 # input cost matrix
        # input difference cost matrix
        self.Rd = np.diag([0.01, 1.0])
        self.Q = np.diag([1.0, 1.0])                   # state cost matrix
        self.Qf = self.Q                               # state final matrix

    def mpc_cost(self, u_k, my_car, points):
        mpc_car = copy.copy(my_car)
        u_k = u_k.reshape(self.horiz, 2).T
        z_k = np.zeros((2, self.horiz+1))

        desired_state = points.T
        cost = 0.0

        for i in range(self.horiz):
            state_dot = mpc_car.move(u_k[0, i], u_k[1, i])
            mpc_car.update_state(state_dot)

            z_k[:, i] = [mpc_car.x, mpc_car.y]
            cost += np.sum(self.R@(u_k[:, i]**2))
            cost += np.sum(self.Q@((desired_state[:, i]-z_k[:, i])**2))
            if i < (self.horiz-1):
                cost += np.sum(self.Rd@((u_k[:, i+1] - u_k[:, i])**2))
        return cost

    def optimize(self, my_car, points):
        self.horiz = points.shape[0]
        bnd = [(-5, 5), (np.deg2rad(-60), np.deg2rad(60))]*self.horiz
        result = minimize(self.mpc_cost, args=(my_car, points), x0=np.zeros(
            (2*self.horiz)), method='SLSQP', bounds=bnd)
        return result.x[1]


class ParkPathPlanning:
    def __init__(self, obstacles):
        self.margin = 5
        # sacale obstacles from env margin to pathplanning margin
        obstacles = obstacles + np.array([self.margin, self.margin])
        obstacles = obstacles[(obstacles[:, 0] >= 0) & (obstacles[:, 1] >= 0)]

        self.obs = np.concatenate([np.array([[0, i] for i in range(100+self.margin)]),
                                  np.array([[100+2*self.margin, i]
                                           for i in range(100+2*self.margin)]),
                                  np.array([[i, 0]
                                           for i in range(100+self.margin)]),
                                  np.array([[i, 100+2*self.margin]
                                           for i in range(100+2*self.margin)]),
                                  obstacles])

        self.ox = [int(item) for item in self.obs[:, 0]]
        self.oy = [int(item) for item in self.obs[:, 1]]
        self.grid_size = 1
        self.robot_radius = 4
        self.a_star = AStarPlanner(
            self.ox, self.oy, self.grid_size, self.robot_radius)
            
    def plan_path(self,sx, sy, gx, gy):    
        rx, ry = self.a_star.planning(sx+self.margin, sy+self.margin, gx+self.margin, gy+self.margin)
        rx = np.array(rx)-self.margin+0.5
        ry = np.array(ry)-self.margin+0.5
        path = np.vstack([rx,ry]).T
        return path[::-1]

    def generate_park_scenario(self, sx, sy, gx, gy):
        rx, ry = self.a_star.planning(
            sx+self.margin, sy+self.margin, gx+self.margin, gy+self.margin)
        rx = np.array(rx)-self.margin+0.5
        ry = np.array(ry)-self.margin+0.5
        path = np.vstack([rx, ry]).T
        path = path[::-1]
        computed_angle = math.atan2(
            path[-10][0], path[-10][1], path[-1][0], path[-1][1])

        s = 4
        l = 8
        d = 2
        w = 4

        if -math.atan2(0, -1) < computed_angle <= math.atan2(-1, 0):
            x_ensure2 = gx
            y_ensure2 = gy
            x_ensure1 = x_ensure2 + d + w
            y_ensure1 = y_ensure2 - l - s
            ensure_path1 = np.vstack(
                [np.repeat(x_ensure1, 3/0.25), np.arange(y_ensure1-3, y_ensure1, 0.25)[::-1]]).T
            ensure_path2 = np.vstack(
                [np.repeat(x_ensure2, 3/0.25), np.arange(y_ensure2, y_ensure2+3, 0.25)[::-1]]).T
            park_path = self.plan_park_down_right(x_ensure2, y_ensure2)

        elif math.atan2(-1, 0) <= computed_angle <= math.atan2(0, 1):
            x_ensure2 = gx
            y_ensure2 = gy
            x_ensure1 = x_ensure2 - d - w
            y_ensure1 = y_ensure2 - l - s
            ensure_path1 = np.vstack(
                [np.repeat(x_ensure1, 3/0.25), np.arange(y_ensure1-3, y_ensure1, 0.25)[::-1]]).T
            ensure_path2 = np.vstack(
                [np.repeat(x_ensure2, 3/0.25), np.arange(y_ensure2, y_ensure2+3, 0.25)[::-1]]).T
            park_path = self.plan_park_down_left(x_ensure2, y_ensure2)

        elif math.atan2(0, 1) < computed_angle <= math.atan2(1, 0):
            x_ensure2 = gx
            y_ensure2 = gy
            x_ensure1 = x_ensure2 - d - w
            y_ensure1 = y_ensure2 + l + s
            ensure_path1 = np.vstack(
                [np.repeat(x_ensure1, 3/0.25), np.arange(y_ensure1, y_ensure1+3, 0.25)]).T
            ensure_path2 = np.vstack(
                [np.repeat(x_ensure2, 3/0.25), np.arange(y_ensure2-3, y_ensure2, 0.25)]).T
            park_path = self.plan_park_up_left(x_ensure2, y_ensure2)

        elif math.atan2(1, 0) < computed_angle <= math.atan2(0, -1):
            x_ensure2 = gx
            y_ensure2 = gy
            x_ensure1 = x_ensure2 + d + w
            y_ensure1 = y_ensure2 + l + s
            ensure_path1 = np.vstack(
                [np.repeat(x_ensure1, 3/0.25), np.arange(y_ensure1, y_ensure1+3, 0.25)]).T
            ensure_path2 = np.vstack(
                [np.repeat(x_ensure2, 3/0.25), np.arange(y_ensure2-3, y_ensure2, 0.25)]).T
            park_path = self.plan_park_up_right(x_ensure2, y_ensure2)

        return np.array([x_ensure1, y_ensure1]), park_path, ensure_path1, ensure_path2

    def plan_park_up_right(self, x1, y1):
        s = 4
        l = 8
        d = 2
        w = 4

        x0 = x1 + d + w
        y0 = y1 + l + s

        curve_x = np.array([])
        curve_y = np.array([])
        y = np.arange(y1, y0+1)
        circle_fun = (6.9**2 - (y-y0)**2)
        x = (np.sqrt(circle_fun[circle_fun >= 0]) + x0-6.9)
        y = y[circle_fun >= 0]
        choices = x > x0-6.9/2
        x = x[choices]
        y = y[choices]
        curve_x = np.append(curve_x, x[::-1])
        curve_y = np.append(curve_y, y[::-1])

        y = np.arange(y1, y0+1)
        circle_fun = (6.9**2 - (y-y1)**2)
        x = (np.sqrt(circle_fun[circle_fun >= 0]) + x1+6.9)
        y = y[circle_fun >= 0]
        x = (x - 2*(x-(x1+6.9)))
        choices = x < x1+6.9/2
        x = x[choices]
        y = y[choices]
        curve_x = np.append(curve_x, x[::-1])
        curve_y = np.append(curve_y, y[::-1])

        park_path = np.vstack([curve_x, curve_y]).T
        return park_path

    def plan_park_up_left(self, x1, y1):
        s = 4
        l = 8
        d = 2
        w = 4

        x0 = x1 - d - w
        y0 = y1 + l + s

        curve_x = np.array([])
        curve_y = np.array([])
        y = np.arange(y1, y0+1)
        circle_fun = (6.9**2 - (y-y0)**2)
        x = (np.sqrt(circle_fun[circle_fun >= 0]) + x0+6.9)
        y = y[circle_fun >= 0]
        x = (x - 2*(x-(x0+6.9)))
        choices = x < x0+6.9/2
        x = x[choices]
        y = y[choices]
        curve_x = np.append(curve_x, x[::-1])
        curve_y = np.append(curve_y, y[::-1])

        y = np.arange(y1, y0+1)
        circle_fun = (6.9**2 - (y-y1)**2)
        x = (np.sqrt(circle_fun[circle_fun >= 0]) + x1-6.9)
        y = y[circle_fun >= 0]
        choices = x > x1-6.9/2
        x = x[choices]
        y = y[choices]
        curve_x = np.append(curve_x, x[::-1])
        curve_y = np.append(curve_y, y[::-1])

        park_path = np.vstack([curve_x, curve_y]).T
        return park_path

    def plan_park_down_right(self, x1, y1):
        s = 4
        l = 8
        d = 2
        w = 4

        x0 = x1 + d + w
        y0 = y1 - l - s

        curve_x = np.array([])
        curve_y = np.array([])
        y = np.arange(y0, y1+1)
        circle_fun = (6.9**2 - (y-y0)**2)
        x = (np.sqrt(circle_fun[circle_fun >= 0]) + x0-6.9)
        y = y[circle_fun >= 0]
        choices = x > x0-6.9/2
        x = x[choices]
        y = y[choices]

        curve_x = np.append(curve_x, x)
        curve_y = np.append(curve_y, y)

        y = np.arange(y0, y1+1)
        circle_fun = (6.9**2 - (y-y1)**2)
        x = (np.sqrt(circle_fun[circle_fun >= 0]) + x1+6.9)
        x = (x - 2*(x-(x1+6.9)))
        y = y[circle_fun >= 0]
        choices = x < x1+6.9/2
        x = x[choices]
        y = y[choices]
        curve_x = np.append(curve_x, x)
        curve_y = np.append(curve_y, y)

        park_path = np.vstack([curve_x, curve_y]).T
        return park_path

    def plan_park_down_left(self, x1, y1):
        s = 4
        l = 8
        d = 2
        w = 4

        x0 = x1 - d - w
        y0 = y1 - l - s

        curve_x = np.array([])
        curve_y = np.array([])
        y = np.arange(y0, y1+1)
        circle_fun = (6.9**2 - (y-y0)**2)
        x = (np.sqrt(circle_fun[circle_fun >= 0]) + x0+6.9)
        y = y[circle_fun >= 0]
        x = (x - 2*(x-(x0+6.9)))
        choices = x < x0+6.9/2
        x = x[choices]
        y = y[choices]
        curve_x = np.append(curve_x, x)
        curve_y = np.append(curve_y, y)

        y = np.arange(y0, y1+1)
        circle_fun = (6.9**2 - (y-y1)**2)
        x = (np.sqrt(circle_fun[circle_fun >= 0]) + x1-6.9)
        y = y[circle_fun >= 0]
        choices = x > x1-6.9/2
        x = x[choices]
        y = y[choices]
        curve_x = np.append(curve_x, x)
        curve_y = np.append(curve_y, y)

        park_path = np.vstack([curve_x, curve_y]).T
        return park_path

class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search
        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position
        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d < self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
