from re import S, X
from turtle import st
import numpy as np
import math

from src.lib.cortex.carstate import CarState
class Pure_Pursuit:
    def __init__(self, coord_list,Lfc=0.3):
        self.k = 0.01  # look forward gain
        self.Lfc = Lfc  # [m] look-ahead distance
        self.Kp = 1.0  # speed proportional gain
        self.WB = 0.3  # [m] wheel base of vehicle
        self.cx, self.cy = zip(*coord_list)
        self.old_nearest_point_index = None

    def search_target_index(self, state:CarState,flag=None):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None and flag=="roundabout":
            # search nearest point index
            print("inhere")
            # state.rear_x=state.x
            # state.rear_y=state.y
            state.f_x = state.x - (
            (state.car_len / 2) * math.cos(state.yaw)
        )
            state.f_y = state.y - (
            (state.car_len / 2) * math.sin(state.yaw)
        )

            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        elif flag=="roundabout":
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
    
        if self.old_nearest_point_index is None and flag is None:
            ind=state.navigator.get_current_node(state.rear_x,state.rear_y,-state.yaw)
            self.old_nearest_point_index=ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind], self.cy[ind])
            if distance_this_index>3:
                ind=state.navigator.get_current_node(state.rear_x,state.rear_y,-state.yaw)
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

        # Use Of Inverted Yaw Here

        alpha = math.atan2(ty - state.y, tx - state.x) - (-state.yaw)

        # print("rear pts: ",state.x,state.y)
        # print("target and yaw :", tx,ty,state.yaw)
        # print("alpha and pts angle", alpha,alpha+state.yaw)

        delta = math.atan2(2.0 * self.WB * math.sin(alpha) / Lf, 1.0)

        # print("computed delta:", delta)

        return delta

    def reset_coord_list(self, coord_list, Lfc=0.125):
        self.cx, self.cy = zip(*coord_list)
        self.old_nearest_point_index = None
        self.Lfc = Lfc