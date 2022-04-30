from hashlib import sha3_384
import time

from src.lib.cortex.control_sys import Pure_Pursuit
import math
import joblib

from src.lib.cortex.carstate import CarState
import pathlib

from loguru import logger

# Extra because no path planning at start
####################################################################################
# data_path = pathlib.Path(  #
#     pathlib.Path(__file__).parent.parent.parent.resolve(), "data", "mid_course.z"  #
# )  #
# data = joblib.load(data_path)  #
# # ptype = data["ptype"]                                                            #
# # etype = data["etype"]                                                            #
# ptype = data[1]  #
# etype = data[2]  #
####################################################################################


class BehaviourCallback:
    def __init__(self, **kwargs):
        pass

    def out_condition(self, **kwargs) -> bool:
        return False

    def toggle_condition(self, **kwargs) -> bool:
        return False

    def __call__(self, car_state):
        raise NotImplementedError

    def reset(self, **kwargs):
        pass

    def set(self, **kwargs):
        raise NotImplementedError


class StopBehvaiour(BehaviourCallback):
    def __call__(self, car_state):
        # return 0 speed and 0 steering
        return {"speed": 0.0, "steer": 0.0}

    def set(self, **kwargs):
        pass


class PriorityBehaviour(BehaviourCallback):
    def __call__(self, car_state):
        # return pririty speed
        return {"speed": car_state.priority_speed}

    def set(self, **kwargs):
        pass


class HighwayBehaviour(BehaviourCallback):
    def __init__(self, **kwargs):
        self.exit_highway = False

    def __call__(self, car_state: CarState):
        if car_state.detected_sign["highway_exit"]:
            self.exit_highway = True

        return {"speed": car_state.highway_speed}

    def out_condition(self, **kwargs) -> bool:
        return self.exit_highway

    def set(self, **kwargs):
        self.exit_highway = False

    def reset(self, **kwargs):

        self.exit_highway = False
        return True


class OvertakeBehaviour(BehaviourCallback):
    def __init__(self, **kwargs):
        super().__init__()
        self.target_speed = None
        self.values = []
        self.target_deteremined = False
        self.speed = 0.0

        self.parallel_reach = False
        self.parallel_out = False
        self.prev_sensor_val = 0.0
        self.current_sensor_val = 0.0
        self.last_update_time = None
        self.target_speed_array = []
        self.WB = 0.3
        self.set_side=None

        # print("behaviour init")

    def reset(self, **kwargs):
        # set path here for changing lanes
        pass

    def __call__(self, car_state: CarState):
        # get target speed
        self.current_sensor_val = car_state.front_distance

        # print("Called overtake")

        if not self.target_deteremined:
            print("inside overtake speed calc")
            dt = time.time() - self.last_update_time
            self.last_update_time = time.time()
            tspeed = (
                (self.prev_sensor_val - self.current_sensor_val) / dt
            ) - car_state.v
            print("Target Speed: ", tspeed)
            self.target_speed_array.append(tspeed)
            if len(self.target_speed_array) > 3:
                self.target_speed = sum(self.target_speed_array[1:]) / len(
                    self.target_speed_array[1:]
                )
                print("Target Speed determined: ", self.target_speed)
                self.target_deteremined = True
                self.speed = car_state.v
            return {"None": None}
        # once target is determined
        # make new target points on the left lane
        cx = car_state.rear_x
        cy = car_state.rear_y

        # Use Of Inverted Yaw

        # print("cx,cy should be: ",car_state.navigator.get_nearest_node(cx,cy,car_state.yaw))
        nn = car_state.navigator.get_nearest_node(cx, cy, -car_state.yaw)

        # have to uncomment this no idea why this doesnt work uncommented
        cx,cy=nn["x"],nn["y"]

        tx = car_state.target_x
        ty = car_state.target_y
        
        try:
        # find m
            m = -(tx - cx) / (ty - cy)
        except:
            m = -(tx - cx) / 0.0001

        # print("m: ",m)
        # find c
        c = ty - m * tx
        # print("C: ",c)

        x3 = tx + ((0.365 ** 2) / (m ** 2 + 1)) ** 0.5
        x4 = tx - ((0.365 ** 2) / (m ** 2 + 1)) ** 0.5
        y3 = m * x3 + c
        y4 = m * x4 + c

        yaw3 = math.atan2(y3, x3)
        yaw4 = math.atan2(y4, x4)

        # print("x3 , y3",x3,y3)
        # print("x4, y4", x4,y4)

        # determine which point to pick

        # how to determine -> find a point on the road with -yaw
        # on = car_state.navigator.get_nearest_node(cx, cy, car_state.yaw)
        # ox, oy = on["x"], on["y"]


        # d3=math.sqrt((ox-x3)**2+(oy-y3)**2)
        # d4=math.sqrt((ox-x4)**2+(oy-y4)**2)

        # if d4 < d3 and self.set_side is None:
        #     print("Current Point:" ,car_state.x,car_state.y)
        #     print("Oncoming Point", ox,oy)
        #     self.set_side=4
        # elif self.set_side is None:
        #     self.set_side=3
        # else:
        #     pass

        self.set_side=4

        if self.set_side==4:
            alpha = math.atan2(y4 - car_state.rear_y, x4 - car_state.rear_x) - (
                    -car_state.yaw
                )
            delta = math.atan2(
                2.0
                * self.WB
                * math.sin(alpha)
                / math.sqrt((y4 - car_state.rear_y) ** 2 + (x4 - car_state.rear_x)**2),
                1.0,
            )
            print("In Overtake x4 y4 delta:", x4, y4, delta)
        else:
            alpha = math.atan2(y3 - car_state.rear_y, x3 - car_state.rear_x) - (
                -car_state.yaw
            )
            delta = math.atan2(
            2.0
            * self.WB
            * math.sin(alpha)
            / math.sqrt((y3 - car_state.rear_y) ** 2 + (x3 - car_state.rear_x)**2),
            1.0,
        )
            print("In Overtake x3 y3 delta:", x3, y3, delta)

        # print("rear pts: ",state.x,state.y)
        # print("target and yaw :", tx,ty,state.yaw)
        # print("alpha and pts angle", alpha,alpha+state.yaw)

        di = delta * 180 / math.pi
        if di > 23:
            di = 23
        elif di < -23:
            di = -23
        car_state.cs_steer = di

        if car_state.side_distance < 0.3:
            self.parallel_reach = True

        if car_state.side_distance > 0.9 and self.parallel_reach:
            self.parallel_out = True

        print("In Overtake ",self.set_side)


        return {"steer": di, "speed": car_state.v}

    def out_condition(self, **kwargs) -> bool:
        if self.parallel_out:
            return True
        # return super().out_condition(**kwargs)

    def set(self, **kwargs):
        # print("Setting up overtake")
        state: CarState
        state = kwargs["car_state"]
        self.prev_sensor_val = state.front_distance
        self.current_sensor_val = state.front_distance
        self.last_update_time = time.time()
        self.speed = state.v


class LaneKeepBehaviour(BehaviourCallback):
    def __call__(self, car_state):
        print("Lanekeeping angle: ", car_state.lanekeeping_angle)
        # return  {"steer":car_state.lanekeeping_angle}
        if abs(car_state.cs_steer - car_state.lanekeeping_angle) > 20:
            return None
        elif car_state.current_ptype == "lk":
            return {"steer": car_state.lanekeeping_angle}
            # return None

    def set(self, **kwargs):
        pass


class ControlSystemBehaviour(BehaviourCallback):
    def __init__(self, coord_list):
        super().__init__()
        self.cs = Pure_Pursuit(coord_list)

    def __call__(self, car_state: CarState):
        ind, lf = self.cs.search_target_index(car_state)
        logger.info(
            f"({car_state.x}, {car_state.y}) Target: {ind} ({self.cs.cx[ind]:.2f}, {self.cs.cy[ind]:.2f})"
        )

        car_state.target_x = self.cs.cx[ind]
        car_state.target_y = self.cs.cy[ind]
        car_state.current_target = (car_state.target_x, car_state.target_y)

        car_state.current_ptype = car_state.navigator.ptype[ind]
        car_state.can_overtake = car_state.navigator.etype[ind]
        car_state.activity_type = car_state.navigator.activity[ind]

        di = self.cs.purest_pursuit_steer_control(car_state, ind, lf)
        di = di * 180 / math.pi
        if di > 23:
            di = 23
        elif di < -23:
            di = -23
        car_state.cs_steer = di
        return {"steer": di, "speed": car_state.max_v}

    def set(self, **kwargs):
        pass

    def reset(self, **kwargs):
        self.cs.reset(kwargs["coord_list"])
        return True


class ObjectStopBehaviour(BehaviourCallback):
    def __call__(self, car_state):
        if car_state.front_distance < 0.3:
            return {"speed": 0.0}

    def set(self, **kwargs):
        pass


class ActionBehaviour:
    def __init__(self, name, release_time=0.0, callback=None):
        self.state = False
        self.state_start = None
        self.action_time = None
        self.release_time = release_time
        self.name = name
        self.callback = callback

    def reset(self, **kwargs):
        self.state = False
        self.state_start = None
        self.action_time = None
        self.callback.reset(**kwargs)
        # self.release_time = release_time
        # self.name = name

    def __call__(self, car_state=None):
        state, toggle = self.state_check()
        if state:
            if self.callback.out_condition():
                self.state = False
                return {"toggle": True}
            return self.callback(car_state)
        elif toggle:
            self.callback.toggle_condition()
            return {"toggle": True}
        else:
            return {}

    def state_check(self, **kwargs):
        if self.state == True:
            if self.action_time is not None:
                if (time.time() - self.state_start) > self.action_time:
                    print("in false state toggle")
                    self.state = False
                    return self.state, True
            return self.state, False
        else:
            return self.state, False

    def set(self, action_time=None, **kwargs):
        if not self.state_start or (
            self.state_start + self.action_time + self.release_time < time.time()
        ):
            self.action_time = action_time
            self.state_start = time.time()
            self.state = True
            self.callback.set(**kwargs)
            print("State set")
            return self.state
        else:
            return self.state

    def check_cooldown(self, **kwargs):

        if not self.state_start or (
            self.state_start + self.action_time + self.release_time < time.time()
        ):
            print("Check Cooldown Called: ", True)
            return True
        else:
            return False


class ActionManager:
    def __init__(self, **kwargs):
        self.lk = None
        self.cs = None
        self.objstop = None
        self.l1_ab = None
        self.l2_ab = None
        self.l3_ab = None
        self.l4_ab = None

    def __call__(self, carstate):

        obj = [
            self.cs,
            self.lk,
            self.l1_ab,
            self.l2_ab,
            self.l3_ab,
            self.l4_ab,
            self.objstop,
        ]
        speed = 0
        steer = 0
        count = 0

        # reset cs after interrupt using either self

        for i in obj:
            if i:
                logger.opt(colors=True).info(
                    f"Currently Active Behaviour <light-blue>{i.name}</light-blue>"
                )
                outputs = i(carstate)
                if outputs and "speed" in outputs.keys():
                    speed = outputs["speed"]
                if outputs and "steer" in outputs.keys():
                    steer = outputs["steer"]
                if outputs and "interrupt" in outputs.keys():
                    self.cs.reset(coord_list=outputs["interrupt"])
                if outputs and "toggle" in outputs.keys():
                    if count == 2:
                        self.l1_ab = None
                    elif count == 3:
                        print("killed l2")
                        self.l2_ab = None
                    elif count == 5:
                        self.l4_ab = None
            count += 1
        print("Output of system (Speed, Steer): ", speed, steer)
        return speed, steer

    def set_action(self, action, action_time=None, **kwargs):
        if action.name == "cs":
            self.cs = action
            self.cs.set(action_time=action_time, **kwargs)
            return True
        elif action.name == "lk":
            self.lk = action
            self.lk.set(action_time=action_time, **kwargs)
            return True
        elif action.name == "hw" or action.name == "roundabout":
            self.l1_ab = action
            self.l1_ab.set(action_time=action_time, **kwargs)
            return True
        elif (
            action.name == "parking"
            or action.name == "overtaking"
            or action.name == "tailing"
        ) and self.l2_ab is None:
            self.l2_ab = action
            self.l2_ab.set(action_time=action_time, **kwargs)
            return True
        elif (
            action.name == "stop"
            or action.name == "priority"
            or action.name == "crosswalk"
        ) and (
            self.l3_ab is None or self.l3_ab.check_cooldown()
        ):  # or action.name!=self.l3_ab.name):
            self.l3_ab = action
            self.l3_ab.set(action_time=action_time, **kwargs)
            return True
        elif action.name == "ramp" or action.name == "interrupt":
            self.l4_ab = action
            self.l4_ab.set(action_time=action_time, **kwargs)
            return True
        elif action.name == "objstop":
            self.objstop = action
            self.objstop.set(action_time=action_time, **kwargs)
            return True
        else:
            return False
