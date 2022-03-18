import datetime
import math
from socket import timeout
from threading import Thread
from time import time
from typing import Optional

from src.config import config
from src.lib.cortex.pathplanning import (
    PathPlanning,
    Purest_Pursuit,
    give_perpendicular_park_pts,
)
from src.templates.workerprocess import WorkerProcess
from time import time

import math


class CarState:
    def __init__(self, max_v=0.28, dt=0.1, car_len=0.365) -> None:
        self.steering_angle = 0.0
        self.det_intersection = False
        # TODO: get initial position from config IDK
        self.x = 0.75
        self.max_v = max_v
        # 0.75, 4.8
        self.y = 4.8
        self.yaw = 0
        self.tl = {}
        self.v = max_v
        self.dt = dt
        self.car_len = car_len
        self.rear_x = self.x - ((car_len / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((car_len / 2) * math.sin(self.yaw))
        self.closest_pt = None
        self.last_update_time = time()
        self.stopped = False
        self.parking = False
        self.stop_slow_start_time = None

    def update_pos(self):
        dt = time() - self.last_update_time
        self.last_update_time = time()
        self.x = self.x + self.v * math.cos(self.yaw) * dt
        self.y = self.y + self.v * math.sin(self.yaw) * dt
        self.yaw = self.yaw + self.v / self.car_len * math.tan(self.steering_angle) * dt

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)

    def change_speed(self, new_speed: float, timeout: float, sign: str = "") -> float:
        # set when the car stopped
        if not self.stop_slow_start_time:
            self.stop_slow_start_time = time()
        # stop for wait secs
        if (time() - self.stop_slow_start_time) <= timeout:
            print(f"ENTER {sign}")
            self.v = new_speed
        # after wait secs start moving
        elif (time() - self.stop_slow_start_time) > timeout:
            print(f"RELEASE {sign}")
            self.stopped = True
            self.v = self.max_v

    def update(
        self,
        det_intersection: Optional[bool] = None,
        x: Optional[float] = None,
        y: Optional[float] = None,
        yaw: Optional[float] = None,
        tl: Optional[dict] = None,
        angle: Optional[float] = None,
    ) -> None:
        self.last_update_time = time()
        if angle:
            self.steering_angle = angle
        if det_intersection:
            self.det_intersection = det_intersection
        if x:
            self.x = x
            self.rear_x = self.x - ((self.car_len / 2) * math.cos(self.yaw))
        if y:
            self.y = y
            self.rear_y = self.y - ((self.car_len / 2) * math.sin(self.yaw))
        if yaw:
            self.yaw = yaw
        if tl:
            self.tl = tl

    def __repr__(self) -> str:
        return f"{datetime.datetime.now()}| {self.steering_angle:.4f}, {self.det_intersection}, {self.x:.4f}, {self.y:.4f}, {self.yaw:.4f}"

    def asdict(self) -> dict:
        return {"angle": self.steering_angle, "intersection": self.det_intersection}


plan = PathPlanning()
a = time()
if config["preplan"] == False:
    coord_list, p_type, etype = plan.get_path(config["start_idx"], config["end_idx"])

else:
    # preplanpath = joblib.load("../nbs/preplan.z")
    # coord_list = [i for i in zip(preplanpath["x"], preplanpath["y"])]
    # coord_list = coord_list[::20][:10]
    # p_type = preplanpath["ptype"]
    # p_type = p_type[::20][:10]
    # etype = preplanpath["etype"]
    # etype = etype[::20][:10]
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
        (3.071424620664199, 2.448292183875666),
        (3.0465564782777963, 2.599609817049711),
        (3.0494334941971126, 2.7494483048050387),
        (3.0507302101554803, 2.8993755087166564),
    ]
    p_type = ["int" for i in range(len(coord_list))]
    etype = [False for i in range(len(coord_list))]
    print("no. of points", len(coord_list))
    print("Using PREPLAN path")

if config["park"]:
    park_x = 2.119
    park_y = 2.09
    park_coord = give_perpendicular_park_pts(park_x, park_y)

pPC = Purest_Pursuit(coord_list)
# print("Time taken by Path Planning:", time() - a)


def controlsystem(vehicle: CarState, ind, Lf):
    di = pPC.purest_pursuit_steer_control(vehicle, ind, Lf)
    di = di * 180 / math.pi

    if di > 23:
        di = 23
    elif di < -23:
        di = -23

    return di


def check_reached(tx, ty, x, y):
    # d = math.sqrt((tx - x) ** 2 + (ty - y) ** 2)
    if math.sqrt((tx - x) ** 2 + (ty - y) ** 2) < 0.1:
        return True
    return False


class DecisionMakingProcess(WorkerProcess):
    # ===================================== Worker process =========================================
    def __init__(self, inPs, outPs, inPsnames=[]):
        """Process used for the image processing needed for lane keeping and for computing the steering value.

        Parameters
        ----------
        inPs : list(Pipe)
            List of input pipes (0 - receive image feed from the camera)
        outPs : list(Pipe)
            List of output pipes (0 - send steering data to the movvement control process)
        """
        super(DecisionMakingProcess, self).__init__(inPs, outPs)
        self.state = CarState()
        self.locsys_first = True
        self.inPsnames = inPsnames

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(DecisionMakingProcess, self).run()

    def _init_threads(self):
        """Initialize the thread."""
        if self._blocker.is_set():
            return

        thr = Thread(
            name="DecisionMakingThread",
            target=self._the_thread,
            args=(
                self.inPs,
                self.outPs,
            ),
        )
        thr.daemon = True
        self.threads.append(thr)

    def _the_thread(self, inPs, outPs):
        """Obtains image, applies the required image processing and computes the steering angle value.

        Parameters
        ----------
        inP  : Pipe
            Input pipe to read the frames from other process.
        outP : Pipe
            Output pipe to send the steering angle value to other process.
        """
        use_self_loc = False
        sign, sign_area = None, 0
        while True:
            try:
                c = time()

                t_lk = time()
                idx = self.inPsnames.index("lk")
                lk_angle, _ = inPs[idx].recv()
                # print(f"Time taken lk {(time() - t_lk):.3f}s {lk_angle}")

                t_id = time()
                idx = self.inPsnames.index("iD")
                detected_intersection = inPs[idx].recv()
                # print(f"Detected {detected_intersection}")

                x = self.state.x
                y = self.state.y
                yaw = self.state.yaw
                trafficlights = None
                imu_data = None
                # sign Detection
                t_sD = time()
                if "sD" in self.inPsnames:
                    idx = self.inPsnames.index("sD")
                    if inPs[idx].poll(timeout=0.1):
                        sign, sign_area = inPs[idx].recv()
                        if sign is None:
                            self.state.stopped = False
                            self.state.stop_slow_start_time = None
                        # print(f"Time taken sD {(time() - t_sD):.2f}s {label}")
                        print(f"{sign} {sign_area}")

                # locsys
                t_loc = time()
                if "loc" in self.inPsnames:
                    idx = self.inPsnames.index("loc")
                    if self.locsys_first:
                        if inPs[idx].poll(timeout=0.3):
                            loc = inPs[idx].recv()

                            x = loc["posA"]
                            y = loc["posB"]
                            if "rotA" in loc.keys():
                                yaw = 2 * math.pi - (loc["rotA"] + math.pi)
                            elif "radA" in loc.keys():
                                yaw = 2 * math.pi - (loc["radA"] + math.pi)

                            self.locsys_first = False
                        use_self_loc = False

                    if inPs[idx].poll(timeout=0.1):
                        loc = inPs[idx].recv()
                        use_self_loc = False
                        x = loc["posA"]
                        y = loc["posB"]
                        if "rotA" in loc.keys():
                            yaw = 2 * math.pi - (loc["rotA"] + math.pi)
                        elif "radA" in loc.keys():
                            yaw = 2 * math.pi - (loc["radA"] + math.pi)
                    else:
                        use_self_loc = True
                    # print(f"Time taken loc {(time() - t_loc):.2f}s {loc}")

                # TODO: add back
                # # if trafficlight process is connected
                # if len(inPs) > 3:
                #     trafficlights = inPs[3].recv()
                #     print(trafficlights)

                # imu
                if "imu" in self.inPsnames:
                    idx = self.inPsnames.index("imu")
                    if inPs[idx].poll(timeout=0.1):
                        imu_data = inPs[idx].recv()
                        yaw_imu = imu_data["yaw"]
                        # print("IMU:", imu_data)
                        # yaw_imu = 360 - imu_data["yaw"]
                        # print("imu_yaw", yaw_imu, "yaw", yaw)
                        # yaw_imu = yaw_imu if yaw_imu > 180 else -yaw_imu
                        yaw_imu = math.radians(yaw_imu)
                        if yaw_imu > math.pi:
                            yaw_imu -= 2 * math.pi
                        yaw_f = yaw_imu
                        yaw = yaw_f
                        # print("yaw", yaw_f)

                self.state.update(
                    det_intersection=detected_intersection,
                    x=x,
                    y=y,
                    yaw=yaw,
                    tl=trafficlights,
                )

                # states_l.append((x, y, yaw_f))
                # print(states_l)
                # print(self.state)
                # print("searched")
                # print(f"({x}, {y}) -> {ind}, {coord_list[ind]}")
                # if p_type[ind - 1] == "int":

                # ------- SWITCH PARKING STATE ---------------
                if config["park"] and check_reached(
                    park_x, park_y, self.state.x, self.state.y
                ):
                    pPC.reset_coord_list(park_coord)
                    self.state.v = 0.5 * self.state.max_v
                    self.state.parking = True

                # --------- GOAL STATE -------------------------

                if (
                    check_reached(pPC.cx[-1], pPC.cy[-1], self.state.x, self.state.y)
                ):
                    # and not self.state.parking
                    print("!!! Goal Reached !!!!")
                    self.state.v = 0.0
                    self.state.steering_angle = 0.0
                # --------- STOP STATE -------------------

                elif sign:
                    if sign == "stop" and sign_area > 2000:
                        self.state.change_speed(0, 5, sign)
                    if (sign == "crosswalk" or sign == "priority") and sign_area > 2000:
                        self.state.change_speed(self.state.max_v * 0.5, 5, sign)
                else:
                    # --------- CONTROL SYS -------------------
                    ind, Lf = pPC.search_target_index(self.state)
                    self.state.steering_angle = controlsystem(self.state, ind, Lf)
                    # --------- LANE KEEPING ------------------
                    # if p_type[ind - 1] == "lk":
                    #     if abs(self.state.steering_angle - lk_angle) < 30:
                    #         self.state.steering_angle = lk_angle
                    #     else:
                    #         print("lane keeping failed")

                # print(f"Current Behaviour : {p_type[ind-1]}")
                # if no locsys use self localization
                if len(inPs) < 3 or use_self_loc:
                # print("Using self localization")
                    self.state.update_pos()

                print(
                    f"({self.state.x:.3f}, {self.state.y:.3f}) {-self.state.steering_angle:.2f}, {self.state.v} {(time() - c):.2f}s"
                )
                for outP in outPs:
                    outP.send((-self.state.steering_angle, self.state.v))

            except Exception as e:
                print("Decision Process error:")
                raise e
