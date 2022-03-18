import datetime
import math
from socket import timeout
from threading import Thread
from time import time
from typing import Optional

from src.config import config
from src.lib.cortex.pathplanning import PathPlanning, Purest_Pursuit
from src.templates.workerprocess import WorkerProcess
from time import time

import math


class CarState:
    def __init__(self, v=0.18, dt=0.1, car_len=0.365) -> None:
        self.steering_angle = 0.0
        self.det_intersection = False
        # TODO: get initial position from config IDK
        self.x = 0.75
        self.max_v = v
        # 0.75, 4.8
        self.y = 4.8
        self.yaw = 0
        self.tl = {}
        self.v = v
        self.dt = dt
        self.car_len = car_len
        self.rear_x = self.x - ((car_len / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((car_len / 2) * math.sin(self.yaw))
        self.closest_pt = None
        self.last_update_time = time()
        self.stopped = False
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

    def stop(self, label: str = "stop") -> float:
        print("here in stop")
        # set when the car stopped
        if not self.stop_slow_start_time:
            self.stop_slow_start_time = time()
        # stop for wait secs
        if (time() - self.stop_slow_start_time) <= 4:
            print("Stop -> Stopping")
            if label == "stop":
                self.v = 0
            else:
                self.v = 0.5 * self.max_v
        # after wait secs start moving
        elif (time() - self.stop_slow_start_time) > 4:
            print("Stop -> Release")
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
    x = [0.5, 1, 1.5, 2, 2.5]
    y = [5.0] * 5
    coord_list = list(zip(x, y))
    p_type = ["int" for i in range(len(coord_list))]
    etype = [False for i in range(len(coord_list))]
    print("no. of points", len(coord_list))

pPC = Purest_Pursuit(coord_list)
# print("Time taken by Path Planning:", time() - a)


def controlsystem(vehicle: CarState, ind, Lf):
    di = pPC.purest_pursuit_steer_control(vehicle, ind, Lf)
    di = di * 180 / math.pi

    if di > 21:
        di = 21
    elif di < -21:
        di = -21

    return di



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
                print(f"Detected {detected_intersection}")

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

                # --------- GOAL STATE -------------------------

                # d = abs(coord_list[-1][0] - x) + abs((coord_list[-1][1] - y))
                # print(f"Goal distance {d}")
                # if d < 0.3:
                #     print("Goal Reached")
                #     self.state.v = 0.0
                #     self.state.steering_angle = 0.0
                # # else:
                # check_goal(coord_list[-1][0],coord_list[-1][1], self.state.x, self.state.y)
                # --------- STOP STATE -------------------
                if sign and sign_area > 2000:
                    self.state.stop()
                else:
                    # --------- CONTROL SYS -------------------
                    ind, Lf = pPC.search_target_index(self.state)
                    self.state.steering_angle = controlsystem(self.state, ind, Lf)
                    # --------- LANE KEEPING ------------------
                    if p_type[ind - 1] == "lk":
                        if abs(self.state.steering_angle - lk_angle) < 30:
                            self.state.steering_angle = lk_angle
                        else:
                            print("lane keeping failed")

                # print(f"Current Behaviour : {p_type[ind-1]}")
                # if no locsys use self localization
                if len(inPs) < 3 or use_self_loc:
                    print("Using self localization")
                    self.state.update_pos()

                print(
                    f"{-self.state.steering_angle}, {self.state.v} {(time() - c):.2f}s"
                )
                for outP in outPs:
                    outP.send((-self.state.steering_angle, self.state.v))

            except Exception as e:
                print("Decision Process error:")
                raise e
