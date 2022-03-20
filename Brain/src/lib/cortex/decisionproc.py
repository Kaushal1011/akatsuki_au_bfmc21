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
import joblib
from src.lib.cortex.actions import Action
import math


stop_a = Action("stop", 4)
priority_a = Action("priority", 4)
crosswalk_a = Action("crosswalk", 4)

class CarState:
    def __init__(self, max_v=0.28, dt=0.1, car_len=0.365) -> None:
        self.steering_angle = 0.0
        self.det_intersection = False
        # TODO: get initial position from config IDK
        self.x = 1.6
        self.max_v = max_v
        # 0.75, 4.8
        self.y = 2.5
        self.yaw = 0
        self.tl = {}
        self.v = max_v
        self.dt = dt
        self.car_len = car_len
        self.rear_x = self.x - ((car_len / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((car_len / 2) * math.sin(self.yaw))
        self.closest_pt = None
        self.last_update_time = time()
        self.release = False
        self.last_release = None
        self.stopped = False
        self.slowed = False
        self.parking = False
        self.parked = False
        self.stop_slow_start_time = None
        self.should_park=True

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
            self.v = self.max_v
            self.slowed = False
            self.stopped = False
            if self.release == False:
                self.last_release = time()
                self.release = True

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
    pPC = Purest_Pursuit(coord_list)

else:
    # preplanpath = joblib.load("../nbs/preplan.z")
    # coord_list = [i for i in zip(preplanpath["x"], preplanpath["y"])]
    # coord_list = coord_list[::20][:10]
    # p_type = preplanpath["ptype"]
    # p_type = p_type[::20][:10]
    # etype = preplanpath["etype"]
    # etype = etype[::20][:10]
    path=joblib.load("./src/data/mid_course.z")
    coord_list=path[0]
    p_type=path[1]
    etype=path[2]
    # print(coord_list)
   
    print("no. of points", len(coord_list))
    print("Using PREPLAN path")
    pPC = Purest_Pursuit(coord_list, Lfc=0.2)

if config["park"]:
    park_x = 2.119
    park_y = 2.09
    park_coord = give_perpendicular_park_pts(park_x, park_y)

# print("Time taken by Path Planning:", time() - a)


def controlsystem(vehicle: CarState, ind, Lf):
    di = pPC.purest_pursuit_steer_control(vehicle, ind, Lf)
    di = di * 180 / math.pi

    if di > 23:
        di = 23
    elif di < -23:
        di = -23

    return di


def check_reached(tx, ty, x, y,parking=False):
    # d = math.sqrt((tx - x) ** 2 + (ty - y) ** 2)
    if math.sqrt((tx - x) ** 2 + (ty - y) ** 2) < 0.15:
        return True
    if math.sqrt((tx - x) ** 2 + (ty - y) ** 2) < 0.15 and parking:
        return True
    print("Goal dist:",math.sqrt((tx - x) ** 2 + (ty - y) ** 2))
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
        # last_angle, last_v = self.state.steering_angle, self.state.v
        # what is this hard coding initial loc
        self.state.x=None
        self.state.y=None
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
                        
                        # print(f"Time taken sD {(time() - t_sD):.2f}s {label}")
                        print(f"{sign} :  {sign_area}")

                # locsys
                t_loc = time()
                if "loc" in self.inPsnames:
                    idx = self.inPsnames.index("loc")
                    if self.locsys_first:
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
                        use_self_loc = False
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
                
                # if p_type[ind - 1] == "int":

                # ------- SWITCH PARKING STATE ---------------
                if config["park"] and check_reached(
                    park_x, park_y, self.state.x, self.state.y
                ):
                    print("Switched to Parking")
                    pPC.reset_coord_list(park_coord,0.5)
                    # self.state.max_v = 0.5 * self.state.max_v
                    self.state.v = self.state.max_v
                    self.state.parking = True

                # --------- SET STATE FOR SIGN -------------------
                if sign == "stop":
                    print("STOPPING",stop_a.set(5))
                elif sign == "priority":
                    print("PRIORITY", priority_a.set(5))
                elif sign == "crosswalk":
                    print("CROSSWALK", crosswalk_a.set(10))

                check_stop, _ = stop_a.state_check()
                check_priority, _ = priority_a.state_check()
                check_crosswalk, _ = crosswalk_a.state_check()

                if check_stop:
                    print("stop_astatecheck true")
                    self.state.v = 0.0
                elif check_priority:
                    print("priority_astatecheck true")
                    self.state.v = 0.5*self.state.max_v
                elif check_crosswalk:
                    print("crosswalk_astatecheck true")
                    self.state.v = 0
                else:
                    print("In Control Sys")
                    
                    self.state.v = self.state.max_v
                    # --------- CONTROL SYS -------------------
                    ind, Lf = pPC.search_target_index(self.state)
                    self.state.steering_angle = controlsystem(self.state, ind, Lf)
                    print(f"({self.state.x}, {self.state.y}) -> {ind}, {coord_list[ind]}")
                
                    # --------- LANE KEEPING ------------------
                    if p_type[ind - 1] == "lk":
                        if abs(self.state.steering_angle - lk_angle) < 23:
                            self.state.steering_angle = lk_angle
                        # else:
                        #     print("lane keeping failed")


                # --------- GOAL STATE -------------------------

                if check_reached(pPC.cx[-1], pPC.cy[-1], self.state.x, self.state.y,self.state.parked):
                    # and not self.state.parking
                    print("!!! Parked !!!!")
                    if self.state.parked == False and self.state.should_park==True:
                        self.state.parked = True
                        self.state.max_v = - self.state.max_v / 2
                        rev = [
                            (3.22, 2.60985613417526),
                            (3.195056598310364, 2.5057786365701986),
                            (3.163438268463328, 2.4081703605553306),
                            (3.118470082301495, 2.323500527720851),
                            (3.054092921646811, 2.257641527583271),
                            (2.9711398784535596, 1.869785942245324),
                            # (2.8747672371145274, 2.174936392629301),
                            # (2.77019798888053, 2.1280308485577373),
                        ]
                        # new_coords = [i for i in zip(pPC.cx[::-1], pPC.cy[::-1])]
                        pPC.reset_coord_list(rev, 0.5)
                        # print(new_coords[:(int(len(new_coords)/2))])
                    elif self.state.parked:
                        print("OUT OF PARKING")
                        self.state.v = 0.0
                        self.state.steering_angle = 0.0
                        #coord_list_test, p_type, etype = plan.get_path(config["start_idx"], config["end_idx"])
                        pPC.reset_coord_list(coord_list, 0.2)
                        self.state.max_v =  -self.state.max_v*2
                        self.state.v = self.state.max_v
                        self.state.should_park=False
                        self.state.parked=False
                    else:
                        print("!!! Goal Reached !!!!")
                        self.state.v = 0.0
                        self.state.steering_angle = 0.0
                
                # print(f"Current Behaviour : {p_type[ind-1]}")
                # if no locsys use self localization
                if len(inPs) < 3 or use_self_loc:
                    # print("Using self localization")
                    self.state.update_pos()
                
                
                try:
                    print(
                        f"({self.state.x:.3f}, {self.state.y:.3f}, {self.state.yaw:.2f}) {-self.state.steering_angle:.2f}, {self.state.v} {(time() - c):.2f}s"
                    )
                except:
                    print("index reset required")
                # if self.state.steering_angle != last_angle or self.state.v != last_v:
                for outP in outPs:
                    outP.send((-self.state.steering_angle, self.state.v))
                    # last_angle, last_v = self.state.steering_angle, self.state.v

            except Exception as e:
                print("Decision Process error:")
                raise e
