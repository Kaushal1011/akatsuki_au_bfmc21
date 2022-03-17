import datetime
import math
from socket import timeout
from threading import Thread
from time import time
from typing import Optional

from src.config import config
from src.lib.cortex.pathplanning import PathPlanning, Purest_Pursuit,MPC
from src.templates.workerprocess import WorkerProcess
from time import time
import joblib

import math

class CarState:
    def __init__(self, v=0.20, dt=0.1, car_len=0.365) -> None:
        self.steering_angle = 0.0
        self.det_intersection = False
        # TODO: get initial position from config IDK
        self.x = 0.75
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
        self.predelta=None

    def update_pos(self, steering_angle):
        dt = time() - self.last_update_time
        self.last_update_time = time()
        self.x = self.x + self.v * math.cos(self.yaw) * dt
        self.y = self.y + self.v * math.sin(self.yaw) * dt
        self.yaw = self.yaw + self.v / self.car_len * math.tan(steering_angle) * dt

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)

    def update(
        self,
        angle: Optional[float] = None,
        det_intersection: Optional[bool] = None,
        x: Optional[float] = None,
        y: Optional[float] = None,
        yaw: Optional[float] = None,
        v: Optional[float] = None,
        tl: Optional[dict] = None,
    ) -> float:
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
        if v:
            print("set v")
            self.v = v
        if tl:
            self.tl = tl
        return self.last_update_time

    def __repr__(self) -> str:
        return f"{datetime.datetime.now()}| {self.steering_angle:.4f}, {self.det_intersection}, {self.x:.4f}, {self.y:.4f}, {self.yaw:.4f}"

    def asdict(self) -> dict:
        return {"angle": self.steering_angle, "intersection": self.det_intersection}


plan = PathPlanning()
a = time()
if config["preplan"] == False:
    coord_list, p_type, etype = plan.get_path(config["start_idx"], config["end_idx"])

else:
    #preplanpath = joblib.load("../nbs/preplan.z")
    #coord_list = [i for i in zip(preplanpath["x"], preplanpath["y"])]
    #coord_list = coord_list[::20][:10]
    #p_type = preplanpath["ptype"]
    #p_type = p_type[::20][:10]
    #etype = preplanpath["etype"]
    #etype = etype[::20][:10]
    x = [0.5, 1, 1.5, 2, 2.5]
    y = [5.0]*5
    coord_list = list(zip(x,y))
    p_type=["int" for i in range(len(coord_list))]
    etype=[False for i in range(len(coord_list))]
    print("no. of points", len(coord_list))

pPC = Purest_Pursuit(coord_list)

if config["useMPC"]:

    MPCcon=MPC(coord_list)
    def mcontrolsystem(vehicle:CarState):
        di,ai,gr=MPCcon.MPC_steer_control(vehicle)

        di = di * 180 / math.pi

        if di > 21:
            di = 21
        elif di < -21:
            di = -21
        
        return di,ai,gr

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
            args=(self.inPs, self.outPs,),
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
        states_l = []
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
                # print(f"Time taken id {(time() - t_id):.3f}s  {detected_intersection}")

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
                        label, area = inPs[idx].recv()
                        print(f"Time taken sD {(time() - t_sD):.2f}s {label}")

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
                        #yaw_imu = yaw_imu if yaw_imu > 180 else -yaw_imu
                        yaw_imu = math.radians(yaw_imu)
                        if yaw_imu > math.pi:
                            yaw_imu -= 2*math.pi
                        yaw_f = yaw_imu
                        print("yaw", yaw_f)

                self.state.update(
                    lk_angle, detected_intersection, x, y, yaw, trafficlights
                )

                
                    
                # states_l.append((x, y, yaw_f))
                # print(states_l)
                # print(self.state)
                ind, Lf = pPC.search_target_index(self.state)
                # print("searched")
                print(f"({x}, {y}) -> {ind}, {coord_list[ind]}")
                # if p_type[ind - 1] == "int":
                angle = controlsystem(self.state, ind, Lf)
                print(f"Angle {angle}")
                # try:
                if config["useMPC"]:
                    
                    
                    di,ai,goal_reached=mcontrolsystem(self.state)
                    self.state.update(v=self.state.v + ai * MPCcon.DT)
                    # print("new_v",self.state.v + ai * MPCcon.DT)
                    # print("MPC output",di,ai)
                    if goal_reached:
                        print("Goal Reached")
                # except Exception as e:
                #     print(e)    
                # lane keeping
                #elif p_type[ind - 1] == "lk":
                #    angle = lk_angle
                #    angle2 = controlsystem(self.state, ind, Lf)
                #    if abs(angle2 - angle) > 21:
                #        print("lane keeping failed")
                #        angle = angle2
                #else:
                #    print("Here in nothingness")

                # print(f"Current Behaviour : {p_type[ind-1]}")

                # if no locsys use self localization
                if len(inPs) < 3 or use_self_loc:
                    pass
                    # print("Using self localization")
                    # self.state.update_pos(angle)

                for outP in outPs:
                    if config["useMPC"]:
                        outP.send((di,self.state.v ))
                    else:
                        outP.send((-angle,self.state.v))
    
                print("Sent to moment control in ",time() - c)
                
            except Exception as e:
                print("Decision Process error:")
                raise e
            # joblib.dump(states_l, 'dump.shakal')
                