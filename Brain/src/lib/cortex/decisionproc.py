import datetime
import math
from threading import Thread
from typing import Optional

from src.config import config
from src.lib.cortex.pathplanning import PathPlanning, Purest_Pursuit
from src.templates.workerprocess import WorkerProcess
from time import time

class CarState:
    def __init__(self, v=0.12, dt=0.1, car_len=0.365) -> None:
        self.steering_angle = 0.0
        self.det_intersection = False
        #TODO: get initial position from config IDK
        self.x = 0.83
        self.y = 14.67
        self.yaw = 0
        self.tl = {}
        self.v = v
        self.dt = dt
        self.car_len = car_len
        self.rear_x = self.x - ((car_len / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((car_len / 2) * math.sin(self.yaw))
        self.closest_pt = None
        self.last_update_time = time()

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
        tl: Optional[dict] = None,
    ) -> None:
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
        return f"{datetime.datetime.now()}| {self.steering_angle}, {self.det_intersection}, {self.x}, {self.y}, {self.yaw}"

    def asdict(self) -> dict:
        return {"angle": self.steering_angle, "intersection": self.det_intersection}


plan = PathPlanning()


def plan_path():
    global p_type
    global coord_list
    global pPC
    coord_list, p_type = plan.get_path(config["start_idx"], config["end_idx"])
    pPC = Purest_Pursuit(coord_list)


plan_path()


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
    def __init__(self, inPs, outPs):
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

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(DecisionMakingProcess, self).run()

    def _init_threads(self):
        """Initialize the thread."""
        if self._blocker.is_set():
            return

        thr = Thread(
            name="StreamSending",
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
        while True:
            try:
                lk_angle, _ = inPs[0].recv()
                detected_intersection = inPs[1].recv()

                x = None
                y = None
                yaw = None
                trafficlights = None
                imu_data = None
                # if locsys process is connected
                if len(inPs) > 2:
                    loc = inPs[2].recv()

                    x = loc["posA"]
                    y = loc["posB"]
                    yaw = 2 * math.pi - (loc["rotA"] + math.pi)
                    
                # if trafficlight process is connected
                if len(inPs) > 3:
                    trafficlights = inPs[3].recv()
                    print(trafficlights)
                # if imu process is connected
                if len(inPs) > 4:
                    imu_data = inPs[4].recv()
                    print(imu_data)

                self.state.update(
                    lk_angle, detected_intersection, x, y, yaw, trafficlights
                )
                print(self.state)
                ind, Lf = pPC.search_target_index(self.state)
                print(ind, coord_list[ind])
                # intersection navigation
                if p_type[ind - 1] == "int":
                    angle = controlsystem(self.state, ind, Lf)
                # lane keeping
                elif p_type[ind - 1] == "lk":
                    angle = lk_angle
                # stop reached
                elif all((x, y) == coord_list[ind]):
                    print("Its time to stop")
                else:
                    print("Here in nothingness")

                print(f"Current Behaviour : {p_type[ind-1]}")
                
                # if no locsys use self localization
                if len(inPs) < 3:
                    print("Using self localization")
                    self.state.update_pos(angle)

                for outP in outPs:
                    outP.send((angle, None))

            except Exception as e:
                print("Decision Process error:")
                raise e
