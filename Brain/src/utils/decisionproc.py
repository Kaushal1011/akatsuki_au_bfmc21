import numpy as np

from threading import Thread
from src.templates.workerprocess import WorkerProcess
import datetime
import math
from typing import *
from src.utils.pathplanning import PathPlanning, Purest_Pursuit

START_IDX = "86"
END_IDX = "27"


class CarState:
    def __init__(self, v=0, dt=0.1, l=0.365) -> None:
        self.steering_angle = 0.0
        self.det_intersection = False
        self.x = 0  
        self.y = 0
        self.yaw = 0
        self.tl = {}
        self.v = v
        self.dt = dt
        self.l = l

    def update_pos(self, steering_angle):
        self.x = self.x + self.v * math.cos(self.yaw) * self.dt
        self.y = self.y + self.v * math.sin(self.yaw) * self.dt
        self.yaw = self.yaw + self.v / self.l * math.tan(steering_angle) * self.dt

    def update(
        self,
        angle: float,
        det_intersection: bool,
        x: float,
        y: float,
        yaw: float,
        tl: dict,
    ) -> None:
        self.steering_angle = angle
        self.det_intersection = det_intersection
        self.x = x
        self.y = y
        self.yaw = yaw
        self.tl = tl

    def __repr__(self) -> str:
        return f"{datetime.datetime.now()}| {self.steering_angle}, {self.det_intersection}, {self.x}, {self.y}, {self.yaw}"

    def asdict(self) -> dict:
        return {"angle": self.steering_angle, "intersection": self.det_intersection}


plan = PathPlanning()
coord_list = plan.get_path(START_IDX, END_IDX)
pPC = Purest_Pursuit(coord_list)

def controlsystem(vehicle: CarState):

    di = pPC.purest_pursuit_steer_control(vehicle)
    di = di*180/math.pi
    
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
                angle, _ = inPs[0].recv()
                detected_intersection = inPs[1].recv()
                loc = inPs[2].recv()
                x = loc["posA"]
                y = loc["posB"]
                yaw = loc["radA"] + math.pi/2
                tl = inPs[3].recv()
                # will send output from behaviours
                self.state.update(angle, detected_intersection, x, y, yaw, tl)
                print(self.state)
                angle = controlsystem(self.state)
                for outP in outPs:
                    outP.send((-angle, None))

            except Exception as e:
                print("Decision Process error:")
                print(e)
