import numpy as np

from threading import Thread
from src.templates.workerprocess import WorkerProcess
import datetime
import math
from copy import deepcopy
from typing import *
from src.utils.pathplanning import PathPlanning

START_IDX = "13"
END_IDX = "111"


class CarState:
    def __init__(self, v=14, dt=0.1, l=0.365) -> None:
        self.steering_angle = 0.0
        self.det_intersection = False
        self.x = 0.5  # TODO
        self.y = 11.25  # TODO
        self.yaw = 0
        self.tl = {}
        self.v = v
        self.dt = dt
        self.l = l

    def update_pos(self, steering_angle):
        self.x = self.x + self.v * math.cos(self.yaw) * self.dt
        self.y = self.y + self.v * math.sin(self.yaw) * self.dt
        self.yaw = (
            self.yaw + self.v / self.l * math.tan(steering_angle) * self.dt
        )  # steering_angle is the steering angle

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
        return f"{datetime.datetime.now()}| {self.steering_angle}, {self.det_intersection}, {self.loc}, {self.tl}"

    def asdict(self) -> dict:
        return {"angle": self.steering_angle, "intersection": self.det_intersection}


class PurePursuit:
    def __init__(self, kld=0.05, ld=0.5):
        self.kld = kld  # look ahead gain
        self.ld = ld  # look ahead distance

    def targetIndex(self, vehicle, cx, cy):
        dx = [vehicle.x - x for x in cx]
        dy = [vehicle.y - y for y in cy]
        dist = [math.sqrt(diffx ** 2 + diffy ** 2) for (diffx, diffy) in zip(dx, dy)]
        index = dist.index(min(dist))
        length = 0.0
        newld = self.kld * vehicle.v + self.ld
        while newld > length and (index + 1) < len(cx):
            diffx = cx[index + 1] - cx[index]
            diffy = cy[index + 1] - cy[index]
            length += math.sqrt(diffx ** 2 + diffy ** 2)
            index += 1

        return index

    def purePursuitControl(self, vehicle, cx, cy, cind):

        index = self.targetIndex(vehicle, cx, cy)

        if cind >= index:
            index = cind
        if index < len(cx):
            tx = cx[index]
            ty = cy[index]
        else:
            tx = cx[-1]
            ty = cy[-1]
            index = len(cx) - 1

        alpha = (
            math.atan2(ty - vehicle.y, tx - vehicle.x) - vehicle.yaw
        )  # calculate the alpah

        if vehicle.v < 0:
            alpha = math.pi - alpha  # check if the vehilce is backwarding

        newld = self.kld * vehicle.v + self.ld

        delta = math.atan2(2.0 * vehicle.l * math.sin(alpha), newld)

        if delta > 25 * math.pi / 180:
            delta = 25 * math.pi / 180
        elif delta < -25 * math.pi / 180:
            delta = -25 * math.pi / 180

        return delta, index


def path_smooth(path, weight_data=0.5, weight_smooth=0.6, tolerance=0.000001):
    """
    Creates a smooth path for a n-dimensional series of coordinates.
    Arguments:
        path: List containing coordinates of a path
        weight_data: Float, how much weight to update the data (alpha)
        weight_smooth: Float, how much weight to smooth the coordinates (beta).
        tolerance: Float, how much change per iteration is necessary to keep iterating.
    Output:
        new: List containing smoothed coordinates.
    """

    new = deepcopy(path)
    dims = len(path[0])
    change = tolerance

    while change >= tolerance:
        change = 0.0
        for i in range(1, len(new) - 1):
            for j in range(dims):

                x_i = path[i][j]
                y_i, y_prev, y_next = new[i][j], new[i - 1][j], new[i + 1][j]

                y_i_saved = y_i
                y_i += weight_data * (x_i - y_i) + weight_smooth * (
                    y_next + y_prev - (2 * y_i)
                )
                new[i][j] = y_i

                change += abs(y_i - y_i_saved)

    return new


purePursuitController = PurePursuit()
plan = PathPlanning()
coord_list = plan.get_path(START_IDX, END_IDX)
path2 = path_smooth(coord_list)
cx = [item[0] for item in path2]
cy = [item[1] for item in path2]


def controlsystem(vehicle: CarState):

    target_ind = purePursuitController.targetIndex(vehicle, cx, cy)
    di, target_ind = purePursuitController.purePursuitControl(
        vehicle, cx, cy, target_ind
    )
    vehicle.update_pos(di)

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
                x = loc["PosA"]
                y = loc["PosB"]
                yaw = loc["RotA"]
                tl = inPs[3].recv()
                # will send output from behaviours
                self.state.update(angle, detected_intersection, x, y, yaw, tl)
                print(self.state)
                angle = controlsystem(self.state)
                for outP in outPs:
                    outP.send((angle, None))

            except Exception as e:
                print("Intersection Detection error:")
                print(e)
