import numpy as np

from threading import Thread
from src.templates.workerprocess import WorkerProcess
import datetime


class CarState:
    def __init__(self, v=0, dt=0.1, l=0.365) -> None:
        self.steering_angle = 0.0
        self.det_intersection = False
        self.loc = {}
        self.tl = {}

    def update(self, angle: float, det_intersection: bool, loc: dict, tl: dict) -> None:
        self.steering_angle = angle
        self.det_intersection = det_intersection
        self.loc = loc
        self.tl = tl

    def __repr__(self) -> str:
        return f"{datetime.datetime.now()}| {self.steering_angle}, {self.det_intersection}, {self.loc}, {self.tl}"

    def asdict(self) -> dict:
        return {"angle": self.steering_angle, "intersection": self.det_intersection}


class DataFusionProcess(WorkerProcess):
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
        super(DataFusionProcess, self).__init__(inPs, outPs)
        self.state = CarState()

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(DataFusionProcess, self).run()

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
                tl = inPs[3].recv()

                self.state.update(angle, detected_intersection, loc, tl)
                print(self.state)

                for outP in outPs:
                    outP.send(self.state.asdict())

            except Exception as e:
                print("Intersection Detection error:")
                print(e)
