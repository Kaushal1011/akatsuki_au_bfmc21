from threading import Thread
from time import time


from src.lib.perception.lanekeepfunctions import LaneKeep as LaneKeepMethod
from src.lib.cortex.posfushandle import Localize
from src.templates.workerprocess import WorkerProcess
from multiprocessing import Pipe
from multiprocessing.connection import Connection
from typing import List


def get_last(inP: Pipe, delta_time: float = 0.1):
    timestamp, data = inP.recv()

    while (time() - timestamp) > delta_time:
        timestamp, data = inP.recv()

    return timestamp, data


class ObjectProcess(WorkerProcess):
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
        super(ObjectProcess, self).__init__(inPs, outPs)

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(ObjectProcess, self).run()

    def _init_threads(self):
        """Initialize the thread."""
        if self._blocker.is_set():
            return

        thr = Thread(
            name="ObjectThread",
            target=self._the_thread,
            args=(
                self.inPs,
                self.outPs,
            ),
        )
        thr.daemon = True
        self.threads.append(thr)

    # ===================================== Custom methods =========================================
    def _the_thread(self, inPs: List[Connection], outPs: List[Connection]):
        """Obtains image, applies the required image processing and computes the steering angle value.

        Parameters
        ----------
        inP  : Pipe
            Input pipe to read the frames from other process.
        outP : Pipe
            Output pipe to send the steering angle value to other process.
        """
        try:
            while True:
                stamp, image = inPs[0].recv()
                distance_data = inPs[1].recv()
                # print("Distance Data ====> ", distance_data)
                # front_distance, side_distance, det_car, det_ped, det_closed_road
                outPs[0].send((0.12, 0.12, False, False, False))

        except Exception as e:
            raise e
