# from multiprocessing import shared_memory
from threading import Thread
from time import time

import numpy as np

# import SharedArray as sa

# from simple_pid import PID
from src.lib.perception.lanekeepfunctions import LaneKeep as LaneKeepMethod
from src.templates.workerprocess import WorkerProcess
from multiprocessing import Pipe

MAX_STEER = 23


def get_last(inP: Pipe, delta_time: float = 0.1):
    timestamp, data = inP.recv()
    while (time() - timestamp) > delta_time:
        timestamp, data = inP.recv()
    
    return timestamp, data


class LaneKeepingProcess(WorkerProcess):
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
        super(LaneKeepingProcess, self).__init__(inPs, outPs)
        self.lk = LaneKeepMethod(use_perspective=False, computation_method="hough")
        # self.frame_shm = sa.attach("shm://shared_frame1")

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(LaneKeepingProcess, self).run()

    def _init_threads(self):
        """Initialize the thread."""
        if self._blocker.is_set():
            return

        thr = Thread(
            name="StreamSending",
            target=self._the_thread,
            args=(
                self.inPs[0],
                self.outPs,
            ),
        )
        thr.daemon = True
        self.threads.append(thr)

    # ===================================== Custom methods =========================================
    def computeSteeringAnglePID(self, val):
        # keep the angle between max steer angle
        val = max(-MAX_STEER, min(val - 90, MAX_STEER))
        return val

    def _the_thread(self, inP, outPs):
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
                # Obtain image
                image_recv_start = time()
                stamps, img = get_last(inP)
                # img = self.frame_shm
                print(f"lk: Time taken to recv image {time() - image_recv_start}")
                # print("Time taken to recieve image", time()- i)
                compute_time = time()
                # Apply image processing
                val, outimage = self.lk(img)
                angle = self.computeSteeringAnglePID(val)

                self.outPs[0].send((angle, None))
                print(f"LK compute time {(time() - compute_time):.4f}s")
                if len(outPs) > 1:
                    print(outimage.shape)
                    self.outPs[1].send((angle, outimage))

                    # print("Sending from Lane Keeping")

                # print("Timetaken by LK: ", time() - a)
        except Exception as e:
            raise e
