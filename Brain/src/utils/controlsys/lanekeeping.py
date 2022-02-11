import socket
import struct
import time
import numpy as np
import datetime
import cv2
import math

from multiprocessing import Process
from threading import Thread
# from simple_pid import PID
from src.lib.lanekeeputils import LaneKeep as LaneKeepMethod
from src.templates.workerprocess import WorkerProcess


class LaneKeepingProcess(WorkerProcess):
    # pid = PID(Kp=1.0, Ki=1.45, Kd=0.15)

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
        self.lk = LaneKeepMethod(use_perspective=True, computation_method="hough")

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
    # def laneKeeping(self, img:np.ndarray):
    #     """Applies required image processing.

    #     Parameters
    #     ----------
    #     img : Pipe
    #         The image on which to apply the algorithm.
    #     """
    #     return self.lk(img)

    def computeSteeringAnglePID(self, val):
        # keep the angle between max steer angle
        val = max(-17, min(val - 90, 17))
        # # Apply pid
        # newVal = self.pid(val)

        # # Calibrate result
        # newVal = val / 2.9

        # newVal = -newVal

        # newVal += 0

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
        while True:
            try:
                # Obtain image
                stamps, img = inP.recv()
                # Apply image processing
                val, outimage = self.lk(img)
                print(f"Computed angle :{val}")
                angle = self.computeSteeringAnglePID(val)
                # Compute steering angle
                # val = self.computeSteeringAngle(val)

                # Print steering angle value
                # print(f"Steer angle is: {val}")
                # Send steering angle value
                for outP in outPs:
                    outP.send((angle, outimage))

            except Exception as e:
                print("Lane keeping error:")
                print(e)
