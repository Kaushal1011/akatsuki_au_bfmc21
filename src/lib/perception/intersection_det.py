# from multiprocessing import shared_memory
from multiprocessing.connection import Connection
from threading import Thread
from time import time
from typing import List

import numpy as np

from src.lib.perception.intersectiondethandle import intersection_det
from src.templates.workerprocess import WorkerProcess
from multiprocessing import Pipe

# import SharedArray as sa
def get_last(inP: Pipe, delta_time: float = 0.1):
    timestamp, data = inP.recv()

    while (time() - timestamp) > delta_time:
        # print("iD: skipping frame")
        timestamp, data = inP.recv()
    return timestamp, data


class IntersectionDetProcess(WorkerProcess):
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
        super(IntersectionDetProcess, self).__init__(inPs, outPs)
        # self.frame_shm = sa.attach("shm://shared_frame1")

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(IntersectionDetProcess, self).run()

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

    def _the_thread(self, inP:Connection, outPs:List[Connection]):
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
                img_rec_time = time()
                # stamps, img = inP.recv()
                stamps, img = get_last(inP, 0.01)
                # img = self.frame_shm
                # Apply image processing
                # print(f"iD: time taken to recv img {time() - img_rec_time}")
                detected, outimage = intersection_det(img)
                # for outP in outPs:
                outPs[0].send(detected)
                if len(outPs) > 1:
                    outPs[1].send((1, outimage))
                    # print("Sending from Intersection Detection")
                # print("Time taken by ID:", time() - a)
        except Exception as e:
            print("Intersection Detection error:")
            print(e)
