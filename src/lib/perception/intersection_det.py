# from multiprocessing import shared_memory
from multiprocessing.connection import Connection
from threading import Thread
from time import time
from typing import List

import numpy as np

from src.lib.perception.intersectiondethandle import intersection_det
from src.templates.workerprocess import WorkerProcess
from loguru import logger

# import SharedArray as sa
def get_last(inP: Connection):
    timestamp, data = inP.recv()
    while inP.poll():
        # print("lk: skipping frame")
        # logger.log("SYNC", f"Skipping Frame delta - {time() - timestamp}")
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

    def _the_thread(self, inP: Connection, outPs: List[Connection]):
        """Obtains image, applies the required image processing and computes the steering angle value.

        Parameters
        ----------
        inP  : Pipe
            Input pipe to read the frames from other process.
        outP : Pipe
            Output pipe to send the steering angle value to other process.
        """
        count = 0
        t = 0.0
        t_r = 0.1
        try:
            while True:
                # Obtain image
                image_recv_start = time()
                # stamps, img = inP.recv()
                stamp, img = get_last(inP)
                count += 1
                logger.log("PIPE", "recv image")
                count += 1
                t_r += time() - image_recv_start
                logger.log(
                    "TIME",
                    f"Time taken to rec image {(t_r/count):.4f}s",
                )
                # img = self.frame_shm
                # Apply image processing
                # print(f"iD: time taken to recv img {time() - img_rec_time}")
                compute_time = time()
                detected, outimage = intersection_det(img)
                t += time() - compute_time
                logger.log(
                    "TIME",
                    f"Process Time -> {(t/count):.4f}s",
                )
                # for outP in outPs:
                outPs[0].send((stamp, detected))
                if len(outPs) > 1:
                    outPs[1].send((stamp, outimage))
                    # print("Sending from Intersection Detection")
                # print("Time taken by ID:", time() - a)
        except Exception as e:
            print("Intersection Detection error:")
            print(e)
