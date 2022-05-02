# from multiprocessing import shared_memory
from multiprocessing.connection import Connection
from threading import Thread
import time
from typing import List
import numpy as np

# import SharedArray as sa
from loguru import logger
import zmq
import numpy as np
import base64
# from simple_pid import PID
from src.lib.perception.lanekeepfunctions import LaneKeep as LaneKeepMethod
from src.templates.workerprocess import WorkerProcess

MAX_STEER = 23


def get_last(inP: Connection):
    timestamp, data = inP.recv()
    while inP.poll():
        # print("lk: skipping frame")
        # logger.log("SYNC", f"Skipping Frame delta - {time() - timestamp}")
        timestamp, data = inP.recv()
    return timestamp, data


class LaneKeepingProcess(WorkerProcess):
    # ===================================== Worker process =========================================
    def __init__(self, inPs: List[Connection], outPs: List[Connection]):
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
                self.inPs,
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
        context = zmq.Context()

        sub_cam = context.socket(zmq.SUB)
        # print("Binding Socket to", self.addr)
        sub_cam.setsockopt(zmq.CONFLATE, 1)
        sub_cam.connect("ipc:///tmp/v4l")
        sub_cam.setsockopt_string(zmq.SUBSCRIBE, '')
        try:
            while True:
                # Obtain image
                image_recv_start = time.time()
                # stamps, img = inP.recv()
                data = sub_cam.recv()
                data = np.frombuffer(data, dtype=np.uint8)
                img = np.reshape(data, (480, 640, 3))
                print("lk img recv")
                logger.log("PIPE", "recv image")
                t_r += time.time() - image_recv_start
                count += 1

                logger.log(
                    "TIME",
                    f"Time taken to rec image {(t_r/count):.4f}s",
                )
                compute_time = time.time()
                # Apply image processing
                if len(outPs) > 1:
                    val, intersection_detected, outimage = self.lk(img, True)
                else:
                    val, intersection_detected = self.lk(img)
                angle = self.computeSteeringAnglePID(val)
                print(f"LK {angle}")
                # self.outPs[0].send((1, angle, intersection_detected))
                t += time.time() - compute_time
                logger.log(
                    "TIME",
                    f"Process Time -> {(t/count):.4f}s",
                )
                # print(f"LK compute time {(time() - compute_time):.4f}s")
                if len(outPs) > 1:
                    self.outPs[1].send((1, outimage))


                # print("Timetaken by LK: ", time() - a)
        except Exception as e:
            raise e