from threading import Thread
import time
from typing import List

from src.templates.workerprocess import WorkerProcess
from multiprocessing.connection import Connection
from src.lib.perception.detect_ov import Detection
from loguru import logger
import zmq
import numpy as np
class SignDetectionProcess(WorkerProcess):
    # ===================================== Worker process =========================================
    def __init__(
        self, inPs: Connection, outPs: Connection, outPnames:List[str]
    ):
        """Process used for the image processing needed for lane keeping and for computing the steering value.

        Parameters
        ----------
        inPs : list(Pipe)
            List of input pipes (0 - receive image feed from the camera)
        outPs : list(Pipe)
            List of output pipes (0 - send steering data to the movvement control process)
        """
        super(SignDetectionProcess, self).__init__(inPs, outPs)
        self.outPnames: List[str] = outPnames

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(SignDetectionProcess, self).run()

    def _init_threads(self):
        """Initialize the thread."""
        if self._blocker.is_set():
            return

        thr = Thread(
            name="SignDetectionThread",
            target=self._the_thread,
            args=(
                self.inPs,
                self.outPs,
            ),
        )
        thr.daemon = True
        self.threads.append(thr)

    def _the_thread(self, inP: Connection, outPs: List[Connection]) -> None:
        """Obtains image, applies the required image processing and computes the steering angle value.

        Parameters
        ----------
        inP  : Pipe
            Input pipe to read the frames from other process.
        outP : Pipe
            Output pipe to send the steering angle value to other process.
        """
        count = 0
        self.detection = Detection()
        context = zmq.Context()

        sub_cam = context.socket(zmq.SUB)
        sub_cam.setsockopt(zmq.CONFLATE, 1)
        sub_cam.connect("ipc:///tmp/v4l")
        sub_cam.setsockopt_string(zmq.SUBSCRIBE, '')
        print(">>> Starting Sign Detection")
        while True:
            try:
                recv_time = time.time()
                data = sub_cam.recv()
                data = np.frombuffer(data, dtype=np.uint8)
                img = np.reshape(data, (480, 640, 3))
                print("sD img recv")
                # print(f"Sign Detection timedelta {time.time() - recv_time}")
                logger.log("PIPE", f"recv image {time.time() - recv_time}")
                count += 1
                start_time = time.time()
                if "stream" in self.outPnames:
                    classes, area, outimage = self.detection(img, bbox=True)
                else:
                    classes, area = self.detection(img)
                    
                print(classes, area)
                # print(f"sD compute time {time.time() - start_time:.2f}s")

                if "fzz" in self.outPnames:
                    idx = self.outPnames.index("fzz")
                    outPs[idx].send((1.0, classes))

                if "stream" in self.outPnames:
                    idx = self.outPnames.index("stream")
                    if outimage is None:
                        outPs[idx].send((1.0, img))
                    else:
                        outPs[idx].send((1.0, outimage))

            except Exception as e:
                print("Sign Detection error:")
                raise e
