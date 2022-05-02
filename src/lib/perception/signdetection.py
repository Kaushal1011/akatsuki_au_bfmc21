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
        print(">>> Starting Sign Detection")
        context_recv = zmq.Context()

        sub_cam = context_recv.socket(zmq.SUB)
        sub_cam.setsockopt(zmq.CONFLATE, 1)
        sub_cam.connect("ipc:///tmp/v4l")
        sub_cam.setsockopt_string(zmq.SUBSCRIBE, '')
        
        context_send = zmq.Context()
        pub_sd = context_send.socket(zmq.PUB)
        pub_sd.bind("ipc:///tmp/v61")

        if self.enable_steam:
            context_send_img = zmq.Context()
            pub_sd_img = context_send_img.socket(zmq.PUB)
            pub_sd_img.bind("ipc:///tmp/v62")

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
                    pub_sd.send_json((classes,area), flags=zmq.NOBLOCK)
                    pub_sd_img.send(outimage.tobytes(), flags=zmq.NOBLOCK)

                else:
                    classes, area = self.detection(img)
                    pub_sd.send_json((classes,area), flags=zmq.NOBLOCK)

            except Exception as e:
                print("Sign Detection error:")
                raise e
