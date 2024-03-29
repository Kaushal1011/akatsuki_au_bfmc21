from threading import Thread
import time
from typing import List

from src.templates.workerprocess import WorkerProcess
from multiprocessing.connection import Connection
from src.lib.perception.detect_ov import Detection
from loguru import logger
import zmq
import numpy as np
import cv2

from multiprocessing import Value
import ctypes

loaded_model = Value(ctypes.c_bool, False)


class SignDetectionProcess(WorkerProcess):
    # ===================================== Worker process =========================================
    def __init__(
        self,
        inPs: Connection,
        outPs: Connection,
        outPnames: List[str],
        enable_stream: bool = True,
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
        self.enable_steam = enable_stream

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
        global loaded_model
        loaded_model.value = True

        context_recv = zmq.Context()
        sub_cam = context_recv.socket(zmq.SUB)
        sub_cam.setsockopt(zmq.CONFLATE, 1)
        sub_cam.connect("ipc:///tmp/v4ls")
        sub_cam.setsockopt_string(zmq.SUBSCRIBE, "")

        context_send = zmq.Context()
        pub_sd = context_send.socket(zmq.PUB)
        pub_sd.setsockopt(zmq.CONFLATE, 1)
        pub_sd.bind("ipc:///tmp/v61")

        if self.enable_steam:
            context_send_img = zmq.Context()
            pub_sd_img = context_send_img.socket(zmq.PUB)
            pub_sd_img.setsockopt(zmq.CONFLATE, 1)
            pub_sd_img.bind("ipc:///tmp/v62")

        while True:
            try:
                recv_time = time.time()
                data = sub_cam.recv()
                data = np.frombuffer(data, dtype=np.uint8)
                img = np.reshape(data, (480, 640, 3))
                # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

                # print("sD img recv")
                # print(f"Sign Detection timedelta {time.time() - recv_time}")
                count += 1
                start_time = time.time()
                #     detections, outimage = self.detection(img, bbox=True)
                #     pub_sd.send_json(detections, flags=zmq.NOBLOCK)
                #     pub_sd_img.send(outimage.tobytes(), flags=zmq.NOBLOCK)
                # else:
                detections, outimage = self.detection(img, bbox=True)
                pub_sd.send_json(detections, flags=zmq.NOBLOCK)
                if self.enable_steam:
                    pub_sd_img.send(outimage.tobytes(), flags=zmq.NOBLOCK)

                # logger.log("SD", f"detections -> {detections}")

            except Exception as e:
                print("Sign Detection error:")
                raise e
