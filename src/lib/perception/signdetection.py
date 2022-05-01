from threading import Thread
import time
from typing import List

from src.templates.workerprocess import WorkerProcess
import platform
import cv2
from multiprocessing.connection import Connection
from src.lib.perception.detect_ov import Detection
from loguru import logger
from multiprocessing import Value
import ctypes

loaded_model = Value(ctypes.c_bool, False)
# if device == "x86_64":
#     print("Using x86 model")
#     from src.lib.perception.detectts_x86 import setup, detect_signs, draw_box
# else:
#     from src.lib.perception.sign_det_cv import setup, detect_signs, draw_box


def get_last(inP: Connection):
    timestamp, data = inP.recv()
    while inP.poll():
        # print("lk: skipping frame")
        # logger.log("SYNC", f"Skipping Frame delta - {time() - timestamp}")
        timestamp, data = inP.recv()
    return timestamp, data


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
        print("Compiled Model", self.detection.compiled_model)
        global loaded_model
        loaded_model.value = True
        print(">>> Starting Sign Detection")
        while True:
            try:
                if inP[0].poll():
                    recv_time = time.time()
                    stamp, img = get_last(inP[0])
                    print(f"recv image {time.time() - recv_time}")
                    logger.log("PIPE", f"recv image {time.time() - recv_time}")
                    count += 1
                    start_time = time.time()
                    if "stream" in self.outPnames:
                        classes, area, outimage = self.detection(img, bbox=True)
                    else:
                        classes, area = self.detection(img)

                    print(f"sD compute time {time.time() - start_time:.2f}s")
                    # print("Model prediction {label}")
                    # box, label, location = out
                    # # box 0 is top left box 1 is bottom right
                    # # area = wxh w=x2-x1 h=y2-y1
                    # area = (box[1][0] - box[0][0]) * (box[1][1] - box[0][1])
                    # # if area < 10000:
                    # #     continue
                    # frame = draw_box(img, label, location, box)

                    # print(label, area)
                    # for outP in outPs:
                    # print((stamp, (classes, area)))

                    if "fzz" in self.outPnames:
                        idx = self.outPnames.index("fzz")
                        outPs[idx].send((stamp, classes))

                    if "stream" in self.outPnames:
                        idx = self.outPnames.index("stream")
                        if outimage is None:
                            outPs[idx].send((stamp, img))
                        else:
                            print("outimage shape", outimage.shape)
                            outPs[idx].send((stamp, outimage))

            except Exception as e:
                print("Sign Detection error:")
                raise e
