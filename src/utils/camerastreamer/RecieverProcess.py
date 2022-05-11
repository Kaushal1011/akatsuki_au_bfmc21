import cv2
import zmq
import base64
import numpy as np

from multiprocessing.connection import Connection
import sys

sys.path.append(".")

from threading import Thread
import cv2
import numpy as np

from src.templates.workerprocess import WorkerProcess
from src.lib.perception.lanekeepfunctions import LaneKeep
from src.lib.perception.detect_ov import Detection


class CameraReceiverProcess(WorkerProcess):
    # ===================================== INIT =========================================
    def __init__(self, inPs: Connection, outPs: Connection, port: int):
        """Process used for debugging. Can be used as a direct frame analyzer, instead of using the VNC
        It receives the images from the raspberry and displays them.

        Parameters
        ----------
        inPs : list(Pipe)
            List of input pipes
        outPs : list(Pipe)
            List of output pipes
        """
        super(CameraReceiverProcess, self).__init__(inPs, outPs)
        self.addr = f"tcp://*:{port}"
        self.lk = LaneKeep(use_perspective=True, computation_method="hough")

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializers and start the threads."""
        super(CameraReceiverProcess, self).run()

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the read thread to receive and display the frames."""
        readTh = Thread(name="StreamReceivingThread", target=self._read_stream)
        self.threads.append(readTh)

    # ===================================== READ STREAM ==================================
    def _read_stream(self):
        """Read the image from input stream, decode it and display it with the CV2 library."""
        context = zmq.Context()
        footage_socket = context.socket(zmq.SUB)
        print("Binding Socket to", self.addr)
        footage_socket.setsockopt(zmq.CONFLATE, 1)
        footage_socket.bind(self.addr)
        footage_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        count = 1
        detection = Detection()
        while True:
            try:
                frame = footage_socket.recv_string()
                img = base64.b64decode(frame)
                npimg = np.fromstring(img, dtype=np.uint8)
                image = cv2.imdecode(npimg, 1)
                imagedet = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                _, _, lk_out = self.lk(image, True)
                _, det_out = detection(imagedet, bbox=True)
                cv2.imshow("LK Stream", lk_out)
                cv2.imshow("SD Stream", det_out)
                if cv2.waitKey(1) & 0xFF == ord("s"):
                    count += 1
                    # print(f"Saving Image {count}.jpg")
                    # cv2.imwrite(f"./dataset/{count}.jpg", source)
            except Exception as e:
                print(e)
                cv2.destroyAllWindows()
                raise e
