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
        context = zmq.Context()
        self.footage_socket = context.socket(zmq.SUB)
        addr = f'tcp://0.0.0.0:{port}'
        print("Binding Socket to", addr)
        self.footage_socket.bind(addr)
        self.footage_socket.setsockopt_string(zmq.SUBSCRIBE, np.unicode(''))


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
        while True:
            try:
                frame = self.footage_socket.recv_string()
                img = base64.b64decode(frame)
                npimg = np.fromstring(img, dtype=np.uint8)
                source = cv2.imdecode(npimg, 1)
                cv2.imshow("Stream", source)
                cv2.waitKey(1)
            except Exception as e:
                print(e)
                cv2.destroyAllWindows()
                raise e

