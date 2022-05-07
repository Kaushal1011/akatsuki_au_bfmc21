import base64
import cv2
import zmq

from multiprocessing.connection import Connection
import socket
import struct
import time
from threading import Thread
import numpy as np
import cv2

from src.config import get_config

config = get_config()
HOST = config["pc_ip"]
from src.templates.workerprocess import WorkerProcess


def get_last(inP: Connection):
    timestamp, data = inP.recv()
    while inP.poll():
        # print("lk: skipping frame")
        # logger.log("SYNC", f"Skipping Frame delta - {time() - timestamp}")
        timestamp, data = inP.recv()
    return timestamp, data


connect2file = {"cam": "4lc", "sd": "62", "lk": "52"}


class CameraStreamerProcess(WorkerProcess):
    # ===================================== INIT =========================================
    def __init__(self, inPs, outPs, connect: str = "sd", port: int = 2244):
        """Process used for sending images over the network to a targeted IP via UDP protocol
        (no feedback required). The image is compressed before sending it.

        Used for visualizing your raspicam images on remote PC.

        Parameters
        ----------
        inPs : list(Pipe)
            List of input pipes, only the first pipe is used to transfer the captured frames.
        outPs : list(Pipe)
            List of output pipes (not used at the moment)
        """
        super(CameraStreamerProcess, self).__init__(inPs, outPs)
        self.port = port
        self.addr = f"tcp://{HOST}:{port}"
        self.file_id = connect2file[connect]
        self.connect = connect
#         if connect == "lk":
#             self.shape = (480, 640, 1)
#         else:
        self.shape = (480, 640, 3)

    #         self.frame_shm = sa.attach("shm://shared_frame1")

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads."""
        super(CameraStreamerProcess, self).run()

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the sending thread."""
        print("Streamer: Thread Init")
        if self._blocker.is_set():
            return
        streamTh = Thread(
            name="StreamSendingThread", target=self._send_thread, args=(self.inPs,)
        )
        streamTh.daemon = True
        self.threads.append(streamTh)

    # ===================================== SEND THREAD ==================================

    def _send_thread(self, inP: Connection):
        """Sending the frames received thought the input pipe to remote client by using the created socket connection.

        Parameters
        ----------
        inP : Pipe
            Input pipe to read the frames from CameraProcess or CameraSpooferProcess.
        """
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]
        context = zmq.Context()
        footage_socket = context.socket(zmq.PUB)
        print("Connecting to ", self.addr)
        footage_socket.setsockopt(zmq.CONFLATE, 1)
        footage_socket.connect(self.addr)

        context_sub = zmq.Context()
        sub_stream = context_sub.socket(zmq.SUB)
        # print("Binding Socket to", self.addr)
        sub_stream.setsockopt(zmq.CONFLATE, 1)
        sub_stream.connect(f"ipc:///tmp/v{self.file_id}")
        sub_stream.setsockopt_string(zmq.SUBSCRIBE, "")
        while True:
            try:
                data = sub_stream.recv()
                data = np.frombuffer(data, dtype=np.uint8)
                image = np.reshape(data, self.shape)
                if self.connect == "cam":
                    image = image[...,::-1]
                encoded, buffer = cv2.imencode(".jpg", image, encode_param)
                jpg_as_text = base64.b64encode(buffer)

                footage_socket.send(jpg_as_text)
                # print("Streamer -> sending Image")
                
            except Exception as e:
                print("CameraStreamer failed to stream images:", e, "\n")
                # Reinitialize the socket for reconnecting to client.
                self.connection = None
                raise e
