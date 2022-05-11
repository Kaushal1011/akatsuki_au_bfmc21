# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

from multiprocessing.connection import Connection
import socket
import struct
import time
from threading import Thread
import zmq

# import SharedArray as sa
import cv2
import base64
from src.config import get_config
import numpy as np

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


connect2file = {
    "cam": "4l",
    "sd": "62",
}


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
        self.file_id = connect2file[connect]

    #         self.frame_shm = sa.attach("shm://shared_frame1")

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads."""
        self._init_socket()
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

    # ===================================== INIT SOCKET ==================================
    def _init_socket(self):
        """Initialize the socket client."""
        self.serverIp = HOST  # PC ip
        # self.serverIp = "0.0.0.0"  # PC ip

        # self.port  # port

        self.client_socket = socket.socket()
        self.connection = None
        # Trying repeatedly to connect the camera receiver.
        print("Streamer: Initialising Socket")
        try:
            while self.connection is None and not self._blocker.is_set():
                try:
                    self.client_socket.connect((self.serverIp, self.port))
                    self.client_socket.setsockopt(
                        socket.SOL_SOCKET, socket.SO_REUSEADDR, 1
                    )
                    self.connection = self.client_socket.makefile("wb")
                except ConnectionRefusedError:
                    time.sleep(0.5)
                    pass
        except KeyboardInterrupt:
            self._blocker.set()
            pass

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

        sub_stream = context.socket(zmq.SUB)
        # print("Binding Socket to", self.addr)
        sub_stream.setsockopt(zmq.CONFLATE, 1)

        sub_stream.connect(f"ipc:///tmp/v{self.file_id}")
        sub_stream.setsockopt_string(zmq.SUBSCRIBE, "")
        while True:
            try:
                # stamp, image = get_last(inP)
                # print(f"Stream timedelta -> {time.time() - stamp}s")
                # image = np.array(self.frame_shm).copy()
                # print(stamps, image)
                # cv2.imshow("Image", image)
                # cv2.waitKey(1)

                data = sub_stream.recv()
                data = np.frombuffer(data, dtype=np.uint8)
                image = np.reshape(data, (480, 640, 3))
                # print("Stream", image.shape)
                pack_time = time.time()
                # print(f"Streamer timedelta {(time.time() - stamp):.4f}s")
                result, image = cv2.imencode(".jpg", image, encode_param)
                data = image.tobytes()
                size = len(data)

                self.connection.write(struct.pack("d", 1.0))
                # print(f"Streaming | sending data size: {size}, timestamp:{stamp}")
                self.connection.write(struct.pack("<L", size))
                self.connection.write(data)
                print(f"Pack Time -> {(time.time() - pack_time):.4f}")
            except Exception as e:
                print("CameraStreamer failed to stream images:", e, "\n")
                # Reinitialize the socket for reconnecting to client.
                self.connection = None
                self._init_socket()
                raise e
