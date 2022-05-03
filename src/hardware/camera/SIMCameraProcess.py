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

import sys

sys.path.append(".")

import socket
import struct
import time
from threading import Thread

import cv2
import numpy as np

from src.templates.workerprocess import WorkerProcess
import zmq

HOST = "0.0.0.0"  # Standard loopback interface address (localhost)
PORT = 5555


class SIMCameraProcess(WorkerProcess):
    # ===================================== INIT =========================================
    def __init__(self, inPs, outPs, outPsname):
        """Process used for debugging. Can be used as a direct frame analyzer, instead of using the VNC
        It receives the images from the raspberry and displays them.

        Parameters
        ----------
        inPs : list(Pipe)
            List of input pipes
        outPs : list(Pipe)
            List of output pipes
        """
        super(SIMCameraProcess, self).__init__(inPs, outPs)
        self.outPsname = outPsname
        self.imgSize = (480, 640, 3)
        self.addr = f"tcp://*:{PORT}"

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializers and start the threads."""
        super(SIMCameraProcess, self).run()

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the read thread to receive and display the frames."""
        readTh = Thread(
            name="StreamReceivingThread",
            target=self._read_stream,
            args=(self.outPs,),
        )
        self.threads.append(readTh)

    # ===================================== READ STREAM ==================================
    def _read_stream(self, outPs):
        """Read the image from input stream, decode it and display it with the CV2 library."""
        try:
            if "lk" in self.outPsname:
                context_lk = zmq.Context()
                pub_cam_lk = context_lk.socket(zmq.PUB)
                pub_cam_lk.setsockopt(zmq.CONFLATE, 1)
                pub_cam_lk.bind("ipc:///tmp/v4l")

            if "sd" in self.outPsname:
                context_sd = zmq.Context()
                pub_cam_sd = context_sd.socket(zmq.PUB)
                pub_cam_sd.setsockopt(zmq.CONFLATE, 1)
                pub_cam_sd.bind("ipc:///tmp/v4ls")

            if "stream" in self.outPsname:
                context_stream = zmq.Context()
                pub_cam_stream = context_stream.socket(zmq.PUB)
                pub_cam_stream.setsockopt(zmq.CONFLATE, 1)
                pub_cam_stream.bind("ipc:///tmp/v4lc")

            context = zmq.Context()
            footage_socket = context.socket(zmq.SUB)
            print("Binding Socket to", self.addr)
            footage_socket.setsockopt(zmq.CONFLATE, 1)
            footage_socket.bind(self.addr)
            footage_socket.setsockopt_string(zmq.SUBSCRIBE, "")

            while True:

                # ----------------------- read image -----------------------
                data = footage_socket.recv()
                image = np.frombuffer(data, np.uint8)
                image = cv2.imdecode(image, cv2.IMREAD_COLOR)
                image = np.reshape(image, self.imgSize)
                image: np.ndarray = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                if "lk" in self.outPsname:
                    pub_cam_lk.send(image, flags=zmq.NOBLOCK)
                    # print("cam -> lk")

                if "stream" in self.outPsname:
                    pub_cam_stream.send(image, flags=zmq.NOBLOCK)
                    # print("cam -> stream")

                if "sd" in self.outPsname:
                    pub_cam_sd.send(image, flags=zmq.NOBLOCK)

        except Exception:
            pass
        finally:
            self.connection.close()
            footage_socket.close()
