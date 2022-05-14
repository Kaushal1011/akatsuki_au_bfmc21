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

import glob
import time
from threading import Thread
from typing import Tuple

import cv2
import numpy as np
import zmq
from src.templates.workerprocess import WorkerProcess
from typing import List
# from multiprocessing import shared_memory
# import SharedArray as sa

# # shm = shared_memory.SharedMemory(name="shared_frame", create=True, size=921600)
# try:
#     sa.delete("shared_frame1")
# except FileNotFoundError as e:
#     print(e)
# shared_frame = sa.create("shm://shared_frame1", (480, 640, 3), dtype=np.uint8)


class CameraSpooferProcess(WorkerProcess):

    # ================================ INIT ===============================================
    def __init__(self, outPsname: List[str], videoDir, ext=".avi"):
        """Processed used for spoofing a camera/ publishing a video stream from a folder
        with videos

        Parameters
        ----------
        inPs : list(Pipe)

        outPs : list(Pipe)
            list of output pipes(order does not matter)
        videoDir : [str]
            path to a dir with videos
        ext : str, optional
            the extension of the file, by default '.h264'
        """
        super(CameraSpooferProcess, self).__init__([],[])

        # params
        self.videoSize = (640, 480)
        self.videoDir = videoDir
        self.videos = self.open_files(self.videoDir, ext=ext)

        self.outPsname = outPsname

    # ===================================== INIT VIDEOS ==================================
    def open_files(self, inputDir, ext):
        """Open all files with the given path and extension

        Parameters
        ----------
        inputDir : string
            the input directory absolute path
        ext : string
            the extention of the files

        Returns
        -------
        list
            A list of the files in the folder with the specified file extension.
        """

        files = glob.glob(inputDir + "/*" + ext)
        return files

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the thread of the process."""

        thPlay = Thread(
            name="VideoPlayerThread", target=self.play_video, args=(self.videos,)
        )
        self.threads.append(thPlay)

    # ===================================== PLAY VIDEO ===================================
    def play_video(self, videos):
        """Iterate through each video in the folder, open a cap and publish the frames.

        Parameters
        ----------
        videos : list(string)
            The list of files with the videos.
        """
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

        while True:
            for video in videos:
                cap = cv2.VideoCapture(video)

                while True:
                    ret, frame = cap.read()
                    stamp = time.time()
                    if ret:
                        frame: np.ndarray = cv2.resize(frame, self.videoSize)
                        
                    if "lk" in self.outPsname:
                        pub_cam_lk.send(frame, flags=zmq.NOBLOCK)
                        # print("cam -> lk")

                    if "stream" in self.outPsname:
                        pub_cam_stream.send(frame, flags=zmq.NOBLOCK)
                        # print("cam -> stream")

                    if "sd" in self.outPsname:
                        pub_cam_sd.send(frame, flags=zmq.NOBLOCK)
                    

                cap.release()
