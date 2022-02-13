import numpy as np

from threading import Thread
from src.lib.intersectiondethandle import intersection_det
from src.templates.workerprocess import WorkerProcess
import time


class IntersectionDetProcess(WorkerProcess):
    # ===================================== Worker process =========================================
    def __init__(self, inPs, outPs):
        """Process used for the image processing needed for lane keeping and for computing the steering value.

        Parameters
        ----------
        inPs : list(Pipe)
            List of input pipes (0 - receive image feed from the camera)
        outPs : list(Pipe)
            List of output pipes (0 - send steering data to the movvement control process)
        """
        super(IntersectionDetProcess, self).__init__(inPs, outPs)

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(IntersectionDetProcess, self).run()

    def _init_threads(self):
        """Initialize the thread."""
        if self._blocker.is_set():
            return

        thr = Thread(
            name="StreamSending",
            target=self._the_thread,
            args=(
                self.inPs[0],
                self.outPs,
            ),
        )
        thr.daemon = True
        self.threads.append(thr)

    def _the_thread(self, inP, outPs):
        """Obtains image, applies the required image processing and computes the steering angle value.

        Parameters
        ----------
        inP  : Pipe
            Input pipe to read the frames from other process.
        outP : Pipe
            Output pipe to send the steering angle value to other process.
        """
        while True:
            try:
                # Obtain image
                stamps, img = inP.recv()
                # Apply image processing
                detected, _ = intersection_det(img)
                for outP in outPs:
                    outP.send(detected)

            except Exception as e:
                print("Intersection Detection error:")
                print(e)
