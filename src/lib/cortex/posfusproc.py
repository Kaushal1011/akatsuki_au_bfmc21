from threading import Thread
from time import time


from src.lib.perception.lanekeepfunctions import LaneKeep as LaneKeepMethod
from src.templates.workerprocess import WorkerProcess
from multiprocessing import Pipe


def get_last(inP: Pipe, delta_time: float = 0.1):
    timestamp, data = inP.recv()

    while (time() - timestamp) > delta_time:
        timestamp, data = inP.recv()

    return timestamp, data


class PositionFusionProcess(WorkerProcess):
    # ===================================== Worker process =========================================
    def __init__(self, inPs, outPs, inPsnames=[]):
        """Process used for the image processing needed for lane keeping and for computing the steering value.

        Parameters
        ----------
        inPs : list(Pipe)
            List of input pipes (0 - receive image feed from the camera)
        outPs : list(Pipe)
            List of output pipes (0 - send steering data to the movvement control process)
        """
        super(PositionFusionProcess, self).__init__(inPs, outPs)
        self.inPsnames = inPsnames

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(PositionFusionProcess, self).run()

    def _init_threads(self):
        """Initialize the thread."""
        if self._blocker.is_set():
            return

        thr = Thread(
            name="PositionFusionThread",
            target=self._the_thread,
            args=(
                self.inPs,
                self.outPs,
            ),
        )
        thr.daemon = True
        self.threads.append(thr)

    # ===================================== Custom methods =========================================
    def _the_thread(self, inPs, outPs):
        """Obtains image, applies the required image processing and computes the steering angle value.

        Parameters
        ----------
        inP  : Pipe
            Input pipe to read the frames from other process.
        outP : Pipe
            Output pipe to send the steering angle value to other process.
        """
        try:
            while True:
                if "loc" in self.inPsnames:
                    idx = self.inPsnames.index("loc")
                    loc = inPs[idx].recv()
                    print("PosFzz: LOC", loc)

                if "imu" in self.inPsnames:
                    idx = self.inPsnames.index("imu")
                    imu = inPs[idx].recv()
                    print("PosFzz: IMU", imu)

                if loc and imu:
                    self.outPs[0].send((loc, imu))
                elif loc:
                    self.outPs[0].send(loc)
                elif imu:
                    self.outPs.send(imu)

        except Exception as e:
            raise e
