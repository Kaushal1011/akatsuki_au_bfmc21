from threading import Thread
from time import time
import math

from src.lib.perception.lanekeepfunctions import LaneKeep as LaneKeepMethod
from src.lib.cortex.posfushandle import Localize
from src.templates.workerprocess import WorkerProcess
from multiprocessing import Pipe
from multiprocessing.connection import Connection
from typing import List
#import  config

def get_last(inP: Pipe, delta_time: float = 0.1):
    data = inP.recv()
    timestamp = data["timestamp"]
    while (time() - timestamp) > delta_time:
        data = inP.recv()
        # print("xxxxxxxxxxxxxx Pos: skipping data")
        # print(time(), data)
        timestamp = data["timestamp"]

    return data


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
        # update gx gy based on initial values
        self.localize = Localize(ix=0,iy=0,gx=0,gy=0)

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
    def _the_thread(self, inPs: List[Connection], outPs: List[Connection]):
        """Obtains image, applies the required image processing and computes the steering angle value.

        Parameters
        ----------
        inP  : Pipe
            Input pipe to read the frames from other process.
        outP : Pipe
            Output pipe to send the steering angle value to other process.
        """
        print("Position Fusion ", self.inPsnames, self.inPs)
        try:
            while True:
                iyaw = None
                ipitch = None
                iroll = None
                ax = None
                ay = None
                az = None
                gx = None
                gy = None
                gyaw = None

                pos = list()
                if "loc" in self.inPsnames:
                    idx = self.inPsnames.index("loc")
                    if inPs[idx].poll():
                        loc:dict = get_last(inPs[idx], 0.5)
                        # print("LOC", time(), loc["timestamp"])
                        gx = loc["posA"]
                        gy = loc["posB"]
                        gyaw = loc["rotA"] if "rotA" in loc.keys() else loc["radA"]
                        # gyaw = 2 * math.pi - (gyaw + math.pi)

                if "imu" in self.inPsnames:
                    idx = self.inPsnames.index("imu")
                    if inPs[idx].poll():
                        idx = self.inPsnames.index("imu")
                        imu = get_last(inPs[idx])
                        # print("IMU", time(), imu["timestamp"])
                        iroll = imu["roll"]
                        ipitch = imu["pitch"]
                        iyaw = imu["yaw"]
                        # iyaw = 2 * math.pi - (iyaw + math.pi)

                        ax = imu["accelx"]
                        ay = imu["accely"]
                        az = imu["accelz"]

                if iyaw or gx:
                    pos_data = self.localize.update(
                        iyaw, ipitch, iroll, ax, ay, az, gx, gy, gyaw
                    )
                    self.outPs[0].send(pos_data)

        except Exception as e:
            raise e
