from threading import Thread
from time import time
import math

from src.lib.perception.lanekeepfunctions import LaneKeep as LaneKeepMethod
from src.lib.cortex.posfushandle import Localize
from src.templates.workerprocess import WorkerProcess
from multiprocessing import Pipe
from multiprocessing.connection import Connection
from typing import List
from loguru import logger
import zmq

# import  config


def get_last(inP: Pipe, delta_time: float = 0.1):
    data = inP.recv()
    # timestamp = data["timestamp"]
    consume_start = time()
    while inP.poll():
        data = inP.recv()
        if time() - consume_start > 0.1:
            print("Time to move on")
            break
        # print("xxxxxxxxxxxxxx Pos: skipping data")
        # print(time(), data)
        # timestamp = data["timestamp"]
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
        self.localize = Localize(gx=0.8, gy=14.8, ix=0.8, iy=14.8)

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
        if "loc" in self.inPsnames:
            context_recv_loc = zmq.Context()
            sub_loc = context_recv_loc.socket(zmq.SUB)
            sub_loc.setsockopt(zmq.CONFLATE, 1)
            sub_loc.connect("ipc:///tmp/v31")
            sub_loc.setsockopt_string(zmq.SUBSCRIBE, "")

        if "imu" in self.inPsnames:
            context_recv_imu = zmq.Context()
            sub_imu = context_recv_imu.socket(zmq.SUB)
            sub_imu.setsockopt(zmq.CONFLATE, 1)
            sub_imu.connect("ipc:///tmp/v21")
            sub_imu.setsockopt_string(zmq.SUBSCRIBE, "")

        context_send = zmq.Context()
        pub_pos = context_send.socket(zmq.PUB)
        pub_pos.bind("ipc:///tmp/v42")

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
                if "imu" in self.inPsnames:
                    imu = sub_imu.recv_json()
                    print("IMU -> ", imu)
                    # print(f'imu delta {time()-imu["timestamp"]}')
                    logger.log("PIPE", f"imu {imu}")
                    # print("IMU", time(), imu["timestamp"])
                    iroll = imu["roll"]
                    ipitch = imu["pitch"]
                    iyaw = imu["yaw"]
                    # iyaw = 2 * math.pi - (iyaw + math.pi)
                    ax = imu["accelx"]
                    ay = imu["accely"]
                    az = imu["accelz"]

                if "loc" in self.inPsnames:
                    loc: dict = sub_loc.recv_json()
                    print("LOC -> ", loc)
                    gx = loc["posA"]
                    gy = loc["posB"]
                    gyaw = loc["rotA"] if "rotA" in loc.keys() else loc["radA"]
                    # gyaw = 2 * math.pi - (gyaw + math.pi)

                if (iyaw is not None) or (gx is not None):
                    pos_data = self.localize.update(
                            iyaw, ipitch, iroll, ax, ay, az, gx, gy, gyaw
                        )
                    
                    # print("pos_data", pos_data)
                    pub_pos.send_json(pos_data)

        except Exception as e:
            raise e
