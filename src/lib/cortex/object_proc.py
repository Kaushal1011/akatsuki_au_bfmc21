from pathlib import Path
from threading import Thread
from time import time
from typing import List
import cv2

from src.lib.perception.lanekeepfunctions import LaneKeep as LaneKeepMethod
from src.lib.cortex.posfushandle import Localize
from src.templates.workerprocess import WorkerProcess
from multiprocessing import Pipe
from multiprocessing.connection import Connection


def get_last(inP: Connection, delta_time: float = 0.1):
    if inP.poll():
        data = inP.recv() 
        while (time() - data["timestamp"]) > delta_time:
            if inP.poll():
                data = inP.recv()
            else:
                return None
        
        return data
    else:
        return None

def get_last_frame(inP: Pipe, delta_time: float = 0.1):

    timestamp, data = inP.recv()
    while (time() - timestamp) > delta_time:
        # print("lk: skipping frame")
        timestamp, data = inP.recv()
    
    return timestamp, data


class DetectCar:
    def __init__(self) -> None:
        #car_cascade_path = Path(Path(__file__).parent.parent.parent.resolve(), "data", "car_cascade.xml")
        #print(car_cascade_path)
        self.car_cascade = cv2.CascadeClassifier("/home/kaypee/akatsuki_au_bfmc21/src/data/car_cascade.xml")

    def __call__(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        car_bbox = self.car_cascade.detectMultiScale(gray, 1.15, 4)
        if len(car_bbox) != 0:
            return True
        return False

class ObjectProcess(WorkerProcess):
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
        super(ObjectProcess, self).__init__(inPs, outPs)
        self.detect_car = DetectCar()

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(ObjectProcess, self).run()

    def _init_threads(self):
        """Initialize the thread."""
        if self._blocker.is_set():
            return

        thr = Thread(
            name="ObjectThread",
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
        try:
            while True:
                # RUN filters for distance 
                # RUN object detection here
                stamp, image = get_last_frame(inPs[0], 0.01)
                distance_data = get_last(inPs[1], 0.1)
                if distance_data:
                    print(time(), distance_data["timestamp"])
                    # detected_car  = self.detect_car(image)
                    # SEND (front_distance, side_distance, det_car, det_ped, det_closed_road)
                    outPs[0].send((distance_data["sonar1"], distance_data["sonar2"], False, False, False))

        except Exception as e:
            raise e
