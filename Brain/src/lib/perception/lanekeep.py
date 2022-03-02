from threading import Thread
from time import time

# from simple_pid import PID
from src.lib.perception.lanekeephandle import LaneKeep as LaneKeepMethod
from src.templates.workerprocess import WorkerProcess

MAX_STEER = 17


class LaneKeepingProcess(WorkerProcess):
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
        super(LaneKeepingProcess, self).__init__(inPs, outPs)
        self.lk = LaneKeepMethod(use_perspective=False, computation_method="hough")

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(LaneKeepingProcess, self).run()

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

    # ===================================== Custom methods =========================================
    def computeSteeringAnglePID(self, val):
        # keep the angle between max steer angle
        val = max(-MAX_STEER, min(val - 90, MAX_STEER))
        return val

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
                i = time()
                stamps, img = inP.recv()
                # print("Time taken to recieve image", time()- i)
                a = time()
                # Apply image processing
                val, outimage = self.lk(img)
                angle = self.computeSteeringAnglePID(val)

                for outP in outPs:
                    outP.send((angle, None))

                # print("Timetaken by LK: ", time() - a)

            except Exception as e:
                print("Lane keeping error:")
                print(e)
