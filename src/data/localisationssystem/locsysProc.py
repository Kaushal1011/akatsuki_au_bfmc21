import math
import time
from threading import Thread

from src.data.localisationssystem.locsys import LocalisationSystem
from src.templates.workerprocess import WorkerProcess

CARID = 4


class LocalisationSystemProcess(WorkerProcess):
    # ================================ CAMERA PROCESS =====================================
    def __init__(self, inPs, outPs, daemon=True):
        """Process that start the raspicam and pipes it to the output pipe, to another process.

        Parameters
        ----------
        inPs : list()
            input pipes (leave empty list)
        outPs : list()
            output pipes (order does not matter, output camera image on all pipes)
        daemon : bool, optional
            daemon process flag, by default True
        """
        super(LocalisationSystemProcess, self).__init__(inPs, outPs, daemon=True)

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads."""
        super(LocalisationSystemProcess, self).run()

    # ===================================== INIT TH ======================================
    def _init_threads(self):
        """Create the Camera Publisher thread and add to the list of threads."""
        trafficTh = Thread(
            name="LocalisationSystemThread", target=self.runListener, args=(self.outPs)
        )
        self.threads.append(trafficTh)

    def runListener(self, outPs):
        # Get time stamp when starting tester
        # Create listener object
        locsys = LocalisationSystem(CARID)
        # Start the listener
        locsys.start()
        # Wait until 60 seconds passed
        while True:
            try:
                coora = locsys.coor()
                if coora:
                    data = {
                        "timestamp": coora["timestamp"],
                        "posA": coora["coor"][0].real,
                        "posB": coora["coor"][0].imag,
                        "radA": math.atan2(
                            coora["coor"][1].real, coora["coor"][1].imag
                        ),
                    }
                    for outP in [outPs]:
                        outP.send((time.time(), data))
                time.sleep(1)
            except KeyboardInterrupt:
                break

        LocalisationSystem.stop()
        LocalisationSystem.join()
