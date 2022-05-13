import math
import os
import pathlib
import time
from threading import Thread

from src.data.localisationssystem.locsys import LocalisationSystem
from src.templates.workerprocess import WorkerProcess
from multiprocessing import Pipe
import zmq


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
        self.outPs = outPs

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

    def runListener(self):
        # Get time stamp when starting tester
        # Create listener object
        beacon = 12345
        id = 89
        serverpublickey = pathlib.Path(
            pathlib.Path(__file__).parent.resolve(), "publickey_server.pem"
        )
        gpsStR, gpsStS = Pipe(duplex=False)
        locsys = LocalisationSystem(id, beacon, serverpublickey, gpsStS)

        # Start the listener
        locsys.start()
        # Wait until 60 seconds passed
        context_send = zmq.Context()
        pub_loc = context_send.socket(zmq.PUB)
        pub_loc.setsockopt(zmq.CONFLATE, 1)
        pub_loc.bind("ipc:///tmp/v31")
        print("Starting Localizaion Server")
        while True:
            try:
                coora = gpsStR.recv()
                if coora:
                    data = {
                        "timestamp": coora["timestamp"],
                        "posA": coora["coor"][0].real,
                        "posB": coora["coor"][0].imag,
                        "radA": math.atan2(
                            coora["coor"][1].real, coora["coor"][1].imag
                        ),
                    }
                    # print("LOC", data)
                    pub_loc.send_json(data, flags=zmq.NOBLOCK)
            except KeyboardInterrupt:
                break

        LocalisationSystem.stop()
        LocalisationSystem.join()
