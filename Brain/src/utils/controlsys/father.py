import json
from std_msgs.msg import String
from src.templates.workerprocess import WorkerProcess
from threading import Thread
import rospy
import time

# import zmq
import random
import socket

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 65432


class SimulatorConnector(WorkerProcess):
    """They call me "Father", all I do is connect them with the simulator."""

    def __init__(self, inPs, outPs) -> None:

        self.port = PORT
        self.serverIp = HOST

        self.client_socket = socket.socket(
            family=socket.AF_INET, type=socket.SOCK_DGRAM
        )
        super(SimulatorConnector, self).__init__(inPs, outPs)

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(SimulatorConnector, self).run()

    def _init_threads(self):
        """Initialize the thread."""
        if self._blocker.is_set():
            return

        thr = Thread(
            name="SimConnect",
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
                # time.sleep(1.0)
                command = inP.recv()
                if command is not None:
                    command = json.dumps(command).encode()
                    self.client_socket.sendto(command, (self.serverIp, self.port))

            except Exception as e:
                # raise e
                print("Sim Connect error:")
                print(e)
