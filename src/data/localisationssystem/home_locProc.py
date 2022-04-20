import json
import socket
from threading import Thread
from src.templates.workerprocess import WorkerProcess
import time

REMOTE_PORT = 8888


class LocalisationProcess(WorkerProcess):
    # ===================================== INIT =========================================
    def __init__(self, inPs, outPs):
        """Run on raspberry. It forwards the control messages received from socket to the serial handler

        Parameters
        ------------
        inPs : list(Pipe)
            List of input pipes (not used at the moment)
        outPs : list(Pipe)
            List of output pipes (order does not matter)
        """

        super(LocalisationProcess, self).__init__(inPs, outPs)

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads"""
        self._init_socket()
        super(LocalisationProcess, self).run()

    # ===================================== INIT SOCKET ==================================
    def _init_socket(self):
        """Initialize the communication socket server."""
        self.port = REMOTE_PORT
        self.serverIp = "0.0.0.0"

        self.server_socket = socket.socket(
            family=socket.AF_INET, type=socket.SOCK_DGRAM
        )
        self.server_socket.bind((self.serverIp, self.port))

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the read thread to transmite the received messages to other processes."""
        readTh = Thread(
            name="LocSysRecvThread", target=self._read_stream, args=(self.outPs,)
        )
        self.threads.append(readTh)

    # ===================================== READ STREAM ==================================
    def _read_stream(self, outPs):
        """Receive the message and forwards them to the SerialHandlerProcess.

        Parameters
        ----------
        outPs : list(Pipe)
            List of the output pipes.
        """
        # self.server_socket.setblocking(False)
        try:
            while True:
                bts, addr = self.server_socket.recvfrom(1024)
                bts = bts.decode()
                command = json.loads(bts)
                for outP in outPs:
                    outP.send(command)

        except Exception as e:
            print("Home LocSys Error")
            print(e)

        finally:
            self.server_socket.close()
