import json
import socket
from threading import Thread

from src.templates.workerprocess import WorkerProcess


class ServerSIM(WorkerProcess):
    # ===================================== INIT =========================================
    def __init__(self, inPs, outPs, port: int, host_ip: str = "0.0.0.0", log=False):
        """Connect LocSys of simulator to Brain"""
        self.port = port
        self.host_ip = host_ip
        self.log = log
        super(ServerSIM, self).__init__(inPs, outPs)

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads"""
        self._init_socket()
        super(ServerSIM, self).run()

    # ===================================== INIT SOCKET ==================================
    def _init_socket(self):
        """Initialize the communication socket server."""
        self.port = self.port
        self.serverIp = self.host_ip

        self.server_socket = socket.socket(
            family=socket.AF_INET, type=socket.SOCK_DGRAM
        )
        self.server_socket.bind((self.serverIp, self.port))

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the read thread to transmite the received messages to other processes."""
        print("init_thread")
        readTh = Thread(
            name="SIMConnectThread", target=self._read_stream, args=(self.outPs,)
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
        print("Server Connect started! Now listening:\n")
        try:
            while True:
                bts, addr = self.server_socket.recvfrom(1024)
                bts = bts.decode()
                command = json.loads(bts)
                if self.log:
                    print(command)
                for outP in outPs:
                    outP.send(command)
        except Exception as e:
            raise e

        finally:
            self.server_socket.close()
