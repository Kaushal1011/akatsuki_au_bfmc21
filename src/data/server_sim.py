import json
import socket
from threading import Thread

from src.templates.workerprocess import WorkerProcess
import zmq


connect2file = {"loc": "31", "imu": "21", "tl": "tl"}


class ServerSIM(WorkerProcess):
    # ===================================== INIT =========================================
    def __init__(
        self, inPs, outPs, connect: str, port: int, host_ip: str = "0.0.0.0", log=False
    ):
        """Connect LocSys of simulator to Brain"""
        self.port = port
        self.host_ip = host_ip
        self.log = log
        self.file_id = connect2file[connect]
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
        context_send = zmq.Context()
        pub_sim = context_send.socket(zmq.PUB)
        pub_sim.bind(f"ipc:///tmp/v{self.file_id}")

        try:
            while True:
                bts, addr = self.server_socket.recvfrom(1024)
                bts = bts.decode()
                command = json.loads(bts)
                if self.log:
                    print(command)
                pub_sim.send_json(command, flags=zmq.NOBLOCK)
                # for outP in outPs:
                #     outP.send(command)
        except Exception as e:
            raise e

        finally:
            self.server_socket.close()
