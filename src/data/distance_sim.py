from threading import Thread

from src.templates.workerprocess import WorkerProcess
import zmq


class DistanceSIM(WorkerProcess):
    # ===================================== INIT =========================================
    def __init__(
        self, inPs, outPs,  port: int, host_ip: str = "0.0.0.0", log=False
    ):
        """Connect Distance of simulator to Brain"""
        self.addr = f"tcp://{host_ip}:{port}"
        self.log = log
        super(DistanceSIM, self).__init__(inPs, outPs)

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads"""
        super(DistanceSIM, self).run()

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
        pub_sim.setsockopt(zmq.CONFLATE, 1)
        pub_sim.bind(f"ipc:///tmp/v11")

        # Subscribe to data from simulator
        context = zmq.Context()
        distance_socket = context.socket(zmq.SUB)
        print("Binding Socket to", self.addr)
        distance_socket.setsockopt(zmq.CONFLATE, 1)
        distance_socket.bind(self.addr)
        distance_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        try:
            while True:
                data = distance_socket.recv_json()
                if self.log:
                    print("DIS sent ->", data)
                pub_sim.send_json(data, flags=zmq.NOBLOCK)
                # for outP in outPs:
                #     outP.send(command)
        except Exception as e:
            raise e

        finally:
            self.server_socket.close()
