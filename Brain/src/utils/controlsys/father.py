import json
from std_msgs.msg import String
from src.templates.workerprocess import WorkerProcess
from threading import Thread
import rospy
import time
# import zmq
import pika
import random


class SimulatorConnector(WorkerProcess):
    """They call me "Father", all I do is connect them with the simulator."""\
        
    def __init__(self, inPs, outPs) -> None:
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5555")
        super(SimulatorConnector, self).__init__(inPs,outPs)

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
            
            topic = random.randrange(9999,10005)
            messagedata = random.randrange(1,215) - 80
            print("%d %d" % (topic, messagedata))
            self.socket.send_string("%d %d" % (topic, messagedata))
            time.sleep(1)

            try:
                # time.sleep(1.0)
                command = inP.recv()
                # file1.writelines(command)
                # if command is not None:
                    # print("SIM>", command)
                    # self.socket.send(json.dumps(command).encode('utf-8'))

            except Exception as e:
                print("Sim Connect error:")
                print(e)

        
