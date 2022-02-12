import json
from std_msgs.msg import String
from src.templates.workerprocess import WorkerProcess
from threading import Thread
import rospy

# file1 = open("Command.txt", "w") 

class SimulatorConnector(WorkerProcess):
    """They call me "Father", all I do is connect them with the simulator."""\
        
    def __init__(self, inPs, outPs) -> None:
        rospy.init_node("SimConnect", anonymous=False)
        self.publisher = rospy.Publisher('/automobile/command', String, queue_size=1)
        print("<sim setup DONE")

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

    def _send_command(self, key):
        """Transmite the command to the remotecontrol receiver. 
        
        Parameters
        ----------
        inP : Pipe
            Input pipe. 
        """
        command = self.rcBrain.getMessage(key)
        if command is not None:
            command = json.dumps(command)
            self.publisher.publish(command)
    
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
                command = inP.recv()
                # file1.writelines(command)
                if command is not None:
                    print("SIM>", command)
                    command = json.dumps(command)
                    self.publisher.publish(command)

            except Exception as e:
                print("Sim Connect error:")
                print(e)

        
