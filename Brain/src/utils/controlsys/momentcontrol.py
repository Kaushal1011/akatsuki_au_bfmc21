import time

from threading import Thread

from src.templates.workerprocess import WorkerProcess

class MovementControl(WorkerProcess):
    # ===================================== Worker process =========================================
    def __init__(self, inPs, outPs):
        """Controls the speed and steering of the vehicle
        
        Parameters
        ------------
        inPs  : list(Pipe)
            List of input pipes (0 - steering angle)
        outPs : list(Pipe) 
            List of output pipes (order does not matter)
        """
        # Initialize parameters
        self.angle = 0.0
        self.speed = 21.0

        super(MovementControl,self).__init__(inPs, outPs)

    def _init_threads(self):
        """Initialize the a thread for initial start and a thread for listening for the steering angle.
        """

        startTh = Thread(name='InitialStart', target = self._singleUpdate, args=(self.outPs, ))
        self.threads.append(startTh)

        sendTh = Thread(name='SteeringListen',target = self._listen_for_steering, args = (self.inPs[0], self.outPs, ))
        self.threads.append(sendTh)

        # signTh = Thread(name='SignListen',target = self._listen_for_stop, args = (self.inPs[1], self.outPs, ))
        # self.threads.append(signTh)
        
    def run(self):
        """Apply the initializing methods and start the threads
        """
        super(MovementControl,self).run()

    def stop(self):
        """Apply the stopping methods and stops the threads
        """
        # Make a reset before stop
        self.speed = 0.0
        self.angle = 0.0
        self._singleUpdate(self.outPs)

        super(MovementControl, self).stop()

    # ===================================== Custom methods =========================================
    def _listen_for_steering(self, inP, outPs):
        """Get the current needed value for the steering angle
        """
        while True:
            try:
                # Get the value through the pipe
                value = inP.recv()

                # Write the value
                self.angle = float(value)

                # Update the value on Nucleo
                self._singleUpdate(outPs)
            except Exception as e:
                print("Listening error:")
                print(e)

    def _listen_for_stop(self, inP, outPs):
        while True:
            try:
                value = inP.recv()

                if value == 0:
                    self.speed = 0.0
                if value == 1:
                    self.speed = 0.0
                    self._singleUpdate(outPs)
                    time.sleep(2)
                    self.speed = 21.0

                self._singleUpdate(outPs)
            except Exception as e:
                print(e)

    def _singleUpdate(self, outPs):
        """Update the state of the controls
        """
        # Initialize the data array to be sent
        data = {}

        # Set longitudinal control
        # if(self.speed != 0):
        #     data['action'] = 'MCTL'
        #     data['speed'] = float(self.speed/100.0)
        # else:
        #     data['action'] = 'BRAK'

        # Set lateral control
        data["action"] = "2"
        data['steerAngle'] = float(self.angle)
        print(data)
        # Send data
        try:
            for outP in outPs:
                outP.send(data)
        except Exception as e:
            print(e)
            