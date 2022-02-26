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
        self.speed = 15.0
        self.init = False
        super(MovementControl, self).__init__(inPs, outPs)

    def _init_threads(self):
        """Initialize the a thread for initial start and a thread for listening for the steering angle."""

        startTh = Thread(
            name="InitialStart", target=self._singleUpdate, args=(self.outPs,)
        )
        self.threads.append(startTh)

        sendTh = Thread(
            name="SteeringListen",
            target=self._listen_for_steering,
            args=(
                self.inPs[0],
                self.outPs,
            ),
        )
        self.threads.append(sendTh)

        # signTh = Thread(name='SignListen',target = self._listen_for_stop, args = (self.inPs[1], self.outPs, ))
        # self.threads.append(signTh)

    def run(self):
        """Apply the initializing methods and start the threads"""
        super(MovementControl, self).run()

    def stop(self):
        """Apply the stopping methods and stops the threads"""
        # Make a reset before stop
        self.speed = 0.0
        self.angle = 0.0
        self._singleUpdate(self.outPs)

        super(MovementControl, self).stop()

    # ===================================== Custom methods =========================================
    def _listen_for_steering(self, inP, outPs):
        """Get the current needed value for the steering angle"""
        while True:
            try:
                # Get the value through the pipe
                value, _ = inP.recv()

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
                    self.speed = 20.0

                self._singleUpdate(outPs)
            except Exception as e:
                print(e)

    def _set_PID(self, outPs):
        """Set PID to True and configure PID"""
        pid_conf_data = {}
        pid_conf_data["action"] = "6"
        pid_conf_data["kp"] = 0.115000
        pid_conf_data["ki"] = 0.810000
        pid_conf_data["kd"] = 0.000222
        pid_conf_data["tf"] = 0.040000

        pid_activate_data = {}
        pid_activate_data["action"] = "4"
        pid_activate_data["activate"] = True
        return pid_activate_data, pid_conf_data

    def _singleUpdate(self, outPs):
        """Update the state of the controls"""
        # Initialize the data array to be sent
        speed_data = {}

        speed_data["action"] = "1"
        speed_data["speed"] = float(self.speed / 100.0)
        # else:
        #     data['action'] = 'BRAK'

        steer_data = {}
        # Set lateral control
        steer_data["action"] = "2"
        steer_data["steerAngle"] = float(self.angle)
        # print(data)
        # Send data
        try:
            for outP in outPs:
                if not self.init:
                    pid_activate_date, pid_conf_data = self._set_PID()
                    outP.send(pid_activate_date)
                    outP.send(pid_conf_data)
                    self.init = True

                outP.send(speed_data)
                outP.send(steer_data)
        except Exception as e:
            print(e)
