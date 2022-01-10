import socket
import struct
import time
import numpy as np
import datetime
import cv2
import math

from multiprocessing    import Process
from threading          import Thread
from simple_pid         import PID
from src.lib.lanekeeputils import LaneKeep as LaneKeepMethod
from src.templates.workerprocess import WorkerProcess

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image


class LaneKeepingProcess(WorkerProcess):
    pid = PID(Kp = 1.0, Ki = 1.45, Kd = 0.15)
    
    # ===================================== Worker process =========================================
    def __init__(self, inPs, outPs):
        """Process used for the image processing needed for lane keeping and for computing the steering value.
        
        Parameters
        ----------
        inPs : list(Pipe) 
            List of input pipes (0 - receive image feed from the camera)
        outPs : list(Pipe) 
            List of output pipes (0 - send steering data to the movvement control process)
        """
        super(LaneKeepingProcess,self).__init__(inPs, outPs)
        self.lk = LaneKeepMethod(use_perspective=True, computation_method="hough")
        
    def run(self):
        """Apply the initializing methods and start the threads.
        """
        super(LaneKeepingProcess,self).run()

    def _init_threads(self):
        """Initialize the thread.
        """
        if self._blocker.is_set():
            return

        thr = Thread(name='StreamSending',target = self._the_thread, args= (self.inPs[0], self.outPs[0], ))
        thr.daemon = True
        self.threads.append(thr)


    # ===================================== Custom methods =========================================
    # def laneKeeping(self, img:np.ndarray):
    #     """Applies required image processing. 
        
    #     Parameters
    #     ----------
    #     img : Pipe
    #         The image on which to apply the algorithm.
    #     """
    #     return self.lk(img)

    def computeSteeringAnglePID(self, val):
        # keep the angle between max steer angle
        val = max(-17, min(val - 90, 17))   
        # # Apply pid
        # newVal = self.pid(val)

        # # Calibrate result
        # newVal = val / 2.9

        # newVal = -newVal

        # newVal += 0

        return val
        
    def _the_thread(self, inP, outP):
        """Obtains image, applies the required image processing and computes the steering angle value. 
        
        Parameters
        ----------
        inP  : Pipe
            Input pipe to read the frames from other process.
        outP : Pipe
            Output pipe to send the steering angle value to other process.
        """
        firstimage = True
        while True:
            try:
                # Obtain image
                stamps, img = inP.recv()
                # Apply image processing
                val = self.lk(img)
                print(f"Computed angle :{val}")
                angle = self.computeSteeringAnglePID(val)
                outimage = display_heading_line(img, angle)
                if firstimage:
                    final_img = display_heading_line(img, angle)
                    cv2.imwrite("./sampleimage.jpg", final_img)
                    firstimage=False
                # Compute steering angle
                # val = self.computeSteeringAngle(val)

                # Print steering angle value
                # print(f"Steer angle is: {val}")
                # Send steering angle value
                
                outP.send(angle)

            except Exception as e:
                print("Lane keeping error:")
                print(e)
                1==1
