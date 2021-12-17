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

from templates.workerprocess import WorkerProcess

class LaneKeeping(WorkerProcess):
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
        super(LaneKeeping,self).__init__(inPs, outPs)
        
    def run(self):
        """Apply the initializing methods and start the threads.
        """
        super(LaneKeeping,self).run()

    def _init_threads(self):
        """Initialize the thread.
        """
        if self._blocker.is_set():
            return

        thr = Thread(name='StreamSending',target = self._the_thread, args= (self.inPs[0], self.outPs[0], ))
        thr.daemon = True
        self.threads.append(thr)


    # ===================================== Custom methods =========================================
    def laneKeeping(self, img):
        """Applies required image processing. 
        
        Parameters
        ----------
        img : Pipe
            The image on which to apply the algorithm.
        """
        # Image dimensions
        height = img.shape[0]
        width = img.shape[1]

        # Transform to grayscale
        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        # Crop to obtain region of interest
        img = img[(int(height/1.8)):height, 0:width]

        # Blur to remove the noise
        img = cv2.GaussianBlur(img, (7,7), 0)

        # Apply adaptive threshold to obtain the road markings
        img = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 21, -8)

        # Keep only a trapezoid containing the lines of the current lane
        region_of_interest_vertices = [
                (0, height - 1),
                (0.25*width, 0),
                (0.75*width, 0),
                (width - 1, height - 1),
        ]

        def region_of_interest(img, vertices):
            mask = np.zeros_like(img)
            match_mask_color = (255,)
            cv2.fillPoly(mask, vertices, match_mask_color)
            masked_image = cv2.bitwise_and(img, mask)
            return masked_image

        img = region_of_interest(
            img,
            np.array([region_of_interest_vertices], np.int32),
        )

        # Compute the hough lines from the image
        total = 0.0
        lines = cv2.HoughLinesP(img, rho=6, theta=np.pi/60, threshold=160, lines=np.array([]), minLineLength=40, maxLineGap=25)

        # Compute the sum of the slopes of the hough lines
        for line in lines:
            for x1, y1, x2, y2 in line:
                if y2 != y1:
                    total = total + (x2 - x1) / (y2 - y1)
                else: return 0.0

        return total

    def computeSteeringAngle(self, val):
        # Apply pid
        newVal = self.pid(val)

        # Calibrate result
        newVal = val / 2.9

        newVal = -newVal

        newVal += 3.1

        return newVal
        
    def _the_thread(self, inP, outP):
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
                # Obtain image
                stamps, img = inP.recv()

                # Apply image processing
                val = self.laneKeeping(img)

                # Compute steering angle
                val = self.computeSteeringAngle(val)

                # Print steering angle value
                #print(val)

                # Send steering angle value
                outP.send(val)

            except Exception as e:
                #print("Lane keeping error:")
                #print(e)
                1==1