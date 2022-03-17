from threading import Thread
import time

from src.templates.workerprocess import WorkerProcess
import platform
from copy import deepcopy
import cv2
device = platform.uname().processor

if device == "x86_64":
    print("Using x86 model")
    from src.lib.perception.detectts_x86 import setup, detect_signs, draw_box
else:
    from src.lib.perception.detectts_armtflite import setup, detect_signs, draw_box


class SignDetectionProcess(WorkerProcess):
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
        super(SignDetectionProcess, self).__init__(inPs, outPs)

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(SignDetectionProcess, self).run()

    def _init_threads(self):
        """Initialize the thread."""
        if self._blocker.is_set():
            return

        thr = Thread(
            name="SignDetectionThread",
            target=self._the_thread,
            args=(self.inPs, self.outPs,),
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
        print("Started Sign Detection")
        count = 0
        model, labels = setup()
        label_areas = []
        while True:
            with open("labelareas.txt", "w") as f: 
                try:
                    stamps, img = inP[0].recv()
                    count += 1        
                    if count %5 != 0:
                        continue
                    print("R sD")
                    # Apply image processing
                    width = img.shape[1]
                    height = img.shape[0]
                    # should be top right quarter
                    img = img[: int(height / 2), int(width / 2) :]

                    a = time.time()
                    # print(self.model)
                    out = detect_signs(img, model, labels)
                    print("Time taken by model ", time.time() - a, "s")
                    if out is not None:
                        print("Model prediction {label}")
                        box, label, location = out
                        # box 0 is top left box 1 is bottom right
                        # area = wxh w=x2-x1 h=y2-y1
                        area = (box[1][0] - box[0][0]) * (box[1][1] - box[0][1])
                        # if area < 10000:
                        #     continue
                        frame = draw_box(img, label, location, box)

                        print(label, area)
                        f.write(f"{label}, {area}")
                        # for outP in outPs:
                        outPs[0].send((label, area))
                    if len(outPs) > 1:
                        if (frame).any():
                            outPs[1].send((1, frame))
                        else:
                            outPs[1].send((1, img))
                except Exception as e:
                    print("Sign Detection error:")
                    print(e)