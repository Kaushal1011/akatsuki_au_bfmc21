from threading import Thread
import time

from src.templates.workerprocess import WorkerProcess
import platform

device = platform.uname().processor

if device == "x86_64":
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
        self.model, self.labels = setup()

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
            args=(self.inPs[0], self.outPs,),
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
        while True:
            try:
                # Obtain image
                stamps, img = inP.recv()
                # Apply image processing
                width = img.shape[1]
                height = img.shape[0]
                # should be top right quarter
                img = img[: int(height / 2), int(width / 2) :]

                a = time.time()
                out = detect_signs(img, self.model, self.labels)
                print("Time taken by model ", time.time() - a, "s")
                if out is not None:
                    print("Model prediction {label}")
                    box, label, location = out
                    # box 0 is top left box 1 is bottom right
                    # area = wxh w=x2-x1 h=y2-y1
                    area = (box[1][0] - box[0][0]) * (box[1][1] - box[0][1])
                    # if area < 10000:
                    #     continue

                    for outP in outPs:
                        outP.send(label)
                # print("Time taken by ID:", time() - a)
            except Exception as e:
                print("Intersection Detection error:")
                print(e)
