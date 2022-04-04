from typing import Tuple
import cv2
import requests
import numpy as np
import time
import json
import socket
from threading import Thread

from workerprocess import WorkerProcess

PI_IP = "0.0.0.0"
PORT = 8888


def localize(img: np.ndarray) -> Tuple[float, float]:
    AREA_THRES = 100.0
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # preprocess
    # kernel = np.ones((5,5), np.uint8)
    # img_erosion = cv2.erode(img, kernel, iterations=2)
    frame_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # (2, 29), (54, 255), (74, 255)
    # 8 116 102
    # 20 252 210
    frame_threshold = cv2.inRange(frame_HSV, (0, 200, 150), (30, 255, 255))
    # processed_img2 = cv2.bitwise_and(img,img, mask=frame_threshold)
    # get contours
    cnts = cv2.findContours(frame_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    # # find contour with max area
    # max = 0
    # idx = -1
    # for i,c in enumerate(cnts):
    #     area = cv2.contourArea(c)
    #     if area > max:
    #         idx = i
    #         max = area
    # blue_box = cnts[idx]
    x = None
    y = None
    if len(cnts) > 0:
        blue_box = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(blue_box) > AREA_THRES:
            # return its center point
            x, y, w, h = cv2.boundingRect(blue_box)
            x = x + w / 2
            y = y + h / 2
    # f_img = cv2.drawContours(processed_img2, [blue_box], -1, (255,0,0), 2)
    # plt.figure(figsize=(12,12))
    # plt.imshow(f_img[100:200,350:450])
    x = round(6 * x / 720, 2) if x else x
    y = round(6 * y / 720, 2) if y else y
    return x, y


def annotate_image(x: float, y: float, image: np.ndarray) -> np.ndarray:
    """Given x and y coordinates of data annotate the image."""
    org = [int((x * 720) / 6), int((y * 720) / 6)]
    if x > 500:
        org[0] = 500
    if y > 650:
        org[1] = 650
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.8
    color = (0, 0, 255)
    thickness = 2

    return cv2.putText(
        image,
        f"car({x},{y})",
        org,
        font,
        fontScale,
        color,
        thickness,
        cv2.LINE_AA,
    )


class LocalisationServer(WorkerProcess):
    """Home Localisation Server"""

    def __init__(self, preview=False) -> None:

        super(LocalisationServer, self).__init__(inPs=[], outPs=[])

        self.preview = preview
        self.port = PORT
        self.serverIp = "0.0.0.0"
        self.threads = list()

    def run(self):
        """Apply the initializing methods and start the threads."""
        self._init_threads()
        self._init_socket()
        for th in self.threads:
            th.start()

        for th in self.threads:
            th.join()
        super(LocalisationServer, self).run()

    def _init_threads(self):
        """Initialize the thread."""
        if self._blocker.is_set():
            return

        thr = Thread(
            name="LocalisationServer",
            target=self._the_thread,
            args=(),
        )
        thr.daemon = False
        self.threads.append(thr)

    def _init_socket(self):
        """Initialize the communication socket client."""
        self.client_socket = socket.socket(
            family=socket.AF_INET, type=socket.SOCK_DGRAM
        )

    def _the_thread(self):
        """Obtains image, applies the required image processing and computes the steering angle value.

        Parameters
        ----------
        inP  : Pipe
            Input pipe to read the frames from other process.
        outP : Pipe
            Output pipe to send the steering angle value to other process.
        """
        print("Started Home Localization System")
        count = 0
        skip_count = 24
        r = requests.get(
            "http://10.20.2.114/asp/video.cgi", auth=("admin", "admin"), stream=True
        )
        bytes1 = bytes()  # buffer
        if r.status_code == 200:
            for idx, chunk in enumerate(r.iter_content(chunk_size=240_000)):
                start_time = time.time()
                count += 1
                bytes1 += chunk
                a = bytes1.find(b"\xff\xd8")  # marks start of the frame
                b = bytes1.find(b"\xff\xd9")  # marks end   of the frame
                c = bytes1.rfind(b"\xff\xd9")  # the end of last frame in chunks

                if idx < skip_count or a == -1 or b == -1:
                    continue
                jpg = bytes1[a : b + 2]  # get frame based on markers
                bytes1 = bytes1[c + 2 :]  # update buffer to store data
                # of last frame present in chunk
                i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                # specify desired output size
                width = 720
                height = 1280
                # specify conjugate x,y coordinates (not y,x)
                input = np.float32([[2, 370], [589, 51], [1264, 66], [806, 719]])
                output = np.float32(
                    [[0, 0], [width - 1, 0], [width - 1, width - 1], [0, width - 1]]
                )

                # compute perspective matrixbytes1 = bytes1[b+2:]
                i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                # specify desired output size
                width = 720
                # height = 1280
                matrix = cv2.getPerspectiveTransform(input, output)

                # do perspective transformation setting area outside input to black
                image = cv2.warpPerspective(
                    i,
                    matrix,
                    (width, width),
                    cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT,
                    borderValue=(0, 0, 0),
                )
                x, y = localize(image)
                if x and y:
                    data = {
                        "timestamp": time.time(),
                        "posA": x,
                        "posB": y,
                        "rotA": 0,
                    }
                    data = json.dumps(data).encode()
                    self.client_socket.sendto(data, (self.serverIp, self.port))
                    print(data)
                    if self.preview:
                        imgOutput = annotate_image(x, y, image)
                        cv2.imshow("Track Image", imgOutput)
                        key = cv2.waitKey(1)
                        if key == 27 or key == 113:
                            cv2.destroyAllWindows()
                            break
                        else:
                            if key != -1:
                                print(key)
                            pass
        else:
            print("Received unexpected status code {}".format(r.status_code))
