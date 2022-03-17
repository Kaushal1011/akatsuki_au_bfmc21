import cv2
import numpy as np
from functools import partial
import time
from typing import Tuple
import io
import picamera
import picamera.array

def empty(a):
    pass


def detection(img, mask, area_threshold: Tuple[int, int], label: str):
    imgRes = cv2.bitwise_and(img, img, mask=mask)
    gray = cv2.cvtColor(imgRes, cv2.COLOR_BGR2GRAY)
    contours, hierarchy = cv2.findContours(
        gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < area_threshold[1] and area > area_threshold[0]:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.05 * peri, True)
            text = label
            # cv2.drawContours(imgContour, [approx], -1, (255, 0, 255), 7)
            # print(len(approx))
            x_, y_, w, h = cv2.boundingRect(approx)
            box = [(x_, y_), (x_ + w, y_ + h)]

            location = (x_, y_)
            # cv2.rectangle(imgContour, (x_, y_), (x_ + w, y_ + h), (0, 255, 0), 5)
            return box, text, location
    return None


def detect_signs(img, model, labels):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blue1 = np.array([110, 50, 50])
    blue2 = np.array([130, 255, 255])
    red1 = np.array([120, 0, 0])
    red2 = np.array([180, 255, 255])
    yellow1 = np.array([10, 52, 50])
    yellow2 = np.array([40, 255, 255])

    masks = [
        cv2.inRange(hsv_img, red1, red2),
        # cv2.inRange(hsv_img, blue1, blue2),
        # cv2.inRange(hsv_img, yellow1, yellow2),
    ]
    f_box = None
    f_text = None
    f_location = None
    max_area = 0
    return detection(img, masks[0], [5, 50_000], "stop")
    for mask, label in zip(masks, labels):
        out = detection(img, mask, [1000, 50_000], label)
        if out:
            box, text, location, area = out
            if area > max_area:
                f_box = box
                f_text = text
                f_location = location
    if f_box:
        return f_box, f_text, f_location
    return None


def setup():
    print("Starting pseudo sign detection")
    PATH_TO_LABELS = "./labels.txt"

    detect_fn = None

    with open(PATH_TO_LABELS) as f:
        my_list = list(f)

    category_index = [i.strip("\n") for i in my_list]
    return detect_fn, category_index


def draw_box(img, text, location, box):
    fontScale = 1
    color = (255, 0, 0)
    thickness = 1
    font = cv2.FONT_HERSHEY_SIMPLEX

    retimg = cv2.rectangle(img, box[0], box[1], color, 4)
    retimg = cv2.putText(
        retimg, text, location, font, fontScale, color, thickness, cv2.LINE_AA
    )
    return retimg


if __name__ == "__main__":
    
    interpreter, labels = setup()
    with picamera.PiCamera() as camera:
        camera.resolution = (1640, 1232)
        camera.framerate = 15

        camera.brightness = 60
        camera.shutter_speed = 1200
        camera.contrast = 50
        camera.iso = 0  # auto
        camera.exposure_mode = "night"
        camera.start_preview()
        time.sleep(0.1)
        with picamera.array.PiRGBArray(camera) as stream:
            camera.capture(stream, format='bgr')
            # At this point the image is available as stream.array
            image = stream.array

            out = detect_signs(image, interpreter, labels)
            if out:
                box, text, location = out
                print(box, text, location)
                cv2.imwrite("object_detection.jpg", draw_box(image, text, location, box))
            else:
                cv2.imwrite("object_detection.jpg", image)
                
