import cv2
import numpy as np
from functools import partial
import time
from typing import Tuple


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
    red1 = np.array([0, 50, 50])
    red2 = np.array([20, 255, 255])
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

    retimg = cv2.rectangle(img, box[0], box[1], color, thickness)
    retimg = cv2.putText(
        retimg, text, location, font, fontScale, color, thickness, cv2.LINE_AA
    )
    return retimg


if __name__ == "__main__":

    frameWidth = 640
    frameHeight = 480
    cap = cv2.VideoCapture()
    cap.set(3, frameWidth)
    cap.set(4, frameHeight)
    interpreter, labels = setup()

    # img = cv2.imread("./data/IMG_1048.JPG", cv2.IMREAD_COLOR)
    # scale_percent = 20  # percent of original size
    # width = int(img.shape[1] * scale_percent / 100)
    # height = int(img.shape[0] * scale_percent / 100)
    # dim = (width, height)

    # # resize image
    # frame = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
    # print(frame.shape)
    while True:
        ret, frame = cap.read()

        out = detect_signs(frame, interpreter, labels)
        if out:
            box, text, location = out
            print(box, text, location)
            cv2.imshow("object detection", draw_box(frame, text, location, box))

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
