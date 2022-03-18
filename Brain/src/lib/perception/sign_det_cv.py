import cv2
from cv2 import blur
import numpy as np
from functools import partial
import time
from typing import Tuple


def empty(a):
    pass


def detection(img, mask, area_threshold: Tuple[int, int], label: str):
    
    imgContour = img.copy()
    black1 = np.array([0,0,0])
    black2 = np.array([180,20,20])
    imgRes = cv2.bitwise_and(img, img, mask=mask)
    blur = cv2.GaussianBlur(imgRes, (7,7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 55, 30)
    kernel = np.ones((5,5))
    dilate = cv2.dilate(canny, kernel, iterations=1)
    contours, hierarchy = cv2.findContours(
        dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    hull = []
    
    for cnt in contours:
        hull.append(cv2.convexHull(cnt, False))
        area = cv2.contourArea(cnt)
        if area < area_threshold[0] and area > area_threshold[1]:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.1 * peri, True)
            text = label
            cv2.drawContours(imgContour, hull, -1, (255, 0, 255), 7)

            x, y, w, h = cv2.boundingRect(approx)
            cropped_contour = img[y:y+h,x:x+w]
            cv2.imshow('', cropped_contour)
            maskb = cv2.inRange(cropped_contour, black1, black2)
            croppedRes = cv2.bitwise_and(cropped_contour, cropped_contour, mask=maskb)
            croppedBlur = cv2.GaussianBlur(croppedRes, (7,7), 1)
            croppedGray = cv2.cvtColor(croppedBlur, cv2.COLOR_BGR2GRAY)
            croppedCanny = cv2.Canny(croppedGray, 55, 30)
            kernel = np.ones((7,7))
            croppedDil = cv2.dilate(croppedCanny, kernel, iterations=1)
            contours_crop, hierarchy = cv2.findContours(croppedDil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if len(contours_crop) != 0:
                print("Crosswalk")

            box = [(x, y), (x + w, y + h)]
            location = (x, y)
            return box, text, location
    return None


def detect_signs(img, model, labels):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blue1 = np.array([90,70,30])
    blue2 = np.array([130,255,255])
    red1 = np.array([0,90,35])
    red2 = np.array([5,255,255])
    yellow1 = np.array([22,244,78])
    yellow2 = np.array([24,246,80])
    masks = [
        cv2.inRange(hsv, blue1, blue2),
        cv2.inRange(hsv, red1, red2),
        cv2.inRange(hsv, yellow1, yellow2)
    ]
    f_box = None
    f_text = None
    f_location = None
    max_area = 0
    maskb = cv2.bitwise_or(masks[0],masks[1])
    maskf = cv2.bitwise_or(maskb,masks[2])
    return detection(img, maskf, [2500, 700], "stop")
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
    color = (0, 255, 0)
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
    cap = cv2.VideoCapture(0)
    cap.set(3, frameWidth)
    cap.set(4, frameHeight)
    interpreter, labels = setup()
    frame = cv2.imread("/home/b0nzo/akatsuki_au_bfmc21/nbs/stop.png")
    frame = frame[0:np.int32(frame.shape[0]/2), np.int32(frame.shape[1]/2):np.int32(frame.shape[1])]
    while True:
        out = detect_signs(frame, interpreter, labels)
        if out:
            box, text, location = out
            print(box, text, location)
            cv2.imshow('',draw_box(frame, text, location, box))
        else:
            cv2.imshow('',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
