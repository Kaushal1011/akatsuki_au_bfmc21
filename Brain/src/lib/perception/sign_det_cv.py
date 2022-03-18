from cProfile import label
import cv2
from cv2 import blur
import numpy as np
from functools import partial
import time
from scipy import stats as st
from typing import List, Tuple


def empty(a):
    pass

def check_stop(img, area_threshold: Tuple[int, int]):
    imgContour = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red1 = np.array([0,90,35])
    red2 = np.array([5,255,255])
    mask = cv2.inRange(hsv, red1, red2)
    imgRes = cv2.bitwise_and(img, img, mask=mask)
    blur = cv2.GaussianBlur(imgRes, (7,7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 10, 35)
    kernel = np.ones((7,7))
    dilate = cv2.dilate(canny, kernel, iterations=1)
    hull = []
    contours, hierarchy = cv2.findContours(
        dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        hull.append(cv2.convexHull(cnt, False))
        area = cv2.contourArea(cnt)
        if area > area_threshold[0] and area < area_threshold[1]:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.1 * peri, True)
            cv2.drawContours(imgContour, hull, -1, (255, 0, 255), 8)
            x, y, w, h = cv2.boundingRect(approx)
            
            return True, x, y, w, h

def check_priority(img, area_threshold: Tuple[int, int]):
    imgContour = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    yellow1 = np.array([22,244,78])
    yellow2 = np.array([24,246,80])
    mask = cv2.inRange(hsv, yellow1, yellow2)
    imgRes = cv2.bitwise_and(img, img, mask=mask)
    blur = cv2.GaussianBlur(imgRes, (7,7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 10, 35)
    kernel = np.ones((7,7))
    dilate = cv2.dilate(canny, kernel, iterations=1)
    hull = []
    contours, hierarchy = cv2.findContours(
        dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        hull.append(cv2.convexHull(cnt, False))
        area = cv2.contourArea(cnt)
        if area > area_threshold[0] and area < area_threshold[1]:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.1 * peri, True)
            cv2.drawContours(imgContour, hull, -1, (255, 0, 255), 8)
            x, y, w, h = cv2.boundingRect(approx)
            
            return True, x, y, w, h

def check_cross(img, area_threshold: Tuple[int, int]):
    imgContour = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blue1 = np.array([90,70,30])
    blue2 = np.array([130,255,255])
    black1 = np.array([0,0,0])
    black2 = np.array([180,20,20])
    mask = cv2.inRange(hsv, blue1, blue2)
    imgRes = cv2.bitwise_and(img, img, mask=mask)
    blur = cv2.GaussianBlur(imgRes, (7,7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 10, 35)
    kernel = np.ones((7,7))
    dilate = cv2.dilate(canny, kernel, iterations=1)
    hull = []
    contours, hierarchy = cv2.findContours(
        dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        hull.append(cv2.convexHull(cnt, False))
        area = cv2.contourArea(cnt)
        if area > area_threshold[0] and area < area_threshold[1]:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.1 * peri, True)
            cv2.drawContours(imgContour, hull, -1, (255, 0, 255), 8)
            x, y, w, h = cv2.boundingRect(approx)

            cropped_contour = img[y:y+h,x:x+w]
            maskb = mask = cv2.inRange(cropped_contour, black1, black2)
            crop_res = cv2.bitwise_and(cropped_contour, cropped_contour, mask=maskb)
            croppedBlur = cv2.GaussianBlur(crop_res, (7,7), 1)
            croppedGray = cv2.cvtColor(croppedBlur, cv2.COLOR_BGR2GRAY)
            croppedCanny = cv2.Canny(croppedGray, 55, 30)
            kernel = np.ones((7,7))
            croppedDil = cv2.dilate(croppedCanny, kernel, iterations=1)
            contours_crop, hierarchy = cv2.findContours(croppedDil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if len(contours_crop) != 0:     
                return True, x, y, w, h

def check_park(img, area_threshold: Tuple[int, int]):
    imgContour = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blue1 = np.array([90,70,30])
    blue2 = np.array([130,255,255])
    black1 = np.array([0,0,0])
    black2 = np.array([180,20,20])
    mask = cv2.inRange(hsv, blue1, blue2)
    imgRes = cv2.bitwise_and(img, img, mask=mask)
    blur = cv2.GaussianBlur(imgRes, (7,7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 10, 35)
    kernel = np.ones((7,7))
    dilate = cv2.dilate(canny, kernel, iterations=1)
    hull = []
    contours, hierarchy = cv2.findContours(
        dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        hull.append(cv2.convexHull(cnt, False))
        area = cv2.contourArea(cnt)
        if area > area_threshold[0] and area < area_threshold[1]:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.1 * peri, True)
            cv2.drawContours(imgContour, hull, -1, (255, 0, 255), 8)
            x, y, w, h = cv2.boundingRect(approx)

            cropped_contour = img[y:y+h,x:x+w]
            maskb = mask = cv2.inRange(cropped_contour, black1, black2)
            crop_res = cv2.bitwise_and(cropped_contour, cropped_contour, mask=maskb)
            croppedBlur = cv2.GaussianBlur(crop_res, (7,7), 1)
            croppedGray = cv2.cvtColor(croppedBlur, cv2.COLOR_BGR2GRAY)
            croppedCanny = cv2.Canny(croppedGray, 55, 30)
            kernel = np.ones((7,7))
            croppedDil = cv2.dilate(croppedCanny, kernel, iterations=1)
            contours_crop, hierarchy = cv2.findContours(croppedDil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if len(contours_crop) == 0:   
                return True, x, y, w, h




def detection(img, label):
    text = "not detected"
    if check_stop(img)[0]:
        _,x,y,w,h = check_stop(img)
        box = [(x, y), (x + w, y + h)]
        text = label[0]
    elif check_priority(img)[0]:
        _,x,y,w,h = check_stop(img)
        box = [(x, y), (x + w, y + h)]
        text = label[3]
    elif check_park(img)[0]:
        _,x,y,w,h = check_stop(img)
        box = [(x, y), (x + w, y + h)]
        text = label[1]
    elif check_cross(img)[0]:
        _,x,y,w,h = check_stop(img)
        box = [(x, y), (x + w, y + h)]
        text = label[2]

    return box, text

        


def setup():
    print("Starting pseudo sign detection")
    detect_fn = None
    my_list = ["stop", "parking", "crosswalk", "priority"]

    
    return detect_fn, my_list


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
        out = detection(frame, labels)
        if out:
            box, text, location = out
            # print(text)
            cv2.imshow('',draw_box(frame, text, location, box))
        else:
            pass
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # print(text)
            break
            
    
