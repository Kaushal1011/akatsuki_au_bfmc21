import cv2
from cv2 import blur
import numpy as np
from functools import partial
import time
from typing import Tuple


def check_car(frame):
    tl_cascade = cv2.CascadeClassifier("car_cascade.xml")
    # frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tls = tl_cascade.detectMultiScale(gray, 1.15, 4)
    if tls.len() !=0:
        return True, tls[0], tls[1], tls[2], tls[3]
    return False,0,0,0,0

def check_cross(frame):
    tl_cascade = cv2.CascadeClassifier("cross_cascade.xml")
    # frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tls = tl_cascade.detectMultiScale(gray, 1.35, 3)
    if tls.len() !=0:
        return True, tls[0], tls[1], tls[2], tls[3]
    return False, 0, 0, 0, 0

def check_tl(frame):
    tl_cascade = cv2.CascadeClassifier("tl_cascade.xml")
    # frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tls = tl_cascade.detectMultiScale(gray, 1.3, 3)
    if tls.len() !=0:
        return True, tls[0], tls[1], tls[2], tls[3]
    return False,0,0,0,0

def detections(img, label):
    text = "not detected"
    ctl,ctlx,ctly,ctlw,ctlh=check_tl(img)
    ccar,ccarx,ccary,ccarw,ccarh=check_car(img)
    cc,ccx,ccy,ccw,cch=check_cross(img)
    box,text,location=None,None,None
    if ctl:
        box = [(ctlx, ctly), (ctlx + ctlw, ctly + ctlh)]
        location = ctlx, ctly
        text = label[0]
    elif ccar:
        box = [(ccarx, ccary), (ccarx + ccarw, ccary + ccarh)]
        location = ccarx, ccary
        text = label[3]
    elif cc:
        box = [(ccx, ccy), (ccx + ccw, ccy + cch)]
        location = ccx, ccy
        text = label[7]
    else:
        box = [(0,0), (0, 0)]
        location = 0, 0
        text = "no sign"

    return box, text, location

        


def setup():
    print("Starting pseudo sign detection")
    detect_fn = detections
    my_list = ["traffic light", "stop", "priority", "car", "highway starts", "one way", "parking", "crosswalk"]

    
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
    frame = cv2.imread("crosswalk.jpeg")
    frame = frame[0:np.int32(frame.shape[0]/2), np.int32(frame.shape[1]/2):np.int32(frame.shape[1])]
    while True:
        out = detections(frame, interpreter, labels)
        if out:
            box, text, location = out
            print(box, text, location)
            cv2.imshow('',draw_box(frame, text, location, box))
        else:
            cv2.imshow('',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
