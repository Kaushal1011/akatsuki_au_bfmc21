import cv2
from cv2 import blur
import numpy as np
from functools import partial
import time
from typing import Tuple


def check_stop(img, area_threshold: Tuple[int, int]):
    imgContour = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    red1 = np.array([0,200,110])
    red2 = np.array([5,255,120])
    red3 = np.array([170,70,40])
    red4 = np.array([180,255,255])
    mask1 = cv2.inRange(hsv, red1, red2)
    mask2 = cv2.inRange(hsv, red3, red4)
    mask = cv2.bitwise_or(mask1, mask2)
    imgRes = cv2.bitwise_and(img, img, mask=mask)
    # cv2.imshow("ii", imgRes)
    blur = cv2.GaussianBlur(imgRes, (7,7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 55, 35)
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
            approx = cv2.approxPolyDP(cnt, 0.2 * peri, True)
            cv2.drawContours(imgContour, hull, -1, (255, 0, 255), 8)
            x, y, w, h = cv2.boundingRect(approx)
            
            return True, x, y, w, h
    return None,None,None,None,None

def check_priority(img, area_threshold: Tuple[int, int]):
    imgContour = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    yellow1 = np.array([18,235,75])
    yellow2 = np.array([28,255,85])
    mask = cv2.inRange(hsv, yellow1, yellow2)
    imgRes = cv2.bitwise_and(img, img, mask=mask)
    blur = cv2.GaussianBlur(imgRes, (7,7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 55, 30)
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
    return None,None,None,None,None

def check_cross(img, area_threshold: Tuple[int, int]):
    imgContour = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    blue1 = np.array([90,90,70])
    blue2 = np.array([140,255,255])
    white1 = np.array([135,254,254])
    white2 = np.array([140,255,255])
    black1 = np.array([0,0,0])
    black2 = np.array([180,20,20])
    mask1 = cv2.inRange(hsv, blue1, blue2)
    mask2 = cv2.inRange(hsv, white1, white2)
    maskf = cv2.bitwise_or(mask1, mask2)
    imgRes = cv2.bitwise_and(img, img, mask=maskf)
    # cv2.imshow("ii", imgRes)
    blur = cv2.GaussianBlur(imgRes, (7,7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 55, 30)
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
            approx = cv2.approxPolyDP(cnt, 0.05 * peri, True)
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 8)
            x, y, w, h = cv2.boundingRect(approx)

            cropped_contour = img[y:y+h,x:x+w]
            croppedGray = cv2.cvtColor(cropped_contour, cv2.COLOR_BGR2GRAY)
            black_pix = np.sum(croppedGray == 0)
            gray_pix = np.sum(croppedGray >= 128)
            print("black pix: ",black_pix)
            print("white pix: ",gray_pix)
            if black_pix >= 20 and gray_pix == 0:   
                return True, x, y, w, h
    return None,None,None,None,None

def check_park(img, area_threshold: Tuple[int, int]):
    imgContour = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    blue1 = np.array([90,90,70])
    blue2 = np.array([140,255,255])
    white1 = np.array([135,254,254])
    white2 = np.array([140,255,255])
    black1 = np.array([0,0,0])
    black2 = np.array([180,20,20])
    mask1 = cv2.inRange(hsv, blue1, blue2)
    mask2 = cv2.inRange(hsv, white1, white2)
    maskf = cv2.bitwise_or(mask1, mask2)
    imgRes = cv2.bitwise_and(img, img, mask=maskf)
    cv2.imshow("ii", imgRes)
    blur = cv2.GaussianBlur(imgRes, (7,7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 55, 30)
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
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 8)
            x, y, w, h = cv2.boundingRect(approx)

            cropped_contour = img[y:y+h,x:x+w]
            croppedGray = cv2.cvtColor(cropped_contour, cv2.COLOR_BGR2GRAY)
            black_pix = np.sum(croppedGray == 0)
            gray_pix = np.sum(croppedGray >= 128)
            print("black_pix:", black_pix)
            print("white pix: ",gray_pix)
            if black_pix < 15 and gray_pix > 0:   
                return True, x, y, w, h
    return None,None,None,None,None


     

def detections(img, label):
    text = "not detected"
    cs,csx,csy,csw,csh=check_stop(img,(700,25000))
    cp,cpx,cpy,cpw,cph=check_priority(img,(400,25000))
    cpa,cpax,cpay,cpaw,cpah=check_park(img,(700,25000))
    cc,ccx,ccy,ccw,cch=check_cross(img,(700,25000))
    box,text,location=None,None,None
    if cs:
        box = [(csx, csy), (csx + csw, csy + csh)]
        location = csx, csy
        text = "stop"
    elif cp:
        box = [(cpx, cpy), (cpx + cpw, cpy + cph)]
        location = cpx, cpy
        text = "priority"
    elif cc:
        box = [(ccx, ccy), (ccx + ccw, ccy + cch)]
        location = ccx, ccy
        text = "crosswalk"
    elif cpa:
        box = [(cpax, cpay), (cpax + cpaw, cpay + cpah)]
        location = cpax, cpay
        text = "parking"
    else:
        box = [(0,0), (0, 0)]
        location = 0, 0
        text = None

    return box, text, location


def detect_signs(image,model,labels):
    return model(image,labels)        


def setup():
    print("Starting pseudo sign detection")
    detect_fn = detections
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
        out = detect_signs(frame, interpreter, labels)
        if out:
            box, text, location = out
            print(box, text, location)
            cv2.imshow('',draw_box(frame, text, location, box))
        else:
            cv2.imshow('',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
