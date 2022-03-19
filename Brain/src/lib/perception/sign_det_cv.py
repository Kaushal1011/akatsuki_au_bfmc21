import cv2
import numpy as np
from typing import Tuple
import time


def empty(a):
    pass

def check_stop(img, area_threshold: Tuple[int, int]):
    imgContour = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
    red1 = np.array([0,90,35])
    red2 = np.array([10,255,255])
    red3 = np.array([350,70,40])
    red4 = np.array([360,255,255])
    mask1 = cv2.inRange(hsv, red1, red2)
    mask2 = cv2.inRange(hsv, red3, red4)
    mask = cv2.bitwise_or(mask1, mask2)
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
            approx = cv2.approxPolyDP(cnt, 0.2 * peri, True)
            cv2.drawContours(imgContour, hull, -1, (255, 0, 255), 8)
            x, y, w, h = cv2.boundingRect(approx)
            
            return True, x, y, w, h
    return None,None,None,None,None

def check_priority(img, area_threshold: Tuple[int, int]):
    imgContour = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
    yellow1 = np.array([40,70,60])
    yellow2 = np.array([60,90,80])
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
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
    blue1 = np.array([220,70,20])
    blue2 = np.array([240,100,40])
    black1 = np.array([0,0,0])
    black2 = np.array([180,20,20])
    mask = cv2.inRange(hsv, blue1, blue2)
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
            # print(area)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.1 * peri, True)
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 8)
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
    return None,None,None,None,None

def check_park(img, area_threshold: Tuple[int, int]):
    imgContour = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
    blue1 = np.array([220,70,20])
    blue2 = np.array([240,100,40])
    black1 = np.array([0,0,0])
    black2 = np.array([180,20,20])
    mask = cv2.inRange(hsv, blue1, blue2)
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
    return None,None,None,None,None




def detect_signs(img, label):
    text = "not detected"
    cs,csx,csy,csw,csh=check_stop(img,(700,25000))
    cp,cpx,cpy,cpw,cph=check_priority(img,(700,25000))
    cpa,cpax,cpay,cpaw,cpah=check_park(img,(700,25000))
    cc,ccx,ccy,ccw,cch=check_cross(img,(700,25000))
    box,text,location=None,None,None
    if cs:
        box = [(csx, csy), (csx + csw, csy + csh)]
        location = csx, csy
        text = label[0]
    elif cp:
        box = [(cpx, cpy), (cpx + cpw, cpy + cph)]
        location = cpx, cpy
        text = label[3]
    elif cpa:
        box = [(cpax, cpay), (cpax + cpaw, cpay + cpah)]
        location = cpax, cpay
        text = label[1]
    elif cc:
        box = [(ccx, ccy), (ccx + ccw, ccy + cch)]
        location = ccx, ccy
        text = label[2]
    else:
        box = [(0,0), (0, 0)]
        location = 0, 0
        text = "no sign"

    return box, text, location

        


def setup():
    print("Starting pseudo sign detection")
    detect_fn = detect_signs
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
    path = "/home/b0nzo/Downloads/bfmc-images/bfmc2020_online_1.avi"
    frameWidth = 640
    frameHeight = 480
    cap = cv2.VideoCapture(path)
    cap.set(3, frameWidth)
    cap.set(4, frameHeight)
    interpreter, labels = setup()
    # frame = cv2.imread("/home/b0nzo/akatsuki_au_bfmc21/nbs/cross.png")
    
    while True:
        ret, frame = cap.read()
        frame = frame[0:np.int32(frame.shape[0]/2), np.int32(frame.shape[1]/2):np.int32(frame.shape[1])]
        out = detect_signs(frame, labels)
        if out:
            box, text, location = out
            # print(text)
            cv2.imshow('',draw_box(frame, text, location, box))
            time.sleep(0.03)
        else:
            pass
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # print(text)
            break
            
    
