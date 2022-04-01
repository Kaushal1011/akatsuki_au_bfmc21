import cv2
import numpy as np
from typing import Tuple
import time

def check_stop(img, area_threshold: Tuple[int, int]):
    imgContour = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red1 = np.array([0,70,70])
    red2 = np.array([10,255,255])
    red3 = np.array([350,70,70])
    red4 = np.array([360,255,255])
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
    return False,0,0,0,0

def check_priority(img, area_threshold: Tuple[int, int]):
    imgContour = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    yellow1 = np.array([18,70,70])
    yellow2 = np.array([28,255,255])
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
    return False,0,0,0,0

def check_cross(img, area_threshold: Tuple[int, int]):
    sift = cv2.SIFT_create()
    index_params = dict(algorithm = 0, trees = 5)
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    img1 = cv2.imread("/home/b0nzo/akatsuki_au_bfmc21/nbs/cross_ref.png")
    img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blue1 = np.array([90,90,70])
    blue2 = np.array([140,255,255])
    mask1 = cv2.inRange(hsv, blue1, blue2)
    imgRes = cv2.bitwise_and(img, img, mask=mask1)
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
            x, y, w, h = cv2.boundingRect(approx)
            cropped_image = img[y:y+h,x:x+w]
            cropped_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
            kp1, des1 = sift.detectAndCompute(img1, None)
            kp2, des2 = sift.detectAndCompute(cropped_image, None)
            bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)
            matches = flann.knnMatch(des1, des2, k=2)
            good_matches = []
            for m, n in matches:
                if m.distance < 0.8 * n.distance:
                    good_matches.append(m)
            if len(good_matches) > 6:
                return True, x, y, w, h
    return False,0,0,0,0
                       
    

def check_park(img, area_threshold: Tuple[int, int]):
    sift = cv2.SIFT_create()
    index_params = dict(algorithm = 0, trees = 5)
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    img1 = cv2.imread("/home/b0nzo/akatsuki_au_bfmc21/nbs/park_ref.png")
    img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blue1 = np.array([90,90,70])
    blue2 = np.array([140,255,255])
    mask1 = cv2.inRange(hsv, blue1, blue2)
    imgRes = cv2.bitwise_and(img, img, mask=mask1)
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
            x, y, w, h = cv2.boundingRect(approx)
            cropped_image = img[y:y+h,x:x+w]
            cropped_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
            kp1, des1 = sift.detectAndCompute(img1, None)
            kp2, des2 = sift.detectAndCompute(cropped_image, None)
            bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)
            # matches = bf.match(des1, des2)
            matches = flann.knnMatch(des1, des2, k=2)
            good_matches = []
            for m, n in matches:
                if m.distance < 0.95 * n.distance:
                    good_matches.append(m)
            if len(good_matches) > 6:
                return True, x, y, w, h
    return False,0,0,0,0


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
    frame = cv2.imread("/home/b0nzo/akatsuki_au_bfmc21/nbs/crosswalk.jpeg")
    # frame = frame[0:np.int32(frame.shape[0]/2), 2*np.int32(frame.shape[1]/3):np.int32(frame.shape[1])]
    
    while True:
        # ret, frame = cap.read()
        # frame = frame[0:np.int32(frame.shape[0]/2), np.int32(frame.shape[1]/2):np.int32(frame.shape[1])]
        out = detect_signs(frame, labels)
        if out:
            box, text, location = out
            # print(text)
            cv2.imshow('',draw_box(frame, text, location, box))
            # time.sleep(0.03)
        else:
            pass
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # print(text)
            break
            
    
