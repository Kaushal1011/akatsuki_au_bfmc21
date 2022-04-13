import cv2
from cv2 import blur
import numpy as np
from functools import partial
import time
from typing import Tuple


def check_priority(img, area_threshold: Tuple[int, int]):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    yellow1 = np.array([18,200,70])
    yellow2 = np.array([28,255,100])
    mask = cv2.inRange(hsv, yellow1, yellow2)
    imgRes = cv2.bitwise_and(img, img, mask=mask)
    blur = cv2.GaussianBlur(imgRes, (7,7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_RGB2GRAY)
    canny = cv2.Canny(gray, 55, 30)
    kernel = np.ones((7,7))
    dilate = cv2.dilate(canny, kernel, iterations=1)
    hull = []
    contours, _ = cv2.findContours(
        dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        hull.append(cv2.convexHull(cnt, False))
        area = cv2.contourArea(cnt)
        if area > area_threshold[0] and area < area_threshold[1]:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.1 * peri, True)
             
            x, y, w, h = cv2.boundingRect(approx)
            return True, x, y, w, h
    return False,0,0,0,0

def check_stop(img, area_threshold: Tuple[int, int]):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV_FULL)
    red1 = np.array([0,70,70])
    red2 = np.array([10,255,255])
    red3 = np.array([350,70,70])
    red4 = np.array([360,255,255])
    mask1 = cv2.inRange(hsv, red1, red2)
    mask2 = cv2.inRange(hsv, red3, red4)
    mask = cv2.bitwise_or(mask1, mask2)
    imgRes = cv2.bitwise_and(img, img, mask=mask)
    blur = cv2.GaussianBlur(imgRes, (7,7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_RGB2GRAY)
    canny = cv2.Canny(gray, 55, 35)
    kernel = np.ones((7,7))
    dilate = cv2.dilate(canny, kernel, iterations=1)
    hull = []
    contours, _ = cv2.findContours(
        dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        hull.append(cv2.convexHull(cnt, False))
        area = cv2.contourArea(cnt)
        if area > area_threshold[0] and area < area_threshold[1]:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.2 * peri, True)
             
            x, y, w, h = cv2.boundingRect(approx)
            return True, x, y, w, h
    return False,0,0,0,0

def check_no_entry(img, area_threshold: Tuple[int, int]):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV_FULL)
    red1 = np.array([0,30,30])
    red2 = np.array([20,255,255])
    red3 = np.array([340,30,30])
    red4 = np.array([360,255,255])
    white1 = np.array([0,0,252])
    white2 = np.array([360,20,255])
    mask1 = cv2.inRange(hsv, red1, red2)
    mask2 = cv2.inRange(hsv, red3, red4)
    mask = cv2.bitwise_or(mask1, mask2)
    imgRes = cv2.bitwise_and(img, img, mask=mask)
    blur = cv2.GaussianBlur(imgRes, (7,7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 55, 35)
    kernel = np.ones((7,7))
    dilate = cv2.dilate(canny, kernel, iterations=1)
    hull = []
    contours, _ = cv2.findContours(
        dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        hull.append(cv2.convexHull(cnt, False))
        area = cv2.contourArea(cnt)
        print(area)
        if area > area_threshold[0] and area < area_threshold[1]:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.2 * peri, True)
            x, y, w, h = cv2.boundingRect(approx)

            cropped_hsv = hsv[y:y+h,x:x+w]
            cropped_image = img[y:y+h,x:x+w]
            mask = cv2.inRange(cropped_hsv, white1, white2)
            croppedRes = cv2.bitwise_and(cropped_image, cropped_image, mask=mask)
            cropped_gray = cv2.cvtColor(croppedRes, cv2.COLOR_BGR2GRAY)
            cropped_canny = cv2.Canny(cropped_gray, 55, 35)
            kernel = np.ones((7,7))
            cropped_dilate = cv2.dilate(cropped_canny, kernel, iterations=1)
            contors, _ = cv2.findContours(
                cropped_dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            for cnt1 in contors:
                area1 = cv2.contourArea(cnt1)
                if area1>1000 and area1/area>=0.1:
                    return True, x, y, w, h
    return False,0,0,0,0

def check_Highway(img, area_threshold: Tuple[int, int]):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV_FULL)
    green1 = np.array([135,30,35])
    green2 = np.array([165,255,255])
    mask = cv2.inRange(hsv, green1, green2)
    imgRes = cv2.bitwise_and(img, img, mask=mask)
    blur = cv2.GaussianBlur(imgRes, (7,7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 55, 30)
    kernel = np.ones((7,7))
    dilate = cv2.dilate(canny, kernel, iterations=1)
    # hull = []
    contours, _ = cv2.findContours(
        dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        # hull.append(cv2.convexHull(cnt, False))
        area = cv2.contourArea(cnt)
        if area > area_threshold[0] and area < area_threshold[1]:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.1 * peri, True)
            # cv2.drawContours(imgContour, hull, -1, (255, 0, 255), 8)
            x, y, w, h = cv2.boundingRect(approx)
            
            return True, x, y, w, h
    return False,0,0,0,0

def check_highway_no(img, area_threshold: Tuple[int, int]):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV_FULL)
    green1 = np.array([100,30,35])
    green2 = np.array([170,255,255])
    red1 = np.array([0,30,35])
    red2 = np.array([15,255,255])
    red3 = np.array([345,30,35])
    red4 = np.array([360,255,255])
    mask0 = cv2.inRange(hsv, green1, green2)
    imgRes = cv2.bitwise_and(img, img, mask=mask0)
    blur = cv2.GaussianBlur(imgRes, (7,7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 55, 30)
    kernel = np.ones((7,7))
    dilate = cv2.dilate(canny, kernel, iterations=1)
    hull = []
    contours, _ = cv2.findContours(
        dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        hull.append(cv2.convexHull(cnt, False))
        area = cv2.contourArea(cnt)
        if area > area_threshold[0] and area < area_threshold[1]:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.1 * peri, True)
            x, y, w, h = cv2.boundingRect(approx)

            cropped_hsv = hsv[y:y+h,x:x+w]
            cropped_image = img[y:y+h,x:x+w]
            mask1 = cv2.inRange(cropped_hsv, red1, red2)
            mask2 = cv2.inRange(cropped_hsv, red3, red4)
            mask = cv2.bitwise_or(mask1, mask2)
            croppedRes = cv2.bitwise_and(cropped_image, cropped_image, mask=mask)
            cropped_gray = cv2.cvtColor(croppedRes, cv2.COLOR_BGR2GRAY)
            contors, _ = cv2.findContours(
                cropped_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            for cnt1 in contors:
                area1 = cv2.contourArea(cnt1)
                if area1 > 7:
                    return True, x, y, w, h
    return False,0,0,0,0

def check_oneway(img, area_threshold: Tuple[int, int]):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    blue1 = np.array([90,90,70])
    blue2 = np.array([140,255,255])
    white1 = np.array([0,0,250])
    white2 = np.array([360,255,255])
    mask1 = cv2.inRange(hsv, blue1, blue2)
    imgRes = cv2.bitwise_and(img, img, mask=mask1)
    blur = cv2.GaussianBlur(imgRes, (7,7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_RGB2GRAY)
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
             
            x, y, w, h = cv2.boundingRect(approx)

            cropped_hsv = hsv[y:y+h,x:x+w]
            cropped_image = img[y:y+h,x:x+w]
            mask = cv2.inRange(cropped_hsv, white1, white2)
            croppedRes = cv2.bitwise_and(cropped_image, cropped_image, mask=mask)
            cropped_gray = cv2.cvtColor(croppedRes, cv2.COLOR_RGB2GRAY)
            contors, _ = cv2.findContours(
                cropped_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            for cnt in contors:
                area1 = cv2.contourArea(cnt)
                if area1 > 800:
                    return True, x, y, w, h
    return False,0,0,0,0


def check_cross(img, area_threshold: Tuple[int, int]):
    sift = cv2.SIFT_create()
    index_params = dict(algorithm = 0, trees = 5)
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    img1 = cv2.imread("/home/b0nzo/akatsuki_au_bfmc21/nbs/cross_ref.png")
    img1 = cv2.cvtColor(img1, cv2.COLOR_RGB2GRAY)
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    blue1 = np.array([90,90,70])
    blue2 = np.array([140,255,255])
    mask1 = cv2.inRange(hsv, blue1, blue2)
    imgRes = cv2.bitwise_and(img, img, mask=mask1)
    blur = cv2.GaussianBlur(imgRes, (7,7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_RGB2GRAY)
    canny = cv2.Canny(gray, 55, 30)
    kernel = np.ones((7,7))
    dilate = cv2.dilate(canny, kernel, iterations=1)
    hull = []
    contours, _ = cv2.findContours(
        dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        hull.append(cv2.convexHull(cnt, False))
        area = cv2.contourArea(cnt)
        if area > area_threshold[0] and area < area_threshold[1]:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.1 * peri, True)
            x, y, w, h = cv2.boundingRect(approx)
            cropped_image = img[y:y+h,x:x+w]
            cropped_image = cv2.cvtColor(cropped_image, cv2.COLOR_RGB2GRAY)
            _, des1 = sift.detectAndCompute(img1, None)
            _, des2 = sift.detectAndCompute(cropped_image, None)
            # bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)
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
    img1 = cv2.cvtColor(img1, cv2.COLOR_RGB2GRAY)
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    blue1 = np.array([90,90,70])
    blue2 = np.array([140,255,255])
    mask1 = cv2.inRange(hsv, blue1, blue2)
    imgRes = cv2.bitwise_and(img, img, mask=mask1)
    blur = cv2.GaussianBlur(imgRes, (7,7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_RGB2GRAY)
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
            cropped_image = cv2.cvtColor(cropped_image, cv2.COLOR_RGB2GRAY)
            _, des1 = sift.detectAndCompute(img1, None)
            _, des2 = sift.detectAndCompute(cropped_image, None)
            # bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)
            # matches = bf.match(des1, des2)
            matches = flann.knnMatch(des1, des2, k=2)
            good_matches = []
            for m, n in matches:
                if m.distance < 0.95 * n.distance:
                    good_matches.append(m)
            if len(good_matches) > 6:
                return True, x, y, w, h
    return False,0,0,0,0


def detections(img, label):
    text = "not detected"
    cs,csx,csy,csw,csh=check_stop(img,(1000,3500))
    cne,cnex,cney,cnew,cneh=check_no_entry(img,(1000,20000))
    cp,cpx,cpy,cpw,cph=check_priority(img,(700,25000))
    chn,chnx,chny,chnw,chnh=check_highway_no(img,(700,25000))
    ch,chx,chy,chw,chh=check_Highway(img,(1500,25000))
    cow,cowx,cowy,coww,cowh=check_oneway(img,(2000,25000))
    cpa,cpax,cpay,cpaw,cpah=check_park(img,(2000,25000))
    cc,ccx,ccy,ccw,cch=check_cross(img,(2000,25000))
    box,text,location=None,None,None
    
    if False:
        box = [(chnx, chny), (chnx + chnw, chny + chnh)]
        location = chnx, chny
        text = label[3]
    elif False:
        box = [(cnex, cney), (cnex + cnew, cney + cneh)]
        location = cnex, cney
        text = label[0]   
    elif False:
        box = [(csx, csy), (csx + csw, csy + csh)]
        location = csx, csy
        text = label[1] 
    elif False:
        box = [(chx, chy), (chx + chw, chy + chh)]
        location = chx, chy
        text = label[4]
    elif False:
        box = [(cpax, cpay), (cpax + cpaw, cpay + cpah)]
        location = cpax, cpay
        text = label[6]
    elif False:
        box = [(ccx, ccy), (ccx + ccw, ccy + cch)]
        location = ccx, ccy
        text = label[7]
    elif False:
        box = [(cowx, cowy), (cowx + coww, cowy + cowh)]
        location = cowx, cowy
        text = label[5]
    elif False:
        box = [(cpx, cpy), (cpx + cpw, cpy + cph)]
        location = cpx, cpy
        text = label[2]
    else:
        box = [(0,0), (0, 0)]
        location = 0, 0
        text = "no sign"

    return box, text, location


def detect_signs(image,model,labels):
    return model(image,labels)        


def setup():
    print("Starting pseudo sign detection")
    detect_fn = detections
    my_list = ["no entry", "stop", "priority", "highend", "highstart", "one way", "parking", "crosswalk"]

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
    frame = cv2.imread("/home/b0nzo/akatsuki_au_bfmc21/nbs/crosswalk.jpeg")
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
        
