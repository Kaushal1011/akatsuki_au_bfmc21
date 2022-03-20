import cv2
import numpy as np
from typing import Tuple
import time


def empty(a):
    pass

def check_stop(img, area_threshold: Tuple[int, int]):
    imgContour = img.copy()
    
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
#     red1 = np.array([0,70,95])
#     red2 = np.array([10,255,135])
    red3 = np.array([170,100,95])
    red4 = np.array([190,180,130])
#     mask1 = cv2.inRange(hsv, red1, red2)
    mask2 = cv2.inRange(hsv, red3, red4)
    # mask = cv2.bitwise_or(mask1, mask2)
    
#     cv2.waitKey(1)
    imgRes = cv2.bitwise_and(img, img, mask=mask2)
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
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    yellow1 = np.array([85,180,100])
    yellow2 = np.array([110,190,200])
    mask = cv2.inRange(hsv, yellow1, yellow2)
    # print("value yell",hsv[93][225])
#     cv2.imshow("ii",mask)
#     cv2.waitKey(5000)
    imgRes = cv2.bitwise_and(img, img, mask=mask)
    blur = cv2.GaussianBlur(imgRes, (7,7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 55, 30)
    kernel = np.ones((7,7))
    dilate = cv2.dilate(canny, kernel, iterations=1)
    hull = []
    contours, hierarchy = cv2.findContours(
        dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#     print()
    
    for cnt in contours:
        hull.append(cv2.convexHull(cnt, False))
        area = cv2.contourArea(cnt)
        print("priority: ",area)
        if area > area_threshold[0] and area < area_threshold[1]:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.1 * peri, True)
            cv2.drawContours(imgContour, hull, -1, (255, 0, 255), 8)
            x, y, w, h = cv2.boundingRect(approx)
            
            return True, x, y, w, h
    return None,None,None,None,None

def check_cross(img, area_threshold: Tuple[int, int]):
    imgContour = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blue1 = np.array([90,90,70])
    blue2 = np.array([140,255,255])
    mask1 = cv2.inRange(hsv, blue1, blue2)
    imgRes = cv2.bitwise_and(img, img, mask=mask1)
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
            # maskb = cv2.inRange(cropped_contour, black1, black2)
            # crop_res = cv2.bitwise_and(cropped_contour, cropped_contour, mask=maskb)
            # croppedBlur = cv2.GaussianBlur(crop_res, (7,7), 1)
            # croppedGray = cv2.cvtColor(croppedBlur, cv2.COLOR_BGR2GRAY)
            # croppedCanny = cv2.Canny(croppedGray, 26, 0)
            # kernel = np.ones((7,7))
            # croppedDil = cv2.dilate(croppedCanny, kernel, iterations=1)
            # contours_crop, hierarchy = cv2.findContours(croppedDil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if black_pix > 50:   
                return True, x, y, w, h
    return None,None,None,None,None

def check_park(img, area_threshold: Tuple[int, int]):
    imgContour = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blue1 = np.array([90,90,70])
    blue2 = np.array([140,255,255])
    mask1 = cv2.inRange(hsv, blue1, blue2)
    imgRes = cv2.bitwise_and(img, img, mask=mask1)
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
            # maskb = cv2.inRange(cropped_contour, black1, black2)
            # crop_res = cv2.bitwise_and(cropped_contour, cropped_contour, mask=maskb)
            # croppedBlur = cv2.GaussianBlur(crop_res, (7,7), 1)
            # croppedGray = cv2.cvtColor(croppedBlur, cv2.COLOR_BGR2GRAY)
            # croppedCanny = cv2.Canny(croppedGray, 26, 0)
            # kernel = np.ones((7,7))
            # croppedDil = cv2.dilate(croppedCanny, kernel, iterations=1)
            # contours_crop, hierarchy = cv2.findContours(croppedDil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if black_pix <= 50:   
                return True, x, y, w, h
    return None,None,None,None,None




def detection(img, label):
    text = "not detected"
    cs,csx,csy,csw,csh=check_stop(img,(700,25000))
    cp,cpx,cpy,cpw,cph=check_priority(img,(100,25000))
#     cpa,cpax,cpay,cpaw,cpah=check_park(img,(700,25000))
#     cc,ccx,ccy,ccw,cch=check_cross(img,(700,25000))
    box,text,location=None,None,None
    if cs:
        box = [(csx, csy), (csx + csw, csy + csh)]
        location = csx, csy
        text = label[0]
    elif cp:
        box = [(cpx, cpy), (cpx + cpw, cpy + cph)]
        location = cpx, cpy
        text = label[3]
#     elif cc:
#         box = [(ccx, ccy), (ccx + ccw, ccy + cch)]
#         location = ccx, ccy
#         text = label[2]
#     elif cpa:
#         box = [(cpax, cpay), (cpax + cpaw, cpay + cpah)]
#         location = cpax, cpay
#         text = label[1]
    else:
        box = [(0,0), (0, 0)]
        location = 0, 0
        text = "no sign"

    return box, text, location

def detect_signs(img,model,labels):
    return model(img,labels)

        


def setup():
    print("Starting pseudo sign detection")
    detect_fn = detection
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
    frame = cv2.imread("/home/b0nzo/akatsuki_au_bfmc21/nbs/cross.png")
    frame = frame[0:np.int32(frame.shape[0]/2), 2*np.int32(frame.shape[1]/3):np.int32(frame.shape[1])]
    
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
            
    