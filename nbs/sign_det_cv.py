import cv2
from cv2 import imread
from matplotlib.pyplot import hsv
import numpy as np

frameWidth = 640
frameHeight = 480
cap = cv2.VideoCapture(0)
cap.set(3, frameWidth)
cap.set(4, frameHeight)
blue1 = np.array([90,90,70])
blue2 = np.array([140,255,255])
white1 = np.array([130,254,254])
white2 = np.array([131,255,255])
black1 = np.array([0,0,0])
black2 = np.array([180,20,20])
lower_red1 = np.array([0,90,35])
higher_red1 = np.array([5,255,255])
yellow1 = np.array([22,244,78])
yellow2 = np.array([24,246,80])

def empty(a):
    pass


cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters", 640, 240)
cv2.createTrackbar("Thresh1", "Parameters", 5, 255, empty)
cv2.createTrackbar("Thresh2", "Parameters", 35, 255, empty)
cv2.createTrackbar("Thresh3", "Parameters", 0, 255, empty)
cv2.createTrackbar("Thresh4", "Parameters", 26, 255, empty)
# cv2.createTrackbar("Area", "Parameters", 0, 1000, empty)

path = "/home/b0nzo/akatsuki_au_bfmc21/nbs/crosswalk.jpeg"
img = cv2.imread(path)
img = img[0:np.int32(img.shape[0]/2), np.int32(img.shape[1]/2):np.int32(img.shape[1])]
hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# cv2.imshow('ii',hsv_img)

while True:
    
    # ret, img = cap.read()
    # hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    imgContour = img.copy()
    
    masks = [
        cv2.inRange(hsv_img, yellow1,yellow2),
        cv2.inRange(hsv_img, blue1, blue2),
        cv2.inRange(hsv_img, lower_red1, higher_red1)
        
        # cv2.inRange(hsv_img, lowtler_red2, higher_red2)
    ]

    maskf = cv2.bitwise_or(masks[0], masks[1])
    maskf = cv2.bitwise_or(maskf, masks[2])

    imgRes = cv2.bitwise_and(img, img, mask=maskf)
    imgBlur = cv2.GaussianBlur(imgRes, (7,7), 1)
    

    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
    Thresh1 = cv2.getTrackbarPos("Thresh1", "Parameters")
    Thresh2 = cv2.getTrackbarPos("Thresh2", "Parameters")
    Thresh3 = cv2.getTrackbarPos("Thresh3", "Parameters")
    Thresh4 = cv2.getTrackbarPos("Thresh4", "Parameters")

    imgCanny = cv2.Canny(imgGray, Thresh1, Thresh2)
    
    kernel = np.ones((5,5))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
    # cv2.imshow('ii',imgDil)
    contours, hierarchy = cv2.findContours(imgDil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    hull = []
    for cnt in contours:
        hull.append(cv2.convexHull(cnt, False))
        area = cv2.contourArea(cnt)
        if area< 50000 and area >1200:    
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.1*peri, True)
            cv2.drawContours(imgContour, hull, -1, (255,0,255), 8)
            x, y, w, h = cv2.boundingRect(approx)

            cropped_contour = img[y:y+h,x:x+w]
            maskb = cv2.inRange(cropped_contour, black1, black2)
            croppedRes = cv2.bitwise_and(cropped_contour, cropped_contour, mask=maskb)
            croppedGray = cv2.cvtColor(croppedRes, cv2.COLOR_BGR2GRAY)
            croppedBlur = cv2.GaussianBlur(croppedGray, (7,7), 2)
            croppedCanny = cv2.Canny(croppedBlur, 27, 0)
            kernel = np.ones((7,7))
            croppedDil = cv2.dilate(croppedCanny, kernel, iterations=1)
            cv2.imshow("iiii", croppedCanny)
            contours_crop, hierarchy = cv2.findContours(croppedDil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            for cnt in contours_crop:
                cropped_area = cv2.contourArea(cnt)
                print(cropped_area)
            if len(contours_crop) != 0:
                print("Crosswalk")
            cv2.rectangle(imgContour, (x-2, y-2), (x+w-4, y+h-4), (0,255,0), 5)
            cv2.putText(imgContour,"Points: " + str(len(hull)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX, .4, (0,255,0), 1)
    # cv2.imshow('iii', croppedRes)
    cv2.imshow("Result", imgContour)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break