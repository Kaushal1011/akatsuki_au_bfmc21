import cv2
import numpy as np

frameWidth = 640
frameHeight = 480
cap = cv2.VideoCapture(0)
cap.set(3, frameWidth)
cap.set(4, frameHeight)
blue1 = np.array([110,50,50])
blue2 = np.array([130,255,255])
red1 = np.array([170,50,50])
red2 = np.array([180,255,255])
yellow1 = np.array([10,52,50])
yellow2 = np.array([40,255,255])

def empty(a):
    pass


cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters", 640, 240)
cv2.createTrackbar("Thresh1", "Parameters", 105, 255, empty)
cv2.createTrackbar("Thresh2", "Parameters", 50, 255, empty)
cv2.createTrackbar("Area", "Parameters", 5000, 30000, empty)



while True:
    ret, img = cap.read()
    imgContour = img.copy()

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    mask0 = cv2.inRange(hsv_img, red1, red2)
    mask1 = cv2.inRange(hsv_img, blue1, blue2)
    mask2 = cv2.inRange(hsv_img, yellow1, yellow2)

    maskf = mask1

    imgRes = cv2.bitwise_and(img, img, mask=maskf)
    imgBlur = cv2.GaussianBlur(imgRes, (7,7), 1)
    

    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
    Thresh1 = cv2.getTrackbarPos("Thresh1", "Parameters")
    Thresh2 = cv2.getTrackbarPos("Thresh2", "Parameters")

    imgCanny = cv2.Canny(imgGray, Thresh1, Thresh2)
    kernel = np.ones((5,5))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)

    contours, hierarchy = cv2.findContours(imgDil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        areaMin = cv2.getTrackbarPos("Area", "Parameters")
        if area< 50000 and area >1000:    
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.05*peri, True)
            if len(approx)==4 or len(approx)==6:
                cv2.drawContours(imgContour, [approx], -1, (255,0,255), 7)
            # print(len(approx))
                x_, y_, w, h = cv2.boundingRect(approx)
                cv2.rectangle(imgContour, (x_, y_), (x_+w, y_+h), (0,255,0), 5)
                cv2.putText(imgContour,"Points: " + str(len(approx)), (x_ + w + 20, y_ + 20), cv2.FONT_HERSHEY_COMPLEX, .7, (0,255,0), 2)

    cv2.imshow("Result", imgContour)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break