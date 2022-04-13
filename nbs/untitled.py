import cv2

im=cv2.imread("lane.png")
im=cv2.resize(im,None,fx=0.5,fy=0.5)
cv2.imshow("i",im)
cv2.waitKey(1000000)