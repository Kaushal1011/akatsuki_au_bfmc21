import numpy as np
import cv2

green = np.uint8([[[12,95,195]]]) #here insert the bgr values which you want to convert to hsv
hsvGreen = cv2.cvtColor(green, cv2.COLOR_BGR2HSV)
print(hsvGreen)
# 
lowerLimit = hsvGreen[0][0][0] - 10, 100, 100
upperLimit = hsvGreen[0][0][0] + 10, 255, 255
# 
print(upperLimit)

print(lowerLimit)

# img1 = cv2.imread("/home/b0nzo/Desktop/cross_gray.png")
# cv2.imshow('Image',img1)
