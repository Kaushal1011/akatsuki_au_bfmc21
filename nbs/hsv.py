import numpy as np
import cv2

green = np.uint8([[[12,194,160]]]) #here insert the bgr values which you want to convert to hsv
hsvGreen = cv2.cvtColor(green, cv2.COLOR_BGR2HSV)
print(hsvGreen)
# 
lowerLimit = hsvGreen[0][0][0] - 10, 100, 100
upperLimit = hsvGreen[0][0][0] + 10, 255, 255
# 
print(upperLimit)

print(lowerLimit)

img1 = cv2.imread("/home/b0nzo/Desktop/highway_enter.png")
hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)

# while True:
#     cv2.imshow("Image",img1)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#             # print(text)
#             break
