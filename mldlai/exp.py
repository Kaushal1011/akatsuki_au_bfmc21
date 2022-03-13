import cv2
import matplotlib.pyplot as plt

img=cv2.imread("detected.png")
width=img.shape[1]
height=img.shape[0]
img=img[:int(height/2),int(3*width/4):]
plt.imshow(img)
plt.show()