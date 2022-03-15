import cv2
import matplotlib.pyplot as plt
img=cv2.imread("Test_track.png")
print(img.shape)
plt.figure(figsize=(20,20))
plt.imshow(img)
plt.show()