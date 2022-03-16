import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import mode

directions_map = np.zeros([10, 5])
# cap = cv2.VideoCapture("optical_flow_det.mp4")
cap = cv2.VideoCapture(0)

ret, first_frame = cap.read()
print(len(first_frame))
prev_gray = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)
hsv = np.zeros_like(first_frame)
hsv[:, :, 1] = 255
param = {
        'pyr_scale': 0.5,  # Image scale (<1) to build pyramids for each image
        'levels': 5,  # Number of pyramid layers
        'winsize': 25,  # Averaging window size
        'iterations': 5,  # Number of iterations the algorithm does at each pyramid level
        'poly_n': 7,  # Size of the pixel neighborhood used to find polynomial expansion in each pixel
        'poly_sigma': 1.5,  # Standard deviation of the Gaussian that is used to smooth derivatives used as a basis for the polynomial expansion
        'flags': cv2.OPTFLOW_LK_GET_MIN_EIGENVALS
    }

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # flow = cv2.calcOpticalFlowFarneback(prev_gray, gray, None, **param)
    optical_flow = cv2.optflow.DualTVL1OpticalFlow_create()
    flow = optical_flow.calc(prev_gray, gray, None)
    mag, ang = cv2.cartToPolar(
        flow[:, :, 0], flow[:, :, 1], angleInDegrees=True)
    ang_180 = ang/2
    gray_previous = gray.copy()

    move_sense = ang[mag > 10]
    move_mode = mode(move_sense)[0]

    if 10 < move_mode <= 100:
        directions_map[-1, 0] = 1
        directions_map[-1, 1:] = 0
        directions_map = np.roll(directions_map, -1, axis=0)
    elif 100 < move_mode <= 190:
        directions_map[-1, 1] = 1
        directions_map[-1, :1] = 0
        directions_map[-1, 2:] = 0
        directions_map = np.roll(directions_map, -1, axis=0)
    elif 190 < move_mode <= 280:
        directions_map[-1, 2] = 1
        directions_map[-1, :2] = 0
        directions_map[-1, 3:] = 0
        directions_map = np.roll(directions_map, -1, axis=0)
    elif 280 < move_mode or move_mode < 10:
        directions_map[-1, 3] = 1
        directions_map[-1, :3] = 0
        directions_map[-1, 4:] = 0
        directions_map = np.roll(directions_map, -1, axis=0)
    else:
        directions_map[-1, -1] = 1
        directions_map[-1, :-1] = 0
        directions_map = np.roll(directions_map, 1, axis=0)

    loc = directions_map.mean(axis=0).argmax()
    if loc == 0:
        text = 'Moving down'
    elif loc == 1:
        text = 'Moving to the right'
    elif loc == 2:
        text = 'Moving up'
    elif loc == 3:
        text = 'Moving to the left'
    else:
        text = 'WAITING'

    hsv[:, :, 0] = ang_180
    hsv[:, :, 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
    rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    frame = cv2.flip(frame, 1)
    cv2.putText(frame, text, (30, 90), cv2.FONT_HERSHEY_COMPLEX, frame.shape[1] / 500, (0, 0, 255), 2)

    k = cv2.waitKey(1) & 0xff
    if k == ord('q'):
        break
    cv2.imshow('Mask', rgb)
    cv2.imshow('Frame', frame)
    k = cv2.waitKey(1) & 0xff
    if k == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()



