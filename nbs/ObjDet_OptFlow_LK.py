import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import mode

# input_video_path = '/home/b0nzo/Documents/akatsuki_au_bfmc21/nbs/optical_flow_det.mp4'
# cap = cv2.VideoCapture(input_video_path)
# cap = cv2.VideoCapture("optical_flow_det.mp4")
cap = cv2.VideoCapture(0)

# params for shi-tsomasi corner detection
feature_params = dict(maxCorners=1000, qualityLevel=0.05,
                      minDistance=10)
# params for lukas-konade sparse optical flow
lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(
    cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

flow_color = (0, 255, 0)
ret, first_frame = cap.read()
print(len(first_frame))
prev_gray = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)
prev = cv2.goodFeaturesToTrack(prev_gray, mask = None, **feature_params)
mask = np.zeros_like(first_frame)

while(cap.isOpened()):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    next, status, error = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev, None, **lk_params)
    good_old = prev[status == 1]
    good_new = next[status == 1]
    for i, (new, old) in enumerate(zip(good_new, good_old)):
        # Returns a contiguous flattened array as (x, y) coordinates for new point
        a, b = new.ravel()
        # Returns a contiguous flattened array as (x, y) coordinates for old point
        c, d = old.ravel()
        # Draws line between new and old position with green color and 2 thickness
        mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), flow_color, 2)
        # Draws filled circle (thickness of -1) at new position with green color and radius of 3
        frame = cv2.circle(frame, (int(a), int(b)), 3, flow_color, -1)
    
    output = cv2.add(frame, mask)
    prev_gray = gray.copy()
    cv2.imshow("sparse optical flow", output)
    # Frames are read by intervals of 10 milliseconds. The programs breaks out of the while loop when the user presses the 'q' key
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break