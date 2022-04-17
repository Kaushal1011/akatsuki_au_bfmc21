import cv2
import numpy as np


def roi_func(img: np.ndarray) -> np.ndarray:
    """Given image get Region of interest

    Args:
        img: (np.ndarray) input image

    Returns:
        np.ndarray: the roi image
    """
    # create stencil just the first time and then save for later use
    roi = [
        (
            int(0.1 * ((img.shape[1] - 1))),
            int(0.45 * (img.shape[0] - 1)),
        ),
        (int(0 * ((img.shape[1] - 1))), int(img.shape[0] - 1)),
        (int(1 * ((img.shape[1] - 1))), int(img.shape[0] - 1)),
        (
            int(0.9 * ((img.shape[1] - 1))),
            int(0.45 * (img.shape[0] - 1)),
        ),
    ]
    stencil = np.zeros_like(img, dtype="uint8")
    # specify coordinates of the polygon
    polygon = np.array(roi)
    # fill polygon with ones
    cv2.fillConvexPoly(stencil, polygon, [255, 255, 255])
    # plt.imshow(stencil)

    img = cv2.bitwise_and(img, img, mask=stencil[:, :])
    return img


def intersection_det(img, area_threshold=7_500) -> np.ndarray:
    # preprocess
    # lower_white = np.array([90, 90, 90])
    # upper_white = np.array([255, 255, 255])
    # mask = cv2.inRange(img, lower_white, upper_white)

    # # Convert image to grayscale, apply threshold, blur & extract edges
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    result = img.copy()
    _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    blur = cv2.GaussianBlur(thresh, (7, 7), 0)
    roi = roi_func(blur)

    # detect horizontal lines
    horizontal_size = blur.shape[1] // 4
    horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (horizontal_size, 10))
    detect_horizontal = cv2.morphologyEx(
        roi, cv2.MORPH_OPEN, horizontal_kernel, iterations=2
    )
    cnts = cv2.findContours(
        detect_horizontal, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    detected = False
    final_contours = []
    
    for c in cnts:
        area = cv2.contourArea(c)
        print(f"Intersection Area -> {area}")
        if area > area_threshold:
            detected = True
            # final_contours.append(c)
            cv2.drawContours(result, [c], -1, (255, 0, 0), 5)

    return detected, result
