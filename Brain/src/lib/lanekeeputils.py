import cv2
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Any, Union, Tuple
import os
import math
from functools import reduce


class LaneKeep:
    def __init__(
        self,
        use_perspective: bool = True,
        computation_method: str = "hough",
        blur_size: Tuple[int, int] = (7, 7),
        adpt_Th_blk_size: int = 21,
        adpt_Th_C: int = 4,
        canny_thres1: int = 40,
        canny_thres2: int = 60,
        luroi: float = 0.25,
        ruroi: float = 0.75,
        lbroi: float = 0,
        rbroi: float = 1,
        hroi: float = 0.55,
    ):
        """Define LaneKeeping pipeline and parameters

        Args:
            use_perspective: (bool) change preprocess pipeline
                        False -> [roi_func, preprocess]
                        True  -> [persepective_wrap, preprocess]
            computation_method:(str) [ hough | sliding_window]
            blur_size: (Tuple[int, int]) Blur kernel size (preprocess)
            adpt_Th_blk_size: (int) Adaptive thresholding block size (preprocess)
            adpt_Th_C: (int) Adaptive thresholidng parameter C (preprocess)
            canny_thres1: (int) Canny Edge Detection threshold parameter (preprocess)
            canny_thres2: (int) Canny Edge Detection threshold parameter (preprocess)
            luroi: (float) left upper ROI region percent
            ruroi: (float) right upper ROI region percent
            lbroi: (float) left bottom ROI region percent
            rbroi: (float) right bottom ROI region percent
            hroi : (float) ROI region percent
        """

        assert (
            computation_method == "hough" or computation_method == "sliding_window"
        ), f"Expected computation_method to be either 'hough' or 'sliding_window'"

        self.computation_method = computation_method
        self.blur_size = blur_size
        # Size of a pixel neighborhood that is used to calculate a threshold value for the pixel
        self.adpt_Th_blk_size = adpt_Th_blk_size
        # const substracted from mean (adaptive threshold)
        self.adpt_Th_C = adpt_Th_C
        # threshold for canny edge detection
        # https://docs.opencv.org/4.x/dd/d1a/group__imgproc__feature.html#ga04723e007ed888ddf11d9ba04e2232de
        self.canny_thres1 = canny_thres1
        self.canny_thres2 = canny_thres2

        # perspective wrap
        self.luroi = luroi
        self.ruroi = ruroi
        self.lbroi = lbroi
        self.rbroi = rbroi
        self.hroi = hroi
        self.stencil = None
        if use_perspective:
            pipeline_list = [self.persepective_wrap, self.preprocess]
        else:
            pipeline_list = [self.roi_func, self.preprocess]

        def compose(f, g):
            return lambda x: g(f(x))

        self.preprocess_pipeline = reduce(compose, pipeline_list, lambda x: x)

    def __call__(self, img: np.ndarray) -> float:
        if self.computation_method == "sliding_window":
            return self.sliding_window_search(img)
        else:
            return self.houghlines_angle(img)

    def roi_func(self, img: np.ndarray) -> np.ndarray:
        """Given image get Region of interest

        Args:
            img: (np.ndarray) input image

        Returns:
            np.ndarray: the roi image
        """
        # create stencil just the first time and then save for later use
        if self.stencil is None:
            roi = [
                (
                    int(self.luroi * ((img.shape[1] - 1))),
                    int(self.hroi * (img.shape[0] - 1)),
                ),
                (int(self.lbroi * ((img.shape[1] - 1))), int(img.shape[0] - 1)),
                (int(self.rbroi * ((img.shape[1] - 1))), int(img.shape[0] - 1)),
                (
                    int(self.ruroi * ((img.shape[1] - 1))),
                    int(self.hroi * (img.shape[0] - 1)),
                ),
            ]
            self.stencil = np.zeros_like(img, dtype="uint8")
            # specify coordinates of the polygon
            polygon = np.array(roi)

            # fill polygon with ones
            cv2.fillConvexPoly(self.stencil, polygon, [255, 255, 255])

        img = cv2.bitwise_and(img, img, mask=self.stencil[:, :, 0])
        return img

    def preprocess(self, img: np.ndarray) -> np.ndarray:
        """Preprocess image for edge detection"""
        # Apply HLS color filtering to filter out white lane lines
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        lower_white = np.array([90, 90, 90]) # change here if it fails to detect
        upper_white = np.array([255, 255, 255])
        mask = cv2.inRange(img, lower_white, upper_white)
        hls_result = cv2.bitwise_and(img, img, mask=mask)

        # Convert image to grayscale, apply threshold, blur & extract edges
        gray = cv2.cvtColor(hls_result, cv2.COLOR_BGR2GRAY)
        thres = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            self.adpt_Th_blk_size,
            self.adpt_Th_C,
        )
        # ret, thresh = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)
        blur = cv2.GaussianBlur(thres, self.blur_size, 0)
        canny = cv2.Canny(blur, self.canny_thres1, self.canny_thres2)
        return canny
        # return hls_result, gray, thres, blur, canny

    def persepective_wrap(self, img: np.ndarray) -> np.ndarray:
        """ROI of image and Perform perspective wrapping on the image"""
        roi = [
            (self.luroi * ((img.shape[1] - 1)), self.hroi * (img.shape[0] - 1)),
            (self.ruroi * ((img.shape[1] - 1)), self.hroi * (img.shape[0] - 1)),
            (self.lbroi * ((img.shape[1] - 1)), img.shape[0] - 1),
            (self.rbroi * ((img.shape[1] - 1)), img.shape[0] - 1),
        ]
        # Get image size
        img_size = (img.shape[1], img.shape[0])
        # Perspective points to be warped
        src = np.float32(roi)
        # Window to be shown
        dst = np.float32(
            [[0, 0], [img.shape[1], 0], [0, img.shape[0]], [img.shape[1], img.shape[0]]]
        )

        # Matrix to warp the image for birdseye window
        matrix = cv2.getPerspectiveTransform(src, dst)
        # Inverse matrix to unwarp the image for final window
        minv = cv2.getPerspectiveTransform(dst, src)
        birdseye = cv2.warpPerspective(img, matrix, img_size)

        return birdseye
        # # Get the birdseye window dimensions
        # height, width = birdseye.shape[:2]

        # # Divide the birdseye view into 2 halves to separate left & right lanes
        # birdseyeLeft = birdseye[0:height, 0 : width // 2]
        # birdseyeRight = birdseye[0:height, width // 2 : width]

        # # Display birdseye view image
        # # cv2.imshow("Birdseye" , birdseye)
        # # cv2.imshow("Birdseye Left" , birdseyeLeft)
        # # cv2.imshow("Birdseye Right", birdseyeRight)
        # return birdseye, birdseyeLeft, birdseyeRight, minv

    def houghlines_angle(self, img: np.ndarray) -> float:
        """Given processed image compute steering angle"""
        # find lanes takes processed image
        processed_img = self.preprocess_pipeline(img)
        cv2.imwrite("proccessed_img.jpg", processed_img)
        lines = find_lanes(processed_img)
        # average slop takes original image
        lanelines = average_slope_intercept(img, lines)
        return compute_steering_angle_lanelinecoord(img[:, :, 0], lane_lines=lanelines)

    def sliding_window_search(self, img: np.ndarray) -> float:
        """Given processed image compute steering angle"""
        processed_img = self.preprocess_pipeline(img)
        hist, _, _ = plotHistogram(processed_img)
        left_fit, right_fit, lx, rx, out_img = slide_window_search(processed_img, hist)
        return (
            compute_steering_angle_lanelineslope(left_fit, right_fit, invert=True),
            out_img,
        )


def processImage(inpImage):
    """
    function used to preprocess image

    Args:
        inpImage ([np.ndarray]): input image to be process

    Returns:
        Tuple(np.Array): [HLS_THRESHOLDED,Grey Converted, Gray Thresholed, Gaussian Blurred, Canny Edge Detected Image]
    """

    # Apply HLS color filtering to filter out white lane lines
    hls = cv2.cvtColor(inpImage, cv2.COLOR_BGR2HLS)
    lower_white = np.array([180, 180, 180])
    upper_white = np.array([255, 255, 255])
    mask = cv2.inRange(inpImage, lower_white, upper_white)
    hls_result = cv2.bitwise_and(inpImage, inpImage, mask=mask)

    # Convert image to grayscale, apply threshold, blur & extract edges
    gray = cv2.cvtColor(hls_result, cv2.COLOR_BGR2GRAY)
    thresh = cv2.adaptiveThreshold(
        gray,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        21,
        4,
    )
    # ret, thresh = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)
    blur = cv2.GaussianBlur(thresh, (7, 7), 0)
    canny = cv2.Canny(blur, 40, 60)

    # Display the processed images
    # plt.imshow("Image", inpImage)
    # plt.imshow("HLS Filtered", hls_result)
    # plt.imshow("Grayscale", gray)
    # plt.imshow("Thresholded", thresh)
    # plt.imshow("Blurred", blur)
    # plt.imshow("Canny Edges", canny)

    return hls_result, gray, thresh, blur, canny


# roi = [[90, 500],       [800, 500],       [200, 1200],       [1600, 1200]]
def perspectiveWarp(inpImage, luroi=0.25, ruroi=0.75, lbroi=0, rbroi=1, hroi=0.55):
    """
    Returns Perspective wrapped image. This function returns birds eye view of the road using the roi polygon defined

    Args:
        inpImage ([type]): Input image to be processed
        luroi (float, optional): Left Up point (x coord). Defaults to 0.25.
        ruroi (float, optional): Right Up point (x coord). Defaults to 0.75.
        lbroi (int, optional): Left Botton point(x coord). Defaults to 0.
        rbroi (int, optional): Right Botton point(x coord). Defaults to 1.
        hroi (float, optional): Y coord for the polygon (top). Defaults to 0.55.

    Returns:
        Tuple([Np.Array]): [Birds Eye, Birds Eye Left, Birds Eye Right, _ ]
    """

    roi = [
        (luroi * ((inpImage.shape[1] - 1)), hroi * (inpImage.shape[0] - 1)),
        (ruroi * ((inpImage.shape[1] - 1)), hroi * (inpImage.shape[0] - 1)),
        (lbroi * ((inpImage.shape[1] - 1)), inpImage.shape[0] - 1),
        (rbroi * ((inpImage.shape[1] - 1)), inpImage.shape[0] - 1),
    ]
    # roi = [[90, 500],       [800, 500],       [200, 1200],       [1600, 1200]]
    print(roi)
    # Get image size
    img_size = (inpImage.shape[1], inpImage.shape[0])

    # Perspective points to be warped

    src = np.float32(roi)

    # Window to be shown
    dst = np.float32(
        [
            [0, 0],
            [inpImage.shape[1], 0],
            [0, inpImage.shape[0]],
            [inpImage.shape[1], inpImage.shape[0]],
        ]
    )

    # Matrix to warp the image for birdseye window
    matrix = cv2.getPerspectiveTransform(src, dst)
    # Inverse matrix to unwarp the image for final window
    minv = cv2.getPerspectiveTransform(dst, src)
    birdseye = cv2.warpPerspective(inpImage, matrix, img_size)

    # Get the birdseye window dimensions
    height, width = birdseye.shape[:2]

    # Divide the birdseye view into 2 halves to separate left & right lanes
    birdseyeLeft = birdseye[0:height, 0 : width // 2]
    birdseyeRight = birdseye[0:height, width // 2 : width]

    # Display birdseye view image
    # cv2.imshow("Birdseye" , birdseye)
    # cv2.imshow("Birdseye Left" , birdseyeLeft)
    # cv2.imshow("Birdseye Right", birdseyeRight)

    return birdseye, birdseyeLeft, birdseyeRight, minv


def slide_window_search(inpimage, histogram, margin=150):
    """
    returns left and right lane line slope and intercept by employing a sliding window search and Lin Reg
    requires histogram of white pixels

    Args:
        inpimage ([type]): canny edge detected image (roi or perspective warped)
        histogram ([type]): histogram of white pixels
        margin: sliding window marin

    Returns: left fit (slope,intercpt), right fit(slope intercept),left line, right line, explaination of search

    """

    # Find the start of left and right lane lines using histogram info
    out_img = np.dstack((inpimage, inpimage, inpimage)) * 255
    midpoint = np.int(histogram.shape[0] / 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # A total of 10 windows will be used
    nwindows = 10
    window_height = np.int(inpimage.shape[0] / nwindows)
    nonzero = inpimage.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    leftx_current = leftx_base
    rightx_current = rightx_base
    minpix = 50
    left_lane_inds = []
    right_lane_inds = []

    #### START - Loop to iterate through windows and search for lane lines #####
    for window in range(nwindows):
        win_y_low = inpimage.shape[0] - (window + 1) * window_height
        win_y_high = inpimage.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        cv2.rectangle(
            out_img,
            (win_xleft_low, win_y_low),
            (win_xleft_high, win_y_high),
            (0, 255, 0),
            2,
        )
        cv2.rectangle(
            out_img,
            (win_xright_low, win_y_low),
            (win_xright_high, win_y_high),
            (0, 255, 0),
            2,
        )
        good_left_inds = (
            (nonzeroy >= win_y_low)
            & (nonzeroy < win_y_high)
            & (nonzerox >= win_xleft_low)
            & (nonzerox < win_xleft_high)
        ).nonzero()[0]
        good_right_inds = (
            (nonzeroy >= win_y_low)
            & (nonzeroy < win_y_high)
            & (nonzerox >= win_xright_low)
            & (nonzerox < win_xright_high)
        ).nonzero()[0]
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # print(rightx_current)
    #### END - Loop to iterate through windows and search for lane lines #######

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    plt.xlim(0, out_img.shape[1])
    plt.ylim(0, out_img.shape[0])

    ploty = np.linspace(0, inpimage.shape[0] - 1, inpimage.shape[0])
    # Apply 2nd degree polynomial fit to fit curves
    plt.gca().invert_yaxis()
    try:
        left_fit = np.polyfit(lefty, leftx, 1)
        left_fitx = left_fit[0] * ploty + left_fit[1]
        # plt.plot(left_fitx,  ploty, color='yellow')
    except:
        left_fit = []
        left_fitx = []
        # plt.plot(left_fitx,  ploty, color='yellow')
    try:
        right_fit = np.polyfit(righty, rightx, 1)
        right_fitx = right_fit[0] * ploty + right_fit[1]
        # plt.plot(right_fitx, ploty, color='yellow')
    except:
        right_fit = []
        right_fitx = []
        # plt.plot(right_fitx, ploty, color='yellow')

    if left_fit == []:
        left_fit = right_fit
        left_fitx = right_fitx
    if right_fit == []:
        right_fit = left_fit
        right_fitx = left_fitx

    ltx = np.trunc(left_fitx)
    rtx = np.trunc(right_fitx)
    # plt.plot(right_fitx)
    # plt.show()

    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    # plt.imshow(out_img)

    return left_fit, right_fit, ltx, rtx, out_img


def compute_steering_angle_lanelinecoord(frame, lane_lines):
    """Find the steering angle based on lane line coordinate
    We assume that camera is calibrated to point to dead center
    """
    if lane_lines is None or len(lane_lines) == 0:
        # print("No lane lines detected, do nothing")
        return 90

    height, width = frame.shape
    if len(lane_lines) == 1:
        # print("Only detected one lane line, just follow it. %s" % lane_lines[0])
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        camera_mid_offset_percent = 0.02
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height / 2)

    # angle (in radian) to center vertical line
    angle_to_mid_radian = math.atan(x_offset / y_offset)
    # angle (in degrees) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
    # this is the steering angle needed by picar front wheel
    steering_angle = angle_to_mid_deg + 90

    # print("new steering angle: %s" % steering_angle)
    return steering_angle


def compute_steering_angle_lanelineslope(leftline=[], rightline=[], invert=True):

    if invert:
        if len(leftline) == 0 and len(rightline) == 0:
            return 90
        elif len(leftline) == 0:
            return math.degrees(math.atan(-1 / rightline[0])) + 90
        elif len(rightline) == 0:
            return math.degrees(math.atan(-1 / leftline[0])) + 90
        else:
            n = -(leftline[0] + rightline[0])
            d = leftline[0] * rightline[0]
            return math.degrees(math.atan(n / 2 * d)) + 90
    else:
        if len(leftline) == 0 and len(rightline) == 0:
            return 90
        elif len(leftline) == 0:
            return math.degrees(math.atan(rightline[0])) + 90
        elif len(rightline) == 0:
            return math.degrees(math.atan(leftline[0])) + 90
        else:
            return math.degrees(math.atan((leftline[0] + rightline[0]) / 2)) + 90


def roi_func(gimg, luroi=0.25, ruroi=0.75, lbroi=0, rbroi=1, hroi=0.55):
    """
    returns  roi masked image

    Args:
        gimg ([type]): input image
        luroi (float, optional): left up x coord %. Defaults to 0.25.
        ruroi (float, optional): right up x coord %. Defaults to 0.75.
        lbroi (int, optional): left bottom x coord %. Defaults to 0.
        rbroi (int, optional): right bottom x coord %. Defaults to 1.
        hroi (float, optional): height from bottom to top y coord % . Defaults to 0.55.

    Returns:
        [type]: [description]
    """
    roi = [
        (int(luroi * ((gimg.shape[1] - 1))), int(hroi * (gimg.shape[0] - 1))),
        (int(lbroi * ((gimg.shape[1] - 1))), int(gimg.shape[0] - 1)),
        (int(rbroi * ((gimg.shape[1] - 1))), int(gimg.shape[0] - 1)),
        (int(ruroi * ((gimg.shape[1] - 1))), int(hroi * (gimg.shape[0] - 1))),
    ]
    stencil = np.zeros_like(gimg, dtype="uint8")
    # print(roi, stencil_coords)
    # specify coordinates of the polygon
    polygon = np.array(roi)
    # print(polygon)

    # fill polygon with ones
    cv2.fillConvexPoly(stencil, polygon, [255, 255, 255])
    # plt.imshow(stencil[:,:,0])
    print(gimg.shape, stencil.shape)
    img = cv2.bitwise_and(gimg, gimg, mask=stencil[:, :, 0])
    return img


def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame

    if slope == 0:
        slope = 0.01
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]


def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        # print("No line_segment segments detected")
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 2 / 3
    # left lane line segment should be on left 2/3 of the screen
    left_region_boundary = width * (1 - boundary)
    # right lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))
    return lane_lines
    return lane_lines, left_fit_average, right_fit_average


def find_lanes(thresh_canny):
    """
    Takes Threshold Canny Edge Detected image as imput

    Args:
        thresh_canny ([type]): [description]

    Returns:
        [type]: [description]
    """
    lines = cv2.HoughLinesP(thresh_canny, 1, np.pi / 180, 30, maxLineGap=200)
    return lines


def plotHistogram(inpImage):
    """
    Plots Histogram of white lane pixels ( Pixel density on x coordinates)

    Args:
        inpImage ([type]): [description]

    Returns:
        Tuple(List,List,List): histogram, lextxBase, rightxBase
    """

    histogram = np.sum(inpImage[inpImage.shape[0] // 2 :, :], axis=0)

    midpoint = np.int(histogram.shape[0] / 2)
    leftxBase = np.argmax(histogram[:midpoint])
    rightxBase = np.argmax(histogram[midpoint:]) + midpoint

    # plt.xlabel("Image X Coordinates")
    # plt.ylabel("Number of White Pixels")

    # Return histogram and x-coordinates of left & right lanes to calculate
    # lane width in pixels
    return histogram, leftxBase, rightxBase