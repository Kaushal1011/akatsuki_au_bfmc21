import math
from functools import reduce
from multiprocessing.sharedctypes import Value
from time import time
from typing import List, Optional, Tuple
from xml.dom import ValidationErr

import cv2
import matplotlib.pyplot as plt
import numpy as np

from src.lib.perception.graph_func import bfs_4


def draw_line(img, lines):
    # create a copy of the original frame
    try:
        dmy = img
        # draw Hough lines
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(dmy, (x1, y1), (x2, y2), (255, 0, 0), 3)

        return dmy
    except Exception:
        return img


def display_heading_line(
    frame,
    steering_angle,
    line_color=(0, 0, 255),
    line_width=5,
):
    heading_image = np.zeros_like(frame)
    if len(frame.shape) == 3:
        height, width, _ = frame.shape
    else:
        height, width = frame.shape
    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image


class LaneKeep:
    def __init__(
        self,
        use_perspective: bool = True,
        hls_lower: List[int] = [90, 90, 90],
        computation_method: str = "hough",
        blur_size: Tuple[int, int] = (7, 7),
        adpt_Th_blk_size: int = 21,
        adpt_Th_C: int = 4,
        canny_thres1: int = 50,
        canny_thres2: int = 150,
        luroi: float = 0.2,
        ruroi: float = 0.8,
        lbroi: float = 0,
        rbroi: float = 1,
        hroi: float = 0.6,
        broi: float = 0,
    ):
        """Define LaneKeeping pipeline and parameters

        Args:
            use_perspective: (bool) change preprocess pipeline
                        False -> [roi_func, preprocess]
                        True  -> [persepective_wrap, preprocess]
            hls_lower: (list) change the hls filter lower bound
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
            broi: (float) ROI region percent from bottom,
        """

        assert (
            computation_method == "hough" or computation_method == "sliding_window"
        ), "Expected computation_method to be either 'hough' or 'sliding_window'"

        assert (
            len(hls_lower) == 3
        ), f"Expected a list of [hue, sat, light], got {hls_lower}"

        assert (
            max(hls_lower) < 255 and min(hls_lower) >= 0
        ), f"Value range for hls is 0~255, got {hls_lower}"

        self.computation_method = computation_method
        self.hls_lower = hls_lower
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
        self.broi = broi
        self.stencil = None
        if use_perspective:
            pipeline_list = [self.persepective_wrap, self.preprocess]
        else:
            pipeline_list = [self.roi_func, self.preprocess]

        def compose(f, g):
            return lambda x: g(f(x))

        self.preprocess_pipeline = reduce(compose, pipeline_list, lambda x: x)

    def __call__(
        self, img: np.ndarray, get_image: bool = False
    ) -> Tuple[float, bool, Optional[np.ndarray]]:
        preprocess_img: np.ndarray = self.preprocess_pipeline(img)
        intersection_detected, cnts = self.intersection_det(preprocess_img)
        mask = np.ones(preprocess_img.shape, dtype="uint8") * 255
        if len(cnts) > 0:
            # print(type(cnts), type(cnts[0]))
            cv2.drawContours(mask, cnts, -1, 0, -1)
        preprocess_img = cv2.bitwise_and(preprocess_img, preprocess_img, mask=mask)

        if self.computation_method == "hough":
            if get_image:
                angle, outimg = self.houghlines_angle(preprocess_img, get_img=get_image)
                print("LK Out angle", angle)
                # if len(cnts) > 0:
                #     self.draw_intersection_bbox(outimg, cnts)
                return angle, intersection_detected, outimg

            else:
                angle = self.houghlines_angle(preprocess_img)
                angle = self.get_lane_error(preprocess_img)
                print("LK Out angle", angle)
                # angle_roadarea = self.graph_road_search(preprocess_img)
                # print(angle, " ", angle_roadarea)
                #             angle = (angle*2 + angle_roadarea) / 3
                return angle, intersection_detected

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
                (
                    int(self.lbroi * ((img.shape[1] - 1))),
                    int((img.shape[0] - 1) * (1 - self.broi)),
                ),
                (
                    int(self.rbroi * ((img.shape[1] - 1))),
                    int((img.shape[0] - 1) * (1 - self.broi)),
                ),
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
        imgn = img.copy()
        imgn = cv2.GaussianBlur(imgn, (17, 17), 0)
        imgn = cv2.cvtColor(imgn, cv2.COLOR_RGB2HSV)

        lower = np.array([0, 0, 249], np.uint8)
        upper = np.array([90, 90, 255], np.uint8)

        mask = cv2.inRange(imgn, lower, upper)
        return mask

    def intersection_det(self, img: np.ndarray, area_threshold=6_500):
        # detect horizontal lines
        horizontal_size = img.shape[1] // 4
        horizontal_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (horizontal_size, 10)
        )
        detect_horizontal = cv2.morphologyEx(
            img, cv2.MORPH_OPEN, horizontal_kernel, iterations=2
        )
        cnts = cv2.findContours(
            detect_horizontal, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        detected = False
        final_contours = []

        for c in cnts:
            area = cv2.contourArea(c)
            if area > area_threshold:
                detected = True
                final_contours.append(c)
                # cv2.drawContours(result, [c], -1, (255, 0, 0), 5)

        return detected, final_contours

    def draw_intersection_bbox(self, img: np.ndarray, cnt) -> np.ndarray:
        cv2.drawContours(img, [cnt], -1, (255, 0, 0), 5)

    def persepective_wrap(self, img: np.ndarray) -> np.ndarray:
        """ROI of image and Perform perspective wrapping on the image"""
        roi = [
            (self.luroi * ((img.shape[1] - 1)), self.hroi * (img.shape[0] - 1)),
            (self.ruroi * ((img.shape[1] - 1)), self.hroi * (img.shape[0] - 1)),
            (self.lbroi * ((img.shape[1] - 1)), (img.shape[0] - 1) * (1 - self.broi)),
            (self.rbroi * ((img.shape[1] - 1)), (img.shape[0] - 1) * (1 - self.broi)),
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
        # minv = cv2.getPerspectiveTransform(dst, src)  # noqa
        birdseye = cv2.warpPerspective(img, matrix, img_size)

        return birdseye

    def houghlines_angle(self, img: np.ndarray, get_img: bool = False) -> float:
        """Given processed image compute steering angle"""
        # find lanes takes processed image
        a = time()
        # processed_img = self.preprocess_pipeline(img)
        processed_img = cv2.Canny(img, self.canny_thres1, self.canny_thres2)
        kernel = np.ones((11, 11), np.uint8)
        processed_img = cv2.dilate(processed_img, kernel)
        # print("Time taken by preprocess", time()- a)
        b = time()

        lines = find_lanes(processed_img)
        # print(lines)
        # average slop takes original image
        lanelines = average_slope_intercept(img, lines)
        try:
            angle = compute_steering_angle_lanelinecoord(
                img[:, :, 0], lane_lines=lanelines
            )
            # print("angle: ", angle)
        except IndexError as e:
            angle = compute_steering_angle_lanelinecoord(img, lane_lines=lanelines)
            # print("angle: ", angle)

        # print("Time taken to find lanes", time()- b)
        c = time()
        if get_img:
            # draw lanelines
            processed_img = draw_line(processed_img, lanelines)
            # draw heading lines
            outimage = display_heading_line(processed_img, angle)
            return angle, outimage
        return angle

    def sliding_window_search(self, img: np.ndarray) -> float:
        """Given processed image compute steering angle"""

        pass

    def graph_road_search(self, img: np.ndarray) -> float:
        # processed_img = self.preprocess_pipeline(img)
        angle, _ = get_road_ratio_angle(img)
        # out_img=display_heading_line(processed_img,angle)
        return angle
        # ,out_img

    def get_lane_error(self, img: np.ndarray) -> float:
        angle = get_error_lane(img)
        return angle


def get_error_lane(mask_image):
    mid_y = mask_image.shape[0] // 2
    pval = int(mid_y + 0.6 * (mask_image.shape[0] // 2))
    mval = int(mid_y + 0.8 * (mask_image.shape[0] // 2))
    # print(pval,mval)
    img_new = mask_image[pval:mval, :]
    img_new = cv2.resize(
        img_new, None, fx=0.35, fy=0.35, interpolation=cv2.INTER_NEAREST
    )
    # plt.plot(img_new)
    print(img_new.shape)
    histogram = np.sum(img_new[img_new.shape[0] // 2 :, :], axis=0)  # noqa
    plt.plot(histogram)
    midpoint = np.int(histogram.shape[0] / 2)
    leftxBase = np.argmax(histogram[:midpoint])
    rightxBase = np.argmax(histogram[midpoint:]) + midpoint
    print(leftxBase, midpoint, rightxBase)
    return ((abs(rightxBase - midpoint) - abs(leftxBase - midpoint)) * 26.5 / (midpoint)) + 90


def get_road_ratio_angle(mask_img):
    mask = mask_img.copy()
    kernel = np.ones((111, 111), np.uint8)
    img_new = cv2.dilate(mask, kernel)
    img_new = cv2.resize(
        img_new, None, fx=0.035, fy=0.035, interpolation=cv2.INTER_NEAREST
    )

    for i in range(img_new.shape[0]):
        for j in range(img_new.shape[1]):
            if img_new[i][j] > 200:
                img_new[i][j] = 1
            else:
                img_new[i][j] = 0
    visited = bfs_4(img_new, (img_new.shape[0], img_new.shape[1] // 2))
    right = len([i for i in visited if i[1] > img_new.shape[1] // 2])
    # print(img_new.shape[1] // 2)
    left = len(visited) - right
    # print("Len of Halfs: ", right, " ", left)
    return (right - left) / len(visited) * 50 + 90, visited


def make_points(frame, line):
    height, width = frame.shape
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
    try:
        height, width, _ = frame.shape
    except ValueError:
        height, width = frame.shape

    left_fit = []
    right_fit = []

    boundary = 1 / 2
    # left lane line segment should be on left 2/3 of the screen
    left_region_boundary = width * (1 - boundary)
    # right lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary
    try:
        for line_segment in line_segments:
            # print(line_segment)
            for x1, y1, x2, y2 in line_segment:
                #  = make_points(frame, ii)[0]
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
    except ValueError as e:
        print(line_segment)
        raise e

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))
    return lane_lines
    # return lane_lines, left_fit_average, right_fit_average


def find_lanes(thresh_canny):
    """
    Takes Threshold Canny Edge Detected image as imput

    Args:
        thresh_canny ([type]): [description]

    Returns:
        [type]: [description]
    """
    # lines = cv2.HoughLines(
    #     thresh_canny,
    #     1,
    #     np.pi / 180,
    #     200,
    #     min_theta=math.pi / 8,
    #     max_theta=7 * math.pi / 8,
    # )
    # return lines
    lines = cv2.HoughLinesP(
        thresh_canny, 1, np.pi / 180, 15, minLineLength=40, maxLineGap=10
    )
    return lines


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
        camera_mid_offset_percent = 0.00
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
