import cv2
import numpy as np
import time
import tensorflow as tf
import os
from typing import *


def setup() -> Tuple[Callable, List[str]]:
    print("Starting x86")
    os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"  # Suppress TensorFlow logging (1)

    tf.get_logger().setLevel("ERROR")  # Suppress TensorFlow logging (2)

    # Enable GPU dynamic memory allocation
    gpus = tf.config.experimental.list_physical_devices("GPU")
    for gpu in gpus:
        tf.config.experimental.set_memory_growth(gpu, True)

    PATH_TO_SAVED_MODEL = "./src/lib/perception/models/saved_model"
    PATH_TO_LABELS = "./src/lib/perception/labels.txt"

    print("Loading model...", end="")
    start_time = time.time()

    # Load saved model and build the detection function
    detect_fn = tf.saved_model.load(PATH_TO_SAVED_MODEL)

    end_time = time.time()
    elapsed_time = end_time - start_time
    print("Done! Took {} seconds".format(elapsed_time))

    with open(PATH_TO_LABELS) as f:
        my_list = list(f)

    category_index = [i.strip("\n") for i in my_list]

    return detect_fn, category_index


def draw_box(img, text, location, box):
    fontScale = 1
    color = (255, 0, 0)
    thickness = 1
    font = cv2.FONT_HERSHEY_SIMPLEX

    retimg = cv2.rectangle(img, box[0], box[1], color, thickness)
    retimg = cv2.putText(
        retimg, text, location, font, fontScale, color, thickness, cv2.LINE_AA
    )
    return retimg


def detect_signs(img, model, labels):

    shapes, predictions_dict, detections, _ = model([img])
    label_id_offset = 1

    if predictions_dict[0][0] > 0.85:

        box = (
            (int(shapes[0][0][1]), int(shapes[0][0][0])),
            (int(shapes[0][0][3]), int(shapes[0][0][2])),
        )
        text = labels[int(detections[0][0].numpy()) - label_id_offset]
        location = (int(shapes[0][0][1]), int(shapes[0][0][0]))
        return box, text, location

    return None


if __name__ == "__main__":

    interpreter, labels = setup()

    frame = cv2.imread("data/priority.jpg", cv2.IMREAD_COLOR)
    print(frame.shape)

    box, text, location = detect_signs(frame, interpreter, labels)
    print(box, text, location)
    cv2.imshow("object detection", draw_box(frame, text, location, box))

    key = cv2.waitKey(0)
    if key == 27:  # esc
        cv2.destroyAllWindows()
