"""
Run object detection on images, Press ESC to exit the program
For Raspberry PI, please use `import tflite_runtime.interpreter as tflite` instead
"""
import re
import cv2
import numpy as np
import time

import tensorflow.lite as tflite
# import tflite_runtime.interpreter as tflite

from PIL import Image
from typing import *


def load_labels(label_path):
    r"""Returns a list of labels"""
    with open(label_path) as f:
        my_list = list(f)

    category_index = [i.strip("\n") for i in my_list]
    return category_index


def load_model(model_path):
    r"""Load TFLite model, returns a Interpreter instance."""
    interpreter = tflite.Interpreter(model_path=model_path, num_threads=4)
    interpreter.allocate_tensors()
    return interpreter


def process_image(interpreter, image, input_index, labels, w, h):
    r"""Process an image, Return a list of detected class ids and positions"""
    input_data = np.expand_dims(image, axis=0)  # expand to 4-dim

    # Process
    interpreter.set_tensor(input_index, input_data)
    interpreter.invoke()

    # Get outputs
    output_details = interpreter.get_output_details()
    # print(output_details)
    # output_details[0] - position
    # output_details[1] - class id
    # output_details[2] - score
    # output_details[3] - count

    positions = np.squeeze(interpreter.get_tensor(output_details[1]["index"]))
    classes = np.squeeze(interpreter.get_tensor(output_details[3]["index"]))
    scores = np.squeeze(interpreter.get_tensor(output_details[0]["index"]))
    # print(positions[0][0])
    # print(classes)
    # print(scores)
    # print([i for i in enumerate(scores)])

    # result = []
    # take highest score element
    score = scores[0]
    if score > 0.5:
        box = (
            (int(positions[0][1] * w), int(positions[0][0] * h)),
            (int(positions[0][3] * w), int(positions[0][2] * h)),
        )
        text = labels[int(classes[0])]
        location = (int(positions[0][1] * w), int(positions[0][0] * h))

        return box, text, location
    else:
        return None


def setup() -> Tuple[Callable, List[str]]:

    PATH_TO_SAVED_MODEL = "./models/trafficsigns.tflite"
    PATH_TO_LABELS = "./labels.txt"

    print("Loading model...", end="")
    start_time = time.time()
    detect_fn = load_model(PATH_TO_SAVED_MODEL)
    end_time = time.time()
    elapsed_time = end_time - start_time
    print("Done! Took {} seconds".format(elapsed_time))

    labels = load_labels(PATH_TO_LABELS)

    return detect_fn, labels


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


def detect_signs(img, interpreter, labels):

    w = img.shape[1]
    h = img.shape[0]

    input_details = interpreter.get_input_details()
    input_shape = input_details[0]["shape"]
    height = input_shape[1]
    width = input_shape[2]

    # Get input index
    input_index = input_details[0]["index"]
    frame = img.copy()
    image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    image = image.resize((width, height))
    input_index = input_details[0]["index"]
    top_result = process_image(interpreter, image, input_index, labels, w, h)
    return top_result


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

