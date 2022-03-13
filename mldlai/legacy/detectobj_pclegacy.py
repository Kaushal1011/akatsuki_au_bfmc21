import cv2
import numpy as np
import time
import tensorflow as tf
import pathlib
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'    # Suppress TensorFlow logging (1)

tf.get_logger().setLevel('ERROR')           # Suppress TensorFlow logging (2)

# Enable GPU dynamic memory allocation
gpus = tf.config.experimental.list_physical_devices('GPU')
for gpu in gpus:
    tf.config.experimental.set_memory_growth(gpu, True)


PATH_TO_SAVED_MODEL = "./data/saved_model"
PATH_TO_LABELS = "./labels.txt"

print('Loading model...', end='')
start_time = time.time()

# Load saved model and build the detection function
detect_fn = tf.saved_model.load(PATH_TO_SAVED_MODEL)

end_time = time.time()
elapsed_time = end_time - start_time
print('Done! Took {} seconds'.format(elapsed_time))

# category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS,
#                                                                     use_display_name=True)

with open('labels.txt') as f:
    my_list = list(f)

category_index = [i.strip("\n") for i in my_list]

cap = cv2.VideoCapture(0)


while True:
    # Read frame from camera
    ret, image_np = cap.read()

    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
    # image_np_expanded = np.expand_dims(image_np, axis=0)

    # Things to try:
    # Flip horizontally
    # image_np = np.fliplr(image_np).copy()

    # Convert image to grayscale
    # image_np = np.tile(
    #     np.mean(image_np, 2, keepdims=True), (1, 1, 3)).astype(np.uint8)

    # input_tensor = tf.convert_to_tensor(
    #     np.expand_dims(image_np, 0), dtype=tf.uint8)
    shapes, predictions_dict, detections,  _ = detect_fn([image_np])

    label_id_offset = 1
    image_np_with_detections = image_np.copy()
    print(shapes[0].numpy())
    print(detections[0].numpy())
    print(predictions_dict[0].numpy())
    # Ending coordinate, here (220, 220)
    # represents the bottom right corner of rectangle
    # end_point = (220, 220)
    # Line thickness of 2 px
    thickness = 2

    # Blue color in BGR
    color = (255, 0, 0)
    if predictions_dict[0][0] > 0.3:
        image_np_with_detections = cv2.rectangle(
            image_np_with_detections, (int(shapes[0][0][1]), int(shapes[0][0][0])), (int(shapes[0][0][3]), int(shapes[0][0][2])), color, thickness)
        font = cv2.FONT_HERSHEY_SIMPLEX  # org
        org = (int(shapes[0][0][1]), int(shapes[0][0][0]))

        # fontScale
        fontScale = 1

        # Blue color in BGR
        color = (255, 0, 0)

        # Line thickness of 2 px
        thickness = 2

        # Using cv2.putText() method
        image_np_with_detections = cv2.putText(image_np_with_detections, category_index[int(detections[0][0].numpy())-1], org, font,
                                               fontScale, color, thickness, cv2.LINE_AA)
    # Display output
    cv2.imshow('object detection', cv2.resize(
        image_np_with_detections, (800, 600)))

    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()