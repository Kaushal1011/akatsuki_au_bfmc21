from tkinter import Frame
import numpy as np
import cv2

thres = 0.4 # Threshold to detect object
nms_threshold = 0.1 #(0.1 to 1) 1 means no suppress , 0.1 means high suppress

classNames = []
with open('coco.names','r') as f:
    classNames = f.read().splitlines()
print(classNames)

font = cv2.FONT_HERSHEY_PLAIN
#font = cv2.FONT_HERSHEY_COMPLEX
Colors = np.random.uniform(0, 255, size=(len(classNames), 3))

weightsPath = "frozen_inference_graph.pb"
configPath = "ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)


    
def getObject(img, objects=[]):  
    classIds, confs, bbox = net.detect(img, confThreshold = thres, nmsThreshold = nms_threshold)
    if len(objects) == 0: objects = classNames
    objectInfo = []
    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            className = classNames[classId-1]
            if className in objects:
                objectInfo.append([confidence*100, box, className])
                cv2.rectangle(img, box, color=(0,255,0), thickness=2)
                cv2.putText(img, classNames[classId-1].upper(), (box[0]+10, box[1]+30), cv2.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 2)
                cv2.putText(img, str(round(confidence*100,2)), (box[0]+200, box[1]+50), cv2.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 2)

    return img, objectInfo
    

if __name__ == "__main__":
    cap = cv2.VideoCapture('/home/b0nzo/Downloads/bfmc2020_online_1.avi')
    # frame = cv2.imread("/home/b0nzo/akatsuki_au_bfmc21/nbs/tl.png")
    while True:
        ret, frame = cap.read()
        result, objectinfo = getObject(frame, objects=['traffic light'])
        # result, objectinfo = getObject(frame)
        print(objectinfo)
        cv2.imshow("output", frame)
        cv2.waitKey(1)