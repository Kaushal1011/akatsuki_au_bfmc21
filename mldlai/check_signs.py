import cv2
import numpy as np
import time
import platform
# import joblib


device = platform.uname().processor

if device == "x86_64":
    print("Using x86")
    from detectts_x86 import setup, detect_signs, draw_box
else:
    from detectts_armtflite import setup, detect_signs, draw_box

if __name__ == "__main__":

    model, labels = setup()

    #  area_arr=[]
    # sign_arr=[]

    cap = cv2.VideoCapture("./data/bfmc/bfmc2020_online_1.avi")
    frame_count = 0
    while cap.isOpened():
        frame_count += 1
        ret, frame = cap.read()
        try:
            img = frame.copy()
        except:
            break
        if frame_count %5 != 0:
            continue
        width = img.shape[1]
        height = img.shape[0]
        # should be top right quarter
        img = img[: int(height / 2), int(width / 2) :]
        a = time.time()
        print(model)
        out = detect_signs(img, model, labels)
        print(time.time() - a)
        if out is not None:
            box, text, location = out
            # box 0 is top left box 1 is bottom right
            # area = wxh w=x2-x1 h=y2-y1
            area = (box[1][0] - box[0][0]) * (box[1][1] - box[0][1])
            print(text)
            # area_arr.append(area)
            # sign_arr.append(text)
            # print(text)
            if area < 10000:
                continue
            frame = draw_box(img, text, location, box)
            # print(box)
        else:
            pass
            # area_arr.append(0)
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        try:
            cv2.imshow("frame", img)
            # cv2.imwrite("detected.png",frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("broke")
                # joblib.dump(area_arr,"area.z")
                break
        except:
            print("broke")
            # joblib.dump(area_arr,"area.z")
            break
    print(frame_count)
    print("broke")
    # joblib.dump(area_arr,"area.z")
    cap.release()
    cv2.destroyAllWindows()
