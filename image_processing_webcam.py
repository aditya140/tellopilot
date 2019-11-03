# image Processing

import numpy as np
import cv2
import numpy as np
import cv2 as cv
from ImageProcessing.yolov3 import Yolo

face_cascade = cv.CascadeClassifier("haarcascade_frontalface_default.xml")
eye_cascade = cv.CascadeClassifier("haarcascade_eye.xml")

cap = cv2.VideoCapture(0)
yolo = Yolo()
yolo.initializeModel()
tracker = cv2.TrackerCSRT().create()
tracker_start = False
first_frame = True
# size=
out = cv2.VideoWriter("project.avi", cv2.VideoWriter_fourcc(*"DIVX"), 15, (1280, 720))
while True:

    # Capture frame-by-frame
    ret, frame = cap.read()
    shape = frame.shape
    print(shape)

    fx = 0.5
    fy = 0.5
    # Our operations on the frame come here
    img = cv2.resize(frame, None, fx=0.5, fy=0.5)
    faces = face_cascade.detectMultiScale(img, 1.3, 5)
    for (x, y, w, h) in faces:
        x = int(x / fx)
        w = int(w / fx)
        y = int(y / fy)
        h = int(h / fy)
        cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        centroid = (x) + (w / 2) - 640
        cv.putText(
            img=frame,
            text="centroid : {}".format(centroid),
            org=(0, 50),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.15 * 5,
            color=(255, 255, 255),
        )
        roi_color = frame[y : y + h, x : x + w]
    # Display the resulting frame
    if tracker_start == False:
        boxes, conf, class_names, frame = yolo.detect(frame, "person", return_img=True)
    if len(conf) >= 1:
        tracker_start = True

    if tracker_start == True:
        if first_frame == True:
            bbox = cv2.selectROI(frame, False)
        tracker.init(frame, bbox)
        # tracker.init(frame, (boxes[0][0], boxes[0][1], boxes[0][0] + boxes[0][2], boxes[0][1] + boxes[0][3]))
        ok, bbox = tracker.update(frame)
        print(ok, bbox)
        print(type(ok))
        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
            person = frame[
                int(bbox[0]) : int(bbox[0]) + int(bbox[2]),
                int(bbox[1]) : int(bbox[1]) + int(bbox[3]),
            ]
        else:
            print("Stop--------------")
            tracker_start = False
            first_frame = True
    out.write(frame)
    cv2.imshow("frame", frame)
    if first_frame == True:
        first_frame = False

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# When everything done, release the capture
cap.release()
out.release()
cv2.destroyAllWindows()
