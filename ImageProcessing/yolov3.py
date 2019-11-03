# Initialize the parameters
confThreshold = 0.8  # Confidence threshold
nmsThreshold = 0.4  # Non-maximum suppression threshold
inpWidth = 416  # Width of network's input image
inpHeight = 416  # Height of network's input image

import cv2 as cv
import numpy as np
import os


class Yolo:
    def __init__(self):
        self.confThreshold = 0.8  # Confidence threshold
        self.nmsThreshold = 0.4  # Non-maximum suppression threshold
        self.inpWidth = 416  # Width of network's input image
        self.inpHeight = 416  # Height of network's input image
        print(os.getcwd())
        classesFile = "./ImageProcessing/coco.names"
        self.classes = None
        self.model_initialized = False

        with open(classesFile, "rt") as f:
            self.classes = f.read().rstrip("\n").split("\n")

    def initializeModel(self):
        modelConfiguration = "./ImageProcessing/yolov3.cfg"
        modelWeights = "./ImageProcessing/yolov3.weights"

        self.net = cv.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
        self.net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)
        self.model_initialized = True

    def getOutputsNames(self):
        # Get the names of all the layers in the network
        layersNames = self.net.getLayerNames()
        # Get the names of the output layers, i.e. the layers with unconnected outputs
        return [layersNames[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

    def drawPred(self, frame, class_name, conf, left, top, right, bottom):
        # Draw a bounding box.
        cv.rectangle(frame, (left, top), (right, bottom), (0, 0, 255))

        label = "%.2f" % conf

        # Get the label for the class name and its confidence

        label = "%s:%s" % (class_name, label)

        # Display the label at the top of the bounding box
        labelSize, baseLine = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(top, labelSize[1])
        cv.putText(
            frame, label, (left, top), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255)
        )
        return frame

    def detect(self, image, object_name, return_img=False):
        """
        Returns: boxes,confidences,class_names,image(optional)
        boxes - list of (coordinates of) all objects [(x1,y1,x2,y2)]

        """
        blob = cv.dnn.blobFromImage(
            image, 1 / 255, (self.inpWidth, self.inpHeight), [0, 0, 0], 1, crop=False
        )
        # Sets the input to the network
        self.net.setInput(blob)
        # Runs the forward pass to get output of the output layers
        outs = self.net.forward(self.getOutputsNames())
        obj_ind = self.classes.index(object_name)

        frameHeight = image.shape[0]
        frameWidth = image.shape[1]

        classIds = []
        confidences = []
        boxes = []
        # Scan through all the bounding boxes output from the network and keep only the
        # ones with high confidence scores. Assign the box's class label as the class with the highest score.
        classIds = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                classId = np.argmax(scores)
                confidence = scores[classId]
                if confidence > confThreshold and classId == obj_ind:
                    center_x = int(detection[0] * frameWidth)
                    center_y = int(detection[1] * frameHeight)
                    width = int(detection[2] * frameWidth)
                    height = int(detection[3] * frameHeight)
                    left = int(center_x - width / 2)
                    top = int(center_y - height / 2)
                    classIds.append(classId)
                    confidences.append(float(confidence))
                    boxes.append([left, top, width, height])

        # Perform non maximum suppression to eliminate redundant overlapping boxes with
        # lower confidences.
        indices = cv.dnn.NMSBoxes(boxes, confidences, confThreshold, nmsThreshold)
        boxes = [boxes[i[0]] for i in indices]
        confidences = [confidences[i[0]] for i in indices]
        classNames = [self.classes[classIds[i[0]]] for i in indices]
        if return_img:
            return (
                boxes,
                confidences,
                classNames,
                self.draw_boxes(image, confidences, boxes, classNames),
            )
        return boxes, confidences, classNames

    def draw_boxes(self, frame, confidences, boxes, object_name):
        for ind, val in enumerate(boxes):
            frame = self.drawPred(
                frame,
                object_name[ind],
                confidences[ind],
                val[0],
                val[1],
                val[0] + val[2],
                val[1] + val[3],
            )
        return frame


# if __name__=="__main__":
#     yolo=Yolo()
#     yolo.initializeModel()
#     frame= cv.imread('bird.jpg')
#     b,con,cl,frame=(yolo.detect(frame,'bird',return_img=True))
#     cv.imwrite("bird-out.jpg", frame.astype(np.uint8));
