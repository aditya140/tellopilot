# import necessary modules
import cv2
import sdl2
import sdl2.ext
import numpy
from ImageProcessing.yolov3 import Yolo

yolo = Yolo()
yolo.initializeModel()

windowSize = (1280, 720)
# initialize the camera
vc = cv2.VideoCapture(0)

# grab and show a first frame from the camera
junk, image = vc.read()
# cv2.imshow('0',image)

# initialize sdl2
sdl2.ext.init()
window = sdl2.ext.Window("test", size=windowSize)
window.show()
windowSurf = sdl2.SDL_GetWindowSurface(window.window)
windowArray = sdl2.ext.pixels3d(windowSurf.contents)


tracker = cv2.TrackerCSRT().create()
running = True
start_tracker = False
detected = False
while running:  # keep reading to have a live feed from the cam
    junk, image = vc.read()
    events = sdl2.ext.get_events()
    for event in events:
        if event.type == sdl2.SDL_KEYDOWN:
            if event.key.keysym.sym == sdl2.SDLK_a:
                if not start_tracker:
                    start_tracker = True
                else:
                    start_tracker = False
                    detected = False
                print(start_tracker)
            elif event.key.keysym.sym == sdl2.SDLK_0:
                running = False
                break
    if detected == False and start_tracker == True:
        boxes, conf, classes, image = yolo.detect(image, "person", return_img=True)
        del tracker
        tracker = cv2.TrackerCSRT().create()
        if len(classes) > 0:
            detected = True
            tracker.init(image, (boxes[0][0], boxes[0][1], boxes[0][2], boxes[0][3]))
    if detected == True and start_tracker == True:
        ok, bbox = tracker.update(image)
        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(image, p1, p2, (255, 0, 0), 2, 1)
        else:
            detected = False
    image = numpy.insert(image, 3, 255, axis=2)  # add alpha
    image = numpy.rot90(image)  # rotate dims
    numpy.copyto(windowArray, image)
    window.refresh()
