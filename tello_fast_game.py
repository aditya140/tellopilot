from djitellopy import Tello
import cv2
import sdl2
import sdl2.ext
import time
import numpy
from ImageProcessing.yolov3 import Yolo
import logging

formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")


def setup_logger(name, log_file, level=logging.INFO):
    """Function setup as many loggers as you want"""

    handler = logging.FileHandler(log_file)
    handler.setFormatter(formatter)

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.addHandler(handler)

    return logger


# first file logger
logger = setup_logger("logger", "tello_fast_game.log")


class Drone(object):
    def __init__(self):
        self.tello = Tello()
        self.yaw_velocity = 0
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.speed = 10
        self.mode = None
        self.send_rc_control = False
        self.height_limit = 200
        # Default Speed of droone
        self.S = 60

    def setZero(self):
        self.yaw_velocity = 0
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0

    def update(self):
        """ Update routine. Send velocities to Tello."""
        if self.send_rc_control:
            self.adjustToLimit()
            self.tello.send_rc_control(
                self.left_right_velocity,
                self.for_back_velocity,
                self.up_down_velocity,
                self.yaw_velocity,
            )

    def adjustToLimit(self):
        if self.tello.get_height_status() >= self.height_limit:
            if self.up_down_velocity > 0:
                self.up_down_velocity = 0


class Game(object):
    def __init__(self):
        self.windowSize = (960, 720)
        # initialize sdl2 game window
        sdl2.ext.init()
        self.window = sdl2.ext.Window("test", size=self.windowSize)
        self.window.show()
        windowSurf = sdl2.SDL_GetWindowSurface(self.window.window)
        self.windowArray = sdl2.ext.pixels3d(windowSurf.contents)
        self.image = None
        # Init Tello object that interacts with the Tello drone
        self.drone = Drone()
        self.mode = None
        self.yolo_initialized = False
        self.tracker_initialized = False
        self.face_finder_initialized = False
        self.FPS = 25
        self.follow_obj = "person"
        self.yolo_tracker_sync_time = 4  # time to run yolo once every _3_ sec
        self.yolo_tracker_last_sync = time.time()
        logger.info("Game Initialized")

    def initialzeYolo(self):
        if not self.yolo_initialized:
            self.yolo = Yolo()
            self.yolo.initializeModel()
            self.yolo_initialized = True

    def initalizeTracker(self):
        if not self.tracker_initialized:
            self.tracker = cv2.TrackerKCF().create()
            self.tracker_initialized = True

    def initializeFaceFinder(self):
        if not self.face_finder_initialized:
            self.face_cascade = cv2.CascadeClassifier(
                "haarcascade_frontalface_default.xml"
            )
            self.face_finder_initialized = True

    def resinitalizeTracker(self):
        del self.tracker
        self.tracker_initialized = False
        self.initalizeTracker()

    def run(self):

        if not self.drone.tello.connect():
            print("Tello not connected")
            logger.error("Tello not connected")
            return
        if not self.drone.tello.set_speed(self.drone.speed):
            print("Not set speed to lowest possible")
            logger.error("Not set speed to lowest possible")
            return

        # In case streaming is on. This happens when we quit this program without the escape key.
        if not self.drone.tello.streamoff():
            print("Could not stop video stream")
            logger.error("Could not stop video stream")
            return

        if not self.drone.tello.streamon():
            print("Could not start video stream")
            logger.error("Could not start video stream")
            return

        frame_read = self.drone.tello.get_frame_read()
        self.should_stop = False
        while not self.should_stop:
            self.image = frame_read.frame
            # self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
            events = sdl2.ext.get_events()
            for event in events:
                if event.type == sdl2.SDL_KEYDOWN:
                    key = event.key.keysym.sym
                    self.key_down(key)
                elif event.type == sdl2.SDL_KEYUP:
                    key = event.key.keysym.sym
                    self.key_up(key)
            if frame_read.stopped:
                frame_read.stop()
                break
            if self.mode != None:
                self.mode_updates()
                self.printMode()
            self.update()
            self.addStatustoImg()
            self.image = cv2.flip(self.image, 1)
            self.image = numpy.insert(self.image, 3, 255, axis=2)  # add alpha
            self.image = numpy.rot90(self.image)  # rotate dims
            numpy.copyto(self.windowArray, self.image)
            self.window.refresh()
            # time.sleep(1/self.FPS)

        self.drone.tello.end()

    def key_down(self, key):
        """ Update velocities based on key pressed
        Arguments:
            key: pygame key
        """
        if key == sdl2.SDLK_UP:  # set forward velocity
            self.drone.for_back_velocity = self.drone.S
        elif key == sdl2.SDLK_DOWN:  # set backward velocity
            self.drone.for_back_velocity = -self.drone.S
        elif key == sdl2.SDLK_LEFT:  # set left velocity
            self.drone.left_right_velocity = -self.drone.S
        elif key == sdl2.SDLK_RIGHT:  # set right velocity
            self.drone.left_right_velocity = self.drone.S
        elif key == sdl2.SDLK_w:  # set up velocity
            self.drone.up_down_velocity = self.drone.S
        elif key == sdl2.SDLK_s:  # set down velocity
            self.drone.up_down_velocity = -self.drone.S
        elif key == sdl2.SDLK_a:  # set yaw clockwise velocity
            self.drone.yaw_velocity = -self.drone.S
        elif key == sdl2.SDLK_d:  # set yaw counter clockwise velocity
            self.drone.yaw_velocity = self.drone.S

    def key_up(self, key):
        """ Update velocities based on key released
        Arguments:
            key: pygame key
        """
        if (
            key == sdl2.SDLK_UP or key == sdl2.SDLK_DOWN
        ):  # set zero forward/backward velocity
            self.drone.for_back_velocity = 0
        elif (
            key == sdl2.SDLK_LEFT or key == sdl2.SDLK_RIGHT
        ):  # set zero left/right velocity
            self.drone.left_right_velocity = 0
        elif key == sdl2.SDLK_w or key == sdl2.SDLK_s:  # set zero up/down velocity
            self.drone.up_down_velocity = 0
        elif key == sdl2.SDLK_a or key == sdl2.SDLK_d:  # set zero yaw velocity
            self.drone.yaw_velocity = 0
        elif key == sdl2.SDLK_t:  # takeoff
            self.drone.tello.takeoff()
            logger.info("Take off")
            self.drone.send_rc_control = True
        elif key == sdl2.SDLK_l:  # land
            self.drone.tello.land()
            logger.info("Land")
            self.drone.send_rc_control = False
        elif key == sdl2.SDLK_b:  # go to ground
            self.drone.tello.go_to_ground()
            logger.info("Got to Ground")
            self.drone.send_rc_control = False
        elif key == sdl2.SDLK_k:  # Manual land and kill motors
            self.drone.tello.emergency_land()
            logger.info("Emmergency land")
            self.drone.send_rc_control = False
        elif key == sdl2.SDLK_0 or key == sdl2.SDLK_ESCAPE:
            self.drone.tello.land()
            time.sleep(3)
            self.should_stop = True
        elif key == sdl2.SDLK_p:
            if self.mode == None:
                self.mode = "Person follow"
                self.detected = False
            elif self.mode == "Person follow":
                self.drone.setZero()
                self.mode = None
            self.drone.send_rc_control = False
        elif key == sdl2.SDLK_f:
            if self.mode == None:
                self.mode = "Face follow"
                self.detected = False
            elif self.mode == "Face follow":
                self.drone.setZero()
                self.mode = None
            self.drone.send_rc_control = False
        elif key == sdl2.SDLK_o:
            if self.mode == None:
                self.mode = "Aquire Face"
                self.detected = False
            elif self.mode == "Aquire Face" or self.mode == "Face follow":
                self.drone.setZero()
                self.mode = None
            self.drone.send_rc_control = False

    def printMode(self):
        cv2.putText(
            img=self.image,
            text="Mode : {}".format(self.mode),
            org=(100, 20),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.20 * 5,
            color=(255, 0, 0),
        )

    def mode_updates(self):
        if self.mode == "Person follow":
            return self.aquire_lock_person()
        if self.mode == "Face follow":
            return self.aquire_lock_face()
        if self.mode == "Aquire facce":
            return self.aquire_face()

    def mark_box(self, bbox):
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        self.image = cv2.rectangle(self.image, p1, p2, (255, 0, 0), 2, 1)

    def aquire_face(self):
        self.drone.tello.go_to_height(80)

        face_detected = False
        while not face_detected:
            gray_img = cv2.cvtColor(self.image, cv2.COLOR_RGB2GRAY)
            self.drone.yaw_velocity = self.drone.S
            faces = self.face_cascade.detectMultiScale(gray_img, 1.3, 5)
            if len(faces) > 0:
                face_detected = True
                self.mode = "Face follow"

    def aquire_lock_person(self):
        bbox = None
        self.initialzeYolo()
        self.initalizeTracker()
        time_now = time.time()
        do_sync = (
            True
            if ((time_now - self.yolo_tracker_last_sync) > self.yolo_tracker_sync_time)
            else False
        )
        if not self.detected or do_sync:
            boxes, conf, classes, self.image = self.yolo.detect(
                self.image, self.follow_obj, return_img=True
            )
            if len(boxes) > 0:
                self.yolo_tracker_last_sync = time.time()
                bbox = boxes[0]
                self.detected = True
                self.resinitalizeTracker()
                self.tracker.init(self.image, (bbox[0], bbox[1], bbox[2], bbox[3]))
        else:
            ok, bbox = self.tracker.update(self.image)
            if ok:
                self.mark_box(bbox)
                self.calculateFollowCommands(bbox=bbox, adj_axis=[1, 0, 0])
            else:
                self.detected = False

    def aquire_lock_face(self):
        bbox = None
        self.initializeFaceFinder()
        gray_img = cv2.cvtColor(self.image, cv2.COLOR_RGB2GRAY)
        faces = self.face_cascade.detectMultiScale(gray_img, 1.3, 5)
        if len(faces) > 0:
            bbox = faces[0]
            bbox = (bbox[0], bbox[1], bbox[2], bbox[3])
            self.calculateFollowCommands(bbox=bbox, adj_axis=[1, 1, 1])
            self.mark_box(bbox)
        else:
            self.drone.setZero()

    def calculateFollowCommands(self, bbox, adj_axis):
        if bbox != None:
            image_shape = (self.image.shape[1] // 2, self.image.shape[0] // 2)
            bbox_center = ((bbox[0] + (bbox[2] / 2)), (bbox[1] + (bbox[3] / 2)))
            bbox_area = bbox[2] * bbox[3]

            if adj_axis[0] == 1:
                self.drone.yaw_velocity = int(
                    (bbox_center[0] - image_shape[0]) * self.drone.S / image_shape[0]
                )
                cv2.putText(
                    img=self.image,
                    text="drone yaw vel : {}".format(self.drone.yaw_velocity),
                    org=(0, 150),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.15 * 5,
                    color=(255, 0, 0),
                )

            if adj_axis[1] == 1:
                self.drone.up_down_velocity = int(
                    (bbox_center[1] - image_shape[1]) * self.drone.S / image_shape[1]
                )
                cv2.putText(
                    img=self.image,
                    text="drone up-down vel : {}".format(self.drone.up_down_velocity),
                    org=(0, 170),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.15 * 5,
                    color=(255, 0, 0),
                )

            if adj_axis[2] == 1:
                desired_area = 10000
                self.drone.for_back_velocity = int(
                    self.drone.S
                    * (desired_area - bbox_area)
                    / max(desired_area, bbox_area)
                )
                cv2.putText(
                    img=self.image,
                    text="drone for-back vel : {}".format(self.drone.for_back_velocity),
                    org=(0, 190),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.15 * 5,
                    color=(255, 0, 0),
                )
                self.drone.send_rc_control = True

    def addStatustoImg(self):
        cv2.putText(
            img=self.image,
            text="Height : {}".format(self.drone.tello.get_tello_status().h),
            org=(0, 50),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.15 * 5,
            color=(255, 255, 255),
        )
        cv2.putText(
            img=self.image,
            text="Battery : {}".format(self.drone.tello.get_tello_status().bat),
            org=(0, 70),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.15 * 5,
            color=(255, 255, 255),
        )
        cv2.putText(
            img=self.image,
            text="Temp : {} - {}".format(
                self.drone.tello.get_tello_status().temph,
                self.drone.tello.get_tello_status().templ,
            ),
            org=(0, 90),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.15 * 5,
            color=(255, 255, 255),
        )

    def update(self):
        """ Update routine. Send velocities to Tello."""
        self.drone.update()


def main():
    game = Game()
    game.run()


if __name__ == "__main__":
    main()
