from djitellopy import Tello
import cv2
import pygame
from pygame.locals import *
import numpy as np
import time
import timeit
from PIL import ImageFont, ImageDraw, Image
from ImageProcessing.yolov3 import Yolo

face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

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
logger = setup_logger("logger", "tello_game.log")

# Speed of the drone
S = 60
FPS = 10


class FrontEnd(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations
            - W and S: Up and down.
            - B: Go to Ground
            - K: Emergency Land
            - Q: Emergency Motor Kill
            - F: Face Follow mode
    """

    def __init__(self):
        # Init pygame
        pygame.init()

        # Creat pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])

        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10
        self.mode = None
        self.send_rc_control = False
        self.yolo = Yolo()
        self.yolo.initializeModel()
        self.tracker = tracker = cv2.TrackerCSRT().create()
        self.locked = False
        self.locked_frame = None

        # create update timer
        pygame.time.set_timer(USEREVENT + 1, 50)
        logger.info("Game Initialized")

    def run(self):

        if not self.tello.connect():
            print("Tello not connected")
            logger.error("Tello not connected")
            return

        if not self.tello.set_speed(self.speed):
            print("Not set speed to lowest possible")
            logger.error("Not set speed to lowest possible")
            return

        # In case streaming is on. This happens when we quit this program without the escape key.
        if not self.tello.streamoff():
            print("Could not stop video stream")
            logger.error("Could not stop video stream")
            return

        if not self.tello.streamon():
            print("Could not start video stream")
            logger.error("Could not start video stream")
            return

        frame_read = self.tello.get_frame_read()
        should_stop = False
        while not should_stop:

            for event in pygame.event.get():
                if event.type == USEREVENT + 1:
                    if self.mode != None:
                        frame_read.frame = self.get_update(frame_read.frame)
                    self.update()
                elif event.type == QUIT:
                    should_stop = True
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        should_stop = True
                    else:
                        self.keydown(event.key)
                elif event.type == KEYUP:
                    self.keyup(event.key)

            if frame_read.stopped:
                frame_read.stop()
                break

            self.screen.fill([0, 0, 0])
            frame = cv2.cvtColor(frame_read.frame, cv2.COLOR_BGR2RGB)
            cv2.putText(
                img=frame,
                text="Height : {}".format(self.tello.get_tello_status().h),
                org=(0, 50),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.15 * 5,
                color=(255, 255, 255),
            )
            cv2.putText(
                img=frame,
                text="Battery : {}".format(self.tello.get_tello_status().bat),
                org=(0, 70),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.15 * 5,
                color=(255, 255, 255),
            )
            cv2.putText(
                img=frame,
                text="Temp : {} - {}".format(
                    self.tello.get_tello_status().temph,
                    self.tello.get_tello_status().templ,
                ),
                org=(0, 90),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.15 * 5,
                color=(255, 255, 255),
            )
            frame = np.rot90(frame)
            frame = np.flipud(frame)
            frame = pygame.surfarray.make_surface(frame)
            self.screen.blit(frame, (0, 0))
            time.sleep((1 / FPS))
            pygame.display.update()
            self.tello.get_tello_status()

        # Call it always before finishing. I deallocate resources.
        self.tello.end()

    def keydown(self, key):
        """ Update velocities based on key pressed
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP:  # set forward velocity
            self.for_back_velocity = S
        elif key == pygame.K_DOWN:  # set backward velocity
            self.for_back_velocity = -S
        elif key == pygame.K_LEFT:  # set left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_RIGHT:  # set right velocity
            self.left_right_velocity = S
        elif key == pygame.K_w:  # set up velocity
            self.up_down_velocity = S
        elif key == pygame.K_s:  # set down velocity
            self.up_down_velocity = -S
        elif key == pygame.K_a:  # set yaw clockwise velocity
            self.yaw_velocity = -S
        elif key == pygame.K_d:  # set yaw counter clockwise velocity
            self.yaw_velocity = S
        elif key == pygame.K_f:
            if self.mode == "Aquire lock" or self.mode == "Follow":
                logger.info("Back to normal mode")
                self.mode = None
                self.locked = False
                self.locked_frame = None
            else:
                logger.info("Aquiring lock")
                self.mode = "Aquire lock"

    def keyup(self, key):
        """ Update velocities based on key released
        Arguments:
            key: pygame key
        """
        if (
            key == pygame.K_UP or key == pygame.K_DOWN
        ):  # set zero forward/backward velocity
            self.for_back_velocity = 0
        elif (
            key == pygame.K_LEFT or key == pygame.K_RIGHT
        ):  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
            self.up_down_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        elif key == pygame.K_t:  # takeoff
            self.tello.takeoff()
            logger.info("Take off")
            self.send_rc_control = True
        elif key == pygame.K_l:  # land
            self.tello.land()
            logger.info("Land")
            self.send_rc_control = False
        elif key == pygame.K_b:  # go to ground
            self.tello.go_to_ground()
            logger.info("Got to Ground")
            self.send_rc_control = False
        elif key == pygame.K_k:  # Manual land and kill motors
            self.tello.emergency_land()
            logger.info("Emmergency land")
            self.send_rc_control = False
        # elif key ==pygame.K_q:
        #     self.tello.emergency()
        #     logger.info("Kill motors")
        #     self.send_rc_control = False

    def update(self):
        """ Update routine. Send velocities to Tello."""
        if self.send_rc_control:
            self.tello.send_rc_control(
                self.left_right_velocity,
                self.for_back_velocity,
                self.up_down_velocity,
                self.yaw_velocity,
            )

    def get_update(self, frame_read):
        if self.mode == "Aquire lock":
            return self.aquire_lock(frame_read)
        if self.mode == "Follow":
            return self.follow(frame_read)

    def face_follow(self, frame_read):
        cv2.cvtColor(frame_read, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(frame_read, 1.3, 5)
        frame_x = frame_read.shape[1]
        frame_y = frame_read.shape[0]
        face_center_x = 0
        for (x, y, w, h) in faces:
            cv2.rectangle(frame_read, (x, y), (x + w, y + h), (255, 0, 0), 2)
            face_center_x = x + (w / 2) - frame_x
            break
        if face_center_x > 200:
            self.yaw_velocity = -S
        if face_center_x < -200:
            self.yaw_velocity = S
        else:
            self.yaw_velocity = 0
        logger.info(
            "Frame_x: {} \tface_center: {} \tyaw_vel: {}".format(
                frame_x, face_center_x, self.yaw_velocity
            )
        )
        return frame_read

    def aquire_lock(self, frame):
        bbox, _, _ = self.yolo.detect(frame, "person")
        if len(bbox) > 0:
            self.locked = True
            self.locked_frame = [int(i) for i in bbox[0]]
            logger.info("Lock Aquired : {}".format(self.locked_frame))
            self.mode = "Follow"
            self.tracker.init(
                frame,
                (
                    self.locked_frame[0],
                    self.locked_frame[1],
                    self.locked_frame[0] + self.locked_frame[2],
                    self.locked_frame[1] + self.locked_frame[3],
                ),
            )
            return self.follow(frame)
        return frame

    def follow(self, frame_read):
        ok, self.locked_frame = self.tracker.update(frame_read)
        self.locked_frame = [int(i) for i in self.locked_frame]
        frame_shape = (frame_read.shape[1], frame_read.shape[0])
        logger.info("Locked Frame : {}".format(self.locked_frame))
        if ok:
            self.calcMovementVector(frame_shape, self.locked_frame)
            cv2.rectangle(
                frame_read,
                (int(self.locked_frame[0]), self.locked_frame[1]),
                (
                    self.locked_frame[0] + self.locked_frame[2],
                    self.locked_frame[1] + self.locked_frame[3],
                ),
                (255, 125, 0),
                2,
            )
        else:
            self.mode = "Aquire Lock"
            self.locked = False
            self.locked_frame = None
        return frame_read

    def calcMovementVector(self, frame_shape, frame):
        frame_center = (int(frame_shape[0] / 2), int(frame_shape[1] / 2))
        x_mov = (frame[0] + frame[2] / 2) - frame_center[0]
        logger.info("X mov : {}".format(x_mov))
        if x_mov > 50 or x_mov < -50:
            self.yaw_velocity = int(S * x_mov / frame_center[0])


def main():
    frontend = FrontEnd()

    # run frontend
    frontend.run()


if __name__ == "__main__":
    main()
