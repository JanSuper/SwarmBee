# from djitellopy module: https://github.com/damiafuentes/DJITelloPy.git

import time
from threading import Thread
import cv2


class BackgroundFrameRead:
    """
    This class read frames from a VideoCapture in background. Use
    backgroundFrameRead.frame to get the current frame.
    """

    def __init__(self, tello, address):
        tello.cap = cv2.VideoCapture(address)

        self.cap = tello.cap

        if not self.cap.isOpened():
            self.cap.open(address)

        # Try grabbing a frame multiple times
        # According to issue #90 the decoder might need some time
        # https://github.com/damiafuentes/DJITelloPy/issues/90#issuecomment-855458905
        start = time.time()
        while time.time() - start < Tello.FRAME_GRAB_TIMEOUT:
            #Tello.LOGGER.debug('trying to grab a frame...')
            self.grabbed, self.frame = self.cap.read()
            if self.frame is not None:
                break
            time.sleep(0.05)

        if not self.grabbed or self.frame is None:
            raise Exception('Failed to grab first frame from video stream')

        self.stopped = False
        self.worker = Thread(target=self.update_frame, args=(), daemon=True)

    def start(self):
        """Start the frame update worker
        Internal method, you normally wouldn't call this yourself.
        """
        self.worker.start()

    def update_frame(self):
        """Thread worker function to retrieve frames from a VideoCapture
        Internal method, you normally wouldn't call this yourself.
        """
        while not self.stopped:
            if not self.grabbed or not self.cap.isOpened():
                self.stop()
            else:
                self.grabbed, self.frame = self.cap.read()

    def stop(self):
        """Stop the frame update worker
        Internal method, you normally wouldn't call this yourself.
        """
        self.stopped = True
        self.worker.join()


from DroneSwarm.src.CV.tello_pose_experimentation.lib.tello import Tello

