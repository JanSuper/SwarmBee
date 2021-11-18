import socket
from typing import Optional
import cv2


class Tello:

    FRAME_GRAB_TIMEOUT = 3

    def __init__(self):
        self.port = 8889
        self.addr = ('192.168.10.1', self.port)
        self.udp_vid_addr = ('udp://0.0.0.0:11111')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.background_frame_read: Optional['BackgroundFrameRead'] = None
        self.cap: Optional[cv2.VideoCapture] = None

    def send(self, message):
        try:
            self.sock.sendto(message.encode(), self.addr)
            print("Sending message: " + message)
        except Exception as e:
            print("Error sending: " + str(e))

    def receive(self):
        return


    def close(self):
        self.sock.close()


    def get_frame_read(self):
        """Get the BackgroundFrameRead object from the camera drone. Then, you just need to call
        backgroundFrameRead.frame to get the actual frame received by the drone.
        Returns:
            BackgroundFrameRead
        """
        if self.background_frame_read is None:
            address = self.udp_vid_addr
            self.background_frame_read = BackgroundFrameRead(self, address)  # also sets self.cap
            self.background_frame_read.start()
        return self.background_frame_read


# from tello_pose_experimentation.lib.BackgroundFrameRead import BackgroundFrameRead
from DroneSwarm.src.CV.tello_pose_experimentation.lib.BackgroundFrameRead import BackgroundFrameRead