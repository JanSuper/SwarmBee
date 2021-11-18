import cv2

from djitellopy import tello
from DroneSwarm.src.Utilities.tello_bluetooth_receiver import BackgroundBluetoothSensorRead


def initialize_tello():
    my_drone = tello.Tello()
    my_drone.connect()
    my_drone.for_back_velocity = 0
    my_drone.left_right_velocity = 0
    my_drone.up_down_velocity = 0
    my_drone.yaw_velocity = 0
    my_drone.speed = 0
    print("Battery Level: ", my_drone.get_battery(), "%")
    my_drone.streamoff()
    my_drone.streamon()
    return my_drone


def tello_get_frame(my_drone, w=360, h=240):
    my_frame: object = my_drone.get_frame_read()
    my_frame = my_frame.frame
    img = cv2.resize(my_frame, (w, h))
    return img

