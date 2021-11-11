""" Keyboard control module """
from djitellopy import tello
from time import sleep
# Copyright (c) 2021 https://github.com/HKagiri/DroneProgramming-OpenCV-Python
import KeyPressModule as kp

# initialize the module
kp.init()
me = tello.Tello()
me.connect()
print(me.get_battery())


def get_key_input():
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 50

    # Roll Inputs
    if kp.get_key("LEFT"):
        lr = -speed
    elif kp.get_key("RIGHT"):
        lr = speed

    # Pitch Inputs
    if kp.get_key("UP"):
        fb = speed
    elif kp.get_key("DOWN"):
        fb = -speed

    # Throttle Inputs
    if kp.get_key("w"):
        ud = speed
    elif kp.get_key("s"):
        ud = -speed

    # Yaw Inputs
    if kp.get_key("a"):
        yv = -speed
    elif kp.get_key("d"):
        yv = speed

    # Land Inputs
    if kp.get_key("q"):
        me.land()

    # Take-Off Inputs
    if kp.get_key("e"):
        me.takeoff()

    return [lr, fb, ud, yv]


while True:
    print(kp.get_key("s"))  # control key
    value = get_key_input()
    # me.send_rc_control(value[0], value[1], value[2], value[3])
    sleep(0.05)
