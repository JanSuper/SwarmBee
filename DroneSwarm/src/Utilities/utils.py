import cv2
import statistics as stats
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


def normalize(list1, var_threshold=0.5, interval=(0.01, 2.1), interval_threshold=3, print_out=False, dir=False):
    flag = 0
    var = []
    if not dir:
        for direction in list1:
            for val in direction:
                if val < interval[0] or interval[1] < val:
                    flag += 1
            var.append(stats.variance(direction))
    else:
        for val in list1:
            if val < interval[0] or interval[1] < val:
                flag += 1
        var.append(stats.variance(list1))
    if max(var) > var_threshold or flag > interval_threshold:
        if print_out:
            if not dir:
                print('Variances: {:.2f}, {:.2f}, {:.2f}'.format(var[0], var[1], var[2]))
            else:
                print('Variances: {:.2f}'.format(var[0]))
            print("Measurement Rejected")
        return False
    else:
        return True


def roll_average(list1, roll=3):
    package = []
    for direction in list1:
        temp = stats.mean(direction[-roll - 1:-1])
        package.append(round(temp, 2))
    return package
