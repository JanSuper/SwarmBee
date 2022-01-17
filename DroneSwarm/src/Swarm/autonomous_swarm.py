# This file uses DroneBlocks' UDP send-receiver example code as a base
# Source: https://github.com/dbaldwin/DroneBlocks-Tello-Python/blob/master/lesson3-udp-send-receive/UDPSendReceive.py

import time
from multiprocessing import Process
import numpy as np
from threading import Thread

from DroneSwarm.src.Swarm.drone import Drone
import DroneSwarm.src.Utilities.KeyPressModule as KeyPress
# from DroneSwarm.src.CV.HandTracking.HandTrackingModule import detect_gesture


def force_land():
    KeyPress.init()
    while True:
        if KeyPress.get_key("s"):
            print("Soft landing initialized")
            # Make all drones stationary
            make_drones_stationary()
            # Land all drones
            send("land")
            break
        if KeyPress.get_key("e"):
            print("Emergency landing initialized")
            # Make all drones stationary
            make_drones_stationary()
            # Stop all drones' motors immediately
            send("emergency")
            break


def make_drones_stationary():
    for drone in drones:
        drone.controller.completed_flightpath = True
        drone.send_rc([0, 0, 0, 0])


def send(message):
    messages_without_wait = ['land', 'emergency']
    print(f"Sending message: {message}")
    for drone in drones:
        drone.send(message)
    if message not in messages_without_wait:
        wait()


def receive():
    while True:
        for drone in drones:
            drone.receive()


def wait():
    while True:
        busy = False
        for drone in drones:
            if drone.busy:
                busy = True
                break
        if not busy:
            break
    check_error()


def check_error():
    for drone in drones:
        if drone.error:
            print("An error occurred: killing program")
            stop_program()


def stop_program():
    make_drones_stationary()
    send("land")
    for drone in drones:
        drone.socket.close()
    exit()


def setup_drone(drone):
    # Perform takeoff
    messages = ["command", "streamoff", "streamon", "takeoff"]
    for message in messages:
        print(f"Sending message to drone #{drone.number}: {message}")
        drone.send(message)
        while drone.busy:
            pass

    # Start drone's ArUco process
    drone.aruco.start()

    # Wait for drone's initial position to be known
    initial_position = None
    while initial_position is None:
        initial_position = drone.sender.recv()
    initial_position = np.rint(np.array(initial_position[:4])).astype(int)
    print(f"Drone #{drone.number}: initial position {initial_position}")

    if drone.number > 1:
        # For follower drones only; create initial flight path
        initial_target = leader_drone.controller.initial_position + drone.offset
        initial_flightpath = [initial_target - initial_position]
        drone.flightpath = initial_flightpath
        print(f"Drone #{drone.number}: initial flight path {drone.flightpath}")

    # Create drone's controller
    drone.create_controller(initial_position, method)

    print(f"Drone #{drone.number} is ready for flight")


def fetch_info_from_aruco():
    previous_positions = {}

    for drone in drones:
        previous_positions[drone] = drone.controller.initial_position

    while True:
        for drone in drones:
            received = drone.sender.recv()
            if received is not None:
                marker_id = received[4]
                current_position = np.rint(np.array(received[:4])).astype(int)

                previous_position = previous_positions.get(drone)
                deltas = np.absolute(current_position - previous_position)
                if any(deltas[:] > 50):
                    # print(f"(ArUco) Drone #{drone.number}: averaged {current_position} and {previous_position}")
                    current_position = (current_position + previous_position) / 2
                previous_positions[drone] = current_position
                # print(f"(ArUco) Drone #{drone.number}: current position = {current_position}")

                if drone.controller.need_new_position:
                    drone.controller.current_position = current_position
                    drone.controller.need_new_position = False

                if drone.controller.need_new_flightpath:
                    # Can only trigger for leader drone
                    # drone.controller.update_flightpath(get_aruco_flightpath(marker_id), method)  # TODO: enable
                    # drone.controller.need_new_flightpath = False
                    pass


# marker groups for pre-defined flight paths
#  30  31  32    33  34  35  66    67  68  69
#   0   1   2     3   4   5  36    37  38  39
#   6   7   8     9  10  11  42    43  44  45

#  12  13  14    15  16  17  48    49  50  51
#  18  19  20    21  22  23  54    55  56  57
#  24  25  26    27  28  29  60    61  62  63


def get_aruco_flightpath(marker_id):
    aruco_flightpaths = [
        [[125, 125, 0, 0]],
        [[-125, 125, 0, 0]],
        [[0, 125, 0, 0]],
        [[0, -125, 0, 0]],
        [[125, -125, 0, 0]],
        [[-125, -125, 0, 0]]
    ]
    groups = [[12, 13, 14, 18, 19, 20, 24, 25, 26],
              [15, 16, 17, 48, 21, 22, 23, 54, 27, 28, 29, 60],
              [49, 50, 51, 55, 56, 57, 61, 62, 63],
              [30, 31, 32, 0, 1, 2, 6, 7, 8],
              [33, 34, 35, 66, 3, 4, 5, 36, 9, 10, 11, 42],
              [67, 68, 69, 37, 38, 39, 43, 44, 45]]
    marker_group = -1
    for i in range(len(groups)):
        for j in range(len(groups[i])):
            if groups[i][j] == marker_id:
                marker_group = i
                break
    return aruco_flightpaths[marker_group]


def start_flying():
    for drone in drones:
        thread = Thread(target=drone.controller.fly_you_fool, args=(method,))
        thread.daemon = True
        thread.start()


def monitor():
    previous_time = time.time()
    follower_flightpath_update_interval = 0.5

    while True:
        if leader_drone.controller.completed_flightpath:
            # Leader drone has finished its flightpath
            all_drones_completed_flightpath = True
            for follower_drone in drones[1:]:
                if not follower_drone.controller.completed_flightpath:
                    all_drones_completed_flightpath = False
                    break

            if all_drones_completed_flightpath:
                print(f"Need new flightpath")
                # Update leader drone's flightpath
                leader_drone.controller.need_new_flightpath = True
                while leader_drone.controller.need_new_flightpath:
                    pass
                leader_drone.controller.completed_flightpath = False
        else:
            # No drone has completed their flightpath
            current_time = time.time()
            if current_time - previous_time > follower_flightpath_update_interval:
                # Add a new target to each follower's flightpath
                leader_current_position = leader_drone.controller.current_position
                for follower_drone in drones[1:]:
                    if not follower_drone.controller.flight_interrupted:
                        new_follower_target = leader_current_position + follower_drone.controller.offset
                        follower_drone.controller.flightpath.append(new_follower_target.tolist())
                        follower_drone.controller.completed_flightpath = False

                previous_time = current_time
        check_error()


def send_dummy_command():
    previous_time = time.time()
    interval = 5
    dummy_command = "battery?"

    while True:
        current_time = time.time()
        if current_time - previous_time > interval:
            for drone in drones:
                if not setup_done:
                    send_dummy = True
                else:
                    send_dummy = drone.controller.completed_flightpath

                if send_dummy:
                    # print(f"Drone #{drone.number}: sending dummy command \"{dummy_command}\"")
                    drone.send_dummy_command(dummy_command)
            previous_time = current_time


# Control parameters
method = "Proportional"  # Trapezoid, Proportional, Circle
no_drones = 2
leader_bluetooth_address = '84:CC:A8:2F:E9:32'  # EDAD = 84:CC:A8:2F:E9:32, EDB0 = 84:CC:A8:2E:9C:B6,
# 60FF = 9C:9C:1F:E1:B0:62
leader_initial_flightpath = []  # [200, 0, 0, 0], [0, 150, 0, 0], [-200, 0, 0, 0], [0, -150, 0, 0]
follower_offsets = [[-50, 0, 0, 0]]

# Setup forced landing
force_land_thread = Thread(target=force_land)
force_land_thread.daemon = True
force_land_thread.start()

# WARNING: Make sure leader drone connects to interface_names[0]
udp_ports = [11111, 11113, 11115]
udp_ports = udp_ports[:no_drones]
interface_names = ['wlxd03745f79670', 'wlxd0374572e205', 'wlx6c5ab04a495e']
interfaces_names = interface_names[:no_drones]

drones = []
# Create leader drone
leader_drone = Drone(1, None, leader_initial_flightpath, np.array([0, 0, 0, 0]), interfaces_names.pop(0),
                     udp_ports.pop(0), leader_bluetooth_address)
drones.append(leader_drone)

# Enable receiver
receive_thread = Thread(target=receive)
receive_thread.daemon = True
receive_thread.start()

setup_done = False
dummy_thread = Thread(target=send_dummy_command)
dummy_thread.daemon = True
dummy_thread.start()

# Setup leader drone
setup_drone(leader_drone)

# Create follower drones
drone_number = 2
while drone_number <= no_drones:
    follower_offset = np.array(follower_offsets.pop(0))
    follower_drone = Drone(drone_number, leader_drone, None, follower_offset, interfaces_names.pop(0),
                           udp_ports.pop(0), None)
    drones.append(follower_drone)
    drone_number += 1

# Setup follower drones
for follower_drone in drones[1:]:
    setup_drone(follower_drone)

position_thread = Thread(target=fetch_info_from_aruco)
position_thread.daemon = True
position_thread.start()

setup_done = True
print("Setup done")

# Get followers into formation
# leader_drone.controller.completed_flightpath = True
# start_flying()
# in_formation = False
# while not in_formation:
#     in_formation = True
#     for follower_drone in drones[1:]:
#         if not follower_drone.controller.completed_flightpath:
#             in_formation = False
#             break
# print(f"Swarm is in formation")

# Start hand-tracking module (currently not working)
# tracking_process = Process(target=detect_gesture, args=(drones, True, False))
# tracking_process.start()

# Start proper flight
# print(f"Start flight")
# leader_drone.controller.completed_flightpath = False
# monitor()
