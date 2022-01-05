# This file uses DroneBlocks' UDP send-receiver example code as a base
# Source: https://github.com/dbaldwin/DroneBlocks-Tello-Python/blob/master/lesson3-udp-send-receive/UDPSendReceive.py

import threading
import numpy as np
from DroneSwarm.src.Swarm.drone import Drone
import DroneSwarm.src.Utilities.KeyPressModule as kp


def send(message):
    print("\nSending message: " + message)
    for drone in drones:
        drone.send(message)
    if message != "land":
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
    send("land")
    for drone in drones:
        drone.socket.close()
    exit()


def takeoff():
    send("command")
    send("battery?")
    send("streamon")
    send("streamoff")
    send("takeoff")


def update_flightpath(leader_flightpath):
    leader_drone.flightpath = leader_flightpath
    leader_drone.controller.trapezoid.set_target(np.array(leader_flightpath[0]))
    for follower_drone in follower_drones:
        follower_drone.update_flightpath(leader_flightpath)
    for drone in drones:
        drone.completed_flightpath = False


def flight():
    for drone in drones:
        thread = threading.Thread(target=drone.flight())
        thread.daemon = True
        thread.start()


def monitor():
    while True:
        new_flightpath = True
        for drone in drones:
            if not drone.completed_flightpath:
                new_flightpath = False
                break
        if new_flightpath:
            print("NEED NEW FLIGHTPATH")
            # new_leader_flightpath = leader_drone.fetch_new_flightpath()
            # update_flightpath(new_leader_flightpath)
        check_error()


def force_land():
    kp.init()
    while True:
        if kp.get_key("q"):
            # Make all drones stationary
            for drone in drones:
                drone.controller.completed_flightpath = True
                drone.send_rc([0, 0, 0, 0])
            # Stop the program
            stop_program()


# Setup emergency landing
receiveThread = threading.Thread(target=force_land)
receiveThread.daemon = True
receiveThread.start()

# Initialize the swarm
initial_leader_flightpath = []
interface_names = ['wlxd03745f79670', 'wlxd0374572e205']
follower_offsets = []
no_drones = len(interface_names)
drones = []

# Directions:
# forward = y positive
# backward = y negative
# left = x negative
# right = x positive

# Create leader drone
leader_drone = Drone(1, initial_leader_flightpath, [0, 0, 0, 0], interface_names[0], None)
drones.append(leader_drone)

# Wait for leader drone's initial position to be known
while leader_drone.controller.aruco.initial_position is None:
    pass
initial_leader_position = leader_drone.controller.aruco.initial_position

# Create follower drones
for index in range(no_drones-1):
    follower_offset = follower_offsets[index]
    follower_flightpath = [initial_leader_position + follower_offset]
    drones.append(Drone(index+2, follower_flightpath, follower_offset, interface_names[index+1], leader_drone))
follower_drones = drones[1:]

print(f"Number of drones = {no_drones}")
for drone in drones:
    print(f"Drone #{drone.number}: initial position = {drone.controller.aruco.initial_position}; offset = "
          f"{drone.controller.offset}; initial target ={drone.controller.flightpath[0]}")

# Create thread to receive information from the drones
receiveThread = threading.Thread(target=receive)
receiveThread.daemon = True
receiveThread.start()


takeoff()

flight()

monitor()
