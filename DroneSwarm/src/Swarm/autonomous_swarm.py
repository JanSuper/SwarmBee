# This file uses DroneBlocks' UDP send-receiver example code as a base
# Source: https://github.com/dbaldwin/DroneBlocks-Tello-Python/blob/master/lesson3-udp-send-receive/UDPSendReceive.py

import numpy as np
from threading import Thread

from DroneSwarm.src.Swarm.drone import Drone
import DroneSwarm.src.Utilities.KeyPressModule as KeyPress


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
        thread = Thread(target=drone.controller.fly_trapezoid)
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
    KeyPress.init()
    while True:
        if KeyPress.get_key("s"):
            print("Soft landing initialized")
            # Make all drones stationary
            make_drones_stationary()
            # Land all drones
            send("land")
        if KeyPress.get_key("e"):
            print("Emergency landing initialized")
            # Make all drones stationary
            make_drones_stationary()
            # Stop all drones' motors immediately
            send("emergency")


def make_drones_stationary():
    for drone in drones:
        drone.controller.completed_flightpath = True
        drone.send_rc([0, 0, 0, 0])


def setup_drone(drone):
    # Perform takeoff
    messages = ["command", "battery?", "streamoff", "streamon", "takeoff"]
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
    initial_position = np.array(initial_position)

    # Create drone's controller
    drone.create_controller(initial_position)


# Setup forced landing
force_land_thread = Thread(target=force_land)
force_land_thread.daemon = True
force_land_thread.start()

interface_names = ['wlxd03745f79670', 'wlxd0374572e205']
udp_ports = [11111, 11113]
drones = []
# Create leader drone
leader_initial_flightpath = []
leader_drone = Drone(1, leader_initial_flightpath, [0, 0, 0, 0], interface_names.pop(0), None, udp_ports.pop(0))
drones.append(leader_drone)

# Enable receiver
receive_thread = Thread(target=receive)
receive_thread.daemon = True
receive_thread.start()

# Setup leader drone
setup_drone(leader_drone)

# Create follower drones
follower_offsets = [[-50, 0, 0, 0]]
no_drones = len(interface_names)
drone_number = 2
while drone_number <= no_drones:
    follower_offset = np.array(follower_offsets.pop(0))
    follower_flightpath = [[leader_drone.controller.initial_position + follower_offset]]
    follower_drone = Drone(drone_number, follower_flightpath, follower_offset, interface_names.pop(0), leader_drone,
                           udp_ports.pop(0))
    drones.append(follower_drone)
    drone_number += 1

# Setup follower drones
for follower_drone in drones[1:]:
    setup_drone(follower_drone)

# Start flying the swarm
flight()

# Monitor the swarm
monitor()
