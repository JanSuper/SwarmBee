import time
import socket
import numpy as np
from multiprocessing import Process, Pipe
from threading import Thread

from DroneSwarm.src.CV.tello_pose_experimentation.ArucoLoc import detect
from DroneSwarm.src.Swarm.drone import Drone
import DroneSwarm.src.Utilities.KeyPressModule as KeyPress


def send_messages(messages):
    messages_without_replies = ["land", "streamoff", "streamon"]
    for message in messages:
        print(f"Sending message: {message}")
        drone.send(message)
        if message not in messages_without_replies:
            while drone.busy:
                pass


def fetch_position():
    initial_position = True
    initial_flight = True
    while True:
        position = drone.sender.recv()
        tmp = []
        for coordinate in position:
            tmp.append(int(coordinate))
        position = tmp
        if position is not None:
            if initial_position:
                print(f"Initial position: {position}")
                initial_position = False
                print("Creating controller")
                temp_process = Process(target=drone.create_controller, args=(position, ))
                temp_process.start()
            else:
                if drone.controller is not None:
                    if initial_flight:
                        print("Controller created")
                        temp_process.join()
                        initial_flight = False
                        print("Starting flight")
                        # fly_thread = Thread(target=drone.controller.fly_trapezoid)
                        # fly_thread.daemon = False
                        # fly_thread.start()
                    else:
                        print(f"Current position: {position}")
                        if drone.controller.need_new_position:
                            drone.controller.trapezoid.set_position(position)
                            drone.controller.need_new_position = False


def force_land():
    KeyPress.init()
    while True:
        if KeyPress.get_key("q"):
            drone.controller.completed_flightpath = True
            drone.send_rc([0, 0, 0, 0])
            send_messages(["land"])


# def fetch_position(senders):
#     no_drones = len(senders)
#     initial = [True] * no_drones
#     if no_drones > 1:
#         initial = False
#     while True:
#         for i in range(no_drones):
#             position = senders[i].recv()
#             if position is not None:
#                 if initial[i]:
#                     print(f"Drone #{i+1}: {position} (initial)")
#                     initial[i] = False
#                 else:
#                     print(f"Drone #{i+1}: {position}")


emergencyThread = Thread(target=force_land)
emergencyThread.daemon = True
emergencyThread.start()

drone = Drone(1, [[50, 0, 0, 0]], [0, 0, 0, 0], 'wlxd03745f79670', None, 11111)

receiveThread = Thread(target=drone.receive)
receiveThread.daemon = False
receiveThread.start()

send_messages(["command", "streamoff", "streamon"])

p = Process(target=fetch_position)
p.start()
drone.aruco.start()

# drone1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# drone1.setsockopt(socket.SOL_SOCKET, 25, 'wlxd03745f79670'.encode())
# drone1.bind(('', 9000))
# drones.append(drone1)
#
# drone2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# drone2.setsockopt(socket.SOL_SOCKET, 25, 'wlxd0374572e205'.encode())
# drone2.bind(('', 9000))
# drones.append(drone2)
#
# no_drones = len(drones)
# senders = []
# processes = []
# udp_ports = [11111]  # 11113
# for i in range(no_drones):
#     drone = drones[i]
#     drone.send_busy("command")
#     # send(drone, "command")
#     # time.sleep(1)
#     drone.send_busy("streamoff")
#     drone.send_busy("streamon")
#     # send(drone, "streamoff")
#     # send(drone, "streamon")
#
#     receiver, sender = Pipe()
#     senders.append(sender)
#     p = Process(target=detect, args=(receiver, udp_ports[i], i+1, ))
#     processes.append(p)
#
# p = Process(target=fetch_position, args=(senders,))
# processes.append(p)
#
# for process in processes:
#     process.start()
#
# commands = ["takeoff", "forward 100", "cw 180", "forward 100", "cw 180", "land"]
# sleep_values = [10, 5, 5, 5, 5, 0]
# for i in range(len(commands)):
#     for drone in drones:
#         send(drone, commands[i])
#     time.sleep(sleep_values[i])
#
# for process in processes:
#     process.join()
