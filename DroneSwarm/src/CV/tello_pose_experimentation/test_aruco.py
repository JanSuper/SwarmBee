# Script to test ArUco detection. Drone will not take off

import time
import numpy as np
from threading import Thread
from multiprocessing import Process

from DroneSwarm.src.Swarm.drone import Drone


def receive():
    while True:
        drone.receive()


def send_messages(messages):
    for message in messages:
        print(f"Sending message: {message}")
        drone.send(message)
        if message != "land":
            while drone.busy:
                pass


def fetch_position():
    previous_time = time.time()
    interval = 0.1
    while True:
        received = drone.sender.recv()
        if received is not None:
            marker_id = received[4]
            current_position = received[:4]
            current_position = np.rint(np.array(current_position)).astype(int)

            current_time = time.time()
            if current_time - previous_time > interval:
                print(f"Current position: {current_position}; marked_id: {marker_id}")
                previous_time = current_time


drone = Drone(1, [], [0, 0, 0, 0], 'wlxd03745f79670', None, 11111)

receiveThread = Thread(target=receive)
receiveThread.daemon = False
receiveThread.start()

send_messages(["command", "battery?", "streamoff", "streamon"])

p = Process(target=fetch_position)
p.start()
drone.aruco.start()
