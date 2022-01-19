import socket
import time

import numpy as np
import pandas as pd
from threading import Thread
from multiprocessing import Pipe, Process
import DroneSwarm.src.Utilities.KeyPressModule as KeyPress
from DroneSwarm.src.CV.tello_pose_experimentation.ArucoLoc import detect


def receive(sending_drone):
    while True:
        try:
            response, ip_address = sending_drone.recvfrom(128)
            decoded_response = response.decode(encoding='utf-8').strip()
            print(f"Received message from drone: {decoded_response}")
            if decoded_response != "":
                busy = False
                if "error" in decoded_response:
                    stop_program(sending_drone)
                    break
        except ValueError as e:
            print(f"Error receiving message from drone: {str(e)}")
            stop_program(sending_drone)


def send(receiving_drone, message):
    print(f"Sending message \"{message}\"")
    messages_without_busy = ["land", "battery?"]
    try:
        receiving_drone.sendto(message.encode(), ('192.168.10.1', 8889))
        if message not in messages_without_busy:
            busy = True
            while busy:
                pass
    except ValueError as e:
        print("Error sending \"" + message + "\" to drone #: " + str(e))
        stop_program(receiving_drone)


def stop_program(error_drone):
    send(error_drone, "land")  # Might throw error
    error_drone.close()
    exit()


def fetch_info_from_aruco(aruco_sender):
    previous_time = time.time()
    while True:
        received = aruco_sender.recv()
        if received is not None:
            if not start:
                start = True
            if flight:
                current_time = time.time()
                if current_time - previous_time > 0.1:
                    elapsed_time = current_time - starting_time
                    elapsed_times.append(elapsed_time)
                    current_position = np.rint(np.array(received[:4])).astype(int).tolist()
                    positions.append(current_position)
                    targets.append(current_target)
                    previous_time = current_time


def force_land(receiving_drone):
    KeyPress.init()
    while True:
        if KeyPress.get_key("s"):
            methods = ["Internal"] * len(elapsed_times)
            df = pd.DataFrame([elapsed_times,
                               positions,
                               targets,
                               methods]).T
            df.columns = ['elapsed_time', 'position', 'target', 'method']
            df.to_csv(f'FlightDataInternal.csv')

            print("Soft landing initialized")
            # Land all drones
            send(receiving_drone, "land")
            break


def send_dummy_command(receiving_drone):
    previous_time_ = time.time()
    while True:
        if dummy:
            current_time_ = time.time()
            if current_time_ - previous_time_ > 5:
                send(receiving_drone, "battery?")
                previous_time_ = current_time_


elapsed_times = []
current_position = [0, 0, 0, 0]
positions = []
current_target = [0, 0, 0, 0]
targets = []

drone = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
drone.setsockopt(socket.SOL_SOCKET, 25, 'wlxd03745f79670'.encode())
drone.bind(('', 9000))
busy = False


receive_thread = Thread(target=receive, args=(drone,))
receive_thread.daemon = True
receive_thread.start()

force_land_thread = Thread(target=force_land, args=())
force_land_thread.daemon = True
force_land_thread.start()

dummy = False
dummy_thread = Thread(target=send_dummy_command, args=(drone,))
dummy_thread.daemon = True
dummy_thread.start()

send(drone, "command")
send(drone, "streamoff")
send(drone, "streamon")
send(drone, "takeoff")
dummy = True

receiver, sender = Pipe()
aruco = Process(target=detect, args=(receiver, 11111, 1))
aruco.start()

start = False
flight = False
position_thread = Thread(target=fetch_info_from_aruco, args=(sender,))

while not start:
    pass

starting_time = time.time()
fly_commands = ["forward 100", "right 100", "back 100", "left 100"]
flight_path = [[100, 0, 0, 0], [0, -100, 0, 0], [-100, 0, 0, 0], [0, 100, 0, 0]]
for fly_command in fly_commands:
    current_target = (np.array(current_position) + np.array(flight_path.pop(0))).tolist()
    if not flight:
        flight = True
    if dummy:
        dummy = False
    send(drone, fly_command)
