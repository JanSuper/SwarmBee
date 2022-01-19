import time
import numpy as np
import pandas as pd
from threading import Thread
from DroneSwarm.src.Swarm.drone import Drone
import DroneSwarm.src.Utilities.KeyPressModule as KeyPress


def receive(sending_drone):
    while True:
        sending_drone.receive()


def send(receiving_drone, message):
    receiving_drone.send(message)
    messages_without_wait = ["land", "battery?"]
    if message not in messages_without_wait:
        while receiving_drone.busy:
            pass


def stop_program(error_drone):
    send(error_drone, "land")  # Might throw error
    error_drone.socket.close()
    exit()


def fetch_info_from_aruco(sending_drone):
    previous_time = time.time()
    while True:
        received = sending_drone.sender.recv()
        print(received)
        if received is not None:
            if not drone.start:
                drone.start = True
            if not drone.dummy:
                current_time = time.time()
                if current_time - previous_time > 0.1:
                    elapsed_time = current_time - drone.starting_time
                    drone.elapsed_times.append(elapsed_time)
                    drone.current_position = np.rint(np.array(received[:4])).astype(int).tolist()
                    drone.positions.append(drone.current_position)
                    drone.targets.append(drone.current_target)
                    previous_time = current_time


def force_land(receiving_drone):
    KeyPress.init()
    while True:
        if KeyPress.get_key("s"):
            methods = ["Internal"] * len(receiving_drone.elapsed_times)
            df = pd.DataFrame([receiving_drone.elapsed_times,
                               receiving_drone.positions,
                               receiving_drone.targets,
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
        if receiving_drone.dummy:
            current_time_ = time.time()
            if current_time_ - previous_time_ > 5:
                send(receiving_drone, "battery?")
                previous_time_ = current_time_


drone = Drone(1, None, [], [], "wlxd03745f79670", 11111, None)

receive_thread = Thread(target=receive, args=(drone,))
receive_thread.daemon = True
receive_thread.start()

force_land_thread = Thread(target=force_land, args=(drone,))
force_land_thread.daemon = True
force_land_thread.start()

dummy_thread = Thread(target=send_dummy_command, args=(drone,))
dummy_thread.daemon = True
dummy_thread.start()

send(drone, "command")
send(drone, "streamoff")
send(drone, "streamon")
send(drone, "takeoff")
drone.dummy = True

drone.aruco.start()
initial_position = None
while initial_position is None:
    initial_position = drone.sender.recv()
initial_position = np.rint(np.array(initial_position[:4])).astype(int)
print(f"Drone #{drone.number}: initial position {initial_position}")

position_thread = Thread(target=fetch_info_from_aruco, args=(drone,))

while not drone.start:
    pass

drone.starting_time = time.time()
fly_commands = ["forward 100", "right 100", "back 100", "left 100"]
flight_path = [[100, 0, 0, 0], [0, -100, 0, 0], [-100, 0, 0, 0], [0, 100, 0, 0]]
for fly_command in fly_commands:
    drone.current_target = (np.array(drone.current_position) + np.array(flight_path.pop(0))).tolist()
    if drone.dummy:
        drone.dummy = False
    send(drone, fly_command)
