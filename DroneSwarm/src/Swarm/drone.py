import math
import socket
import numpy as np
import pandas as pd
from multiprocessing import Pipe

from DroneSwarm.src.Control.FlightPathController import FlightPathController
from DroneSwarm.src.CV.tello_pose_experimentation.ArucoLoc import ArucoProcess


class Drone:

    def __init__(self, number, leader_drone, flightpath, offset, interface_name, udp_port, bt_address):
        self.number = number
        self.leader_drone = leader_drone
        self.flightpath = flightpath
        self.offset = offset
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, 25, interface_name.encode())
        self.socket.bind(('', 9000))
        receiver, sender = Pipe()
        self.sender = sender
        self.udp_port = udp_port
        self.aruco = ArucoProcess(receiver, udp_port, number)
        self.bt_address = bt_address
        self.busy = False
        self.error = False
        self.method = None
        self.controller = None

        # Testing variables
        self.starting_time = None
        self.elapsed_times = []
        self.positions = []
        self.current_target = [0, 0, 0, 0]
        self.targets = []

    def update_test_variables(self, elapsed_time, current_position):
        self.elapsed_times.append(elapsed_time)
        self.positions.append(current_position.tolist())
        self.targets.append(self.current_target)

    def export_data(self):
        methods = [self.method] * len(self.elapsed_times)
        df = pd.DataFrame([self.elapsed_times,
                           self.positions,
                           self.targets,
                           methods]).T
        df.columns = ['elapsed_time', 'position', 'target', 'method']
        df.to_csv('FlightData.csv')

    def create_controller(self, initial_position, method):
        self.method = method
        self.controller = FlightPathController(self, initial_position, method=method)

    def send_no_busy(self, message):
        print(f"Drone #{self.number}: sending message \"{message}\"")
        try:
            self.socket.sendto(message.encode(), ('192.168.10.1', 8889))
        except ValueError as e:
            self.error = True
            print("Error sending \"" + message + "\" to drone #" + str(self.number) + ": " + str(e))

    def send(self, message):
        print(f"Drone #{self.number}: sending message \"{message}\"")
        try:
            self.socket.sendto(message.encode(), ('192.168.10.1', 8889))
            self.busy = True
        except ValueError as e:
            self.error = True
            print("Error sending \"" + message + "\" to drone #" + str(self.number) + ": " + str(e))

    def send_rc(self, u):
        # Scale u
        u = np.array(u)
        max_vel = 30
        quads = 0
        for vel in u:
            quads += (vel * vel)
        length_u = math.sqrt(quads)
        if length_u > 30:
            u = u * max_vel / length_u

        # Round u to the nearest integers
        u = np.rint(u).astype(int)

        command = f"rc {u[0]} {u[1]} {u[2]} {u[3]}"
        self.send_no_busy(command)

    def receive(self):
        try:
            response, ip_address = self.socket.recvfrom(128)
            decoded_response = response.decode(encoding='utf-8').strip()
            if decoded_response != "":
                self.busy = False
                if "error" in decoded_response:
                    self.error = True
            print("Received message from drone #" + str(self.number) + ": " + decoded_response)
        except ValueError as e:
            self.error = True
            print("Error receiving message from drone #" + str(self.number) + ": " + str(e))
