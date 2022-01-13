import math
import socket
import numpy as np
from multiprocessing import Pipe, Process

from DroneSwarm.src.Control.FlightPathController import FlightPathController
from DroneSwarm.src.CV.tello_pose_experimentation.ArucoLoc import detect


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
        self.aruco = Process(target=detect, args=(receiver, udp_port, number,))
        self.busy = False
        self.error = False
        self.controller = None
        self.bt_address = bt_address

    def create_controller(self, initial_position):
        self.controller = FlightPathController(self, initial_position)

    def send_dummy_command(self, dummy_command):
        try:
            self.socket.sendto(dummy_command.encode(), ('192.168.10.1', 8889))
        except ValueError as e:
            self.error = True
            print("Error sending dummy command to drone #" + str(self.number) + ": " + str(e))

    def send(self, message):
        try:
            self.socket.sendto(message.encode(), ('192.168.10.1', 8889))
            self.busy = True
        except ValueError as e:
            self.error = True
            print("Error sending \"" + message + "\" to drone #" + str(self.number) + ": " + str(e))

    def send_rc(self, u):
        u = np.array(u)
        max_vel = 30
        quads = 0
        for vel in u:
            quads += (vel * vel)
        length_u = math.sqrt(quads)
        if length_u > 30:
            u = u * max_vel / length_u
        # tmp_u = []
        # for vel in u:
        #     tmp_u.append(max(-30, min(30, vel)))
        # u = tmp_u
        command = f"rc {u[0]} {u[1]} {u[2]} {u[3]}"
        print(f"Drone #{self.number} remote control: " + command)
        try:
            self.socket.sendto(command.encode(), ('192.168.10.1', 8889))
        except ValueError as e:
            self.error = True
            print("Error sending \"" + command + "\" to drone #" + str(self.number) + ": " + str(e))

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
