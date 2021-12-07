import socket
import time

from DroneSwarm.src.Control.Trapezoid import Trapezoid


class Drone:

    def __init__(self, number, initial_position, offset, interface_name):
        self.number = number
        self.trapezoid = Trapezoid()
        self.trapezoid.set_position(initial_position)
        self.offset = offset
        self.flightpath = []
        self.completed_flightpath = False
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, 25, interface_name.encode())
        self.socket.bind(('', 9000))
        self.busy = False
        self.error = False

    def send(self, message):
        try:
            self.socket.sendto(message.encode(), ('192.168.10.1', 8889))
            self.busy = True
        except ValueError as e:
            self.error = True
            print("Error sending \"" + message + "\" to drone #" + str(self.number) + ": " + str(e))

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

    def update_flightpath(self, leader_flightpath):
        self.flightpath = []
        for leader_coordinate in leader_flightpath:
            coordinate = [0]*4
            coordinate[0] = leader_coordinate[0] + self.offset[0]
            coordinate[1] = leader_coordinate[1] + self.offset[1]
            coordinate[2] = leader_coordinate[2] + self.offset[2]
            coordinate[3] = leader_coordinate[3] + self.offset[3]
            self.flightpath.append(coordinate)
        self.trapezoid.set_target(self.flightpath[0])

    def flight(self):
        target_index = 0
        interval = 0.2  # 20 ms
        previous_time = time.time()
        while True:
            if not self.completed_flightpath:
                if self.trapezoid.reached:
                    if target_index < len(self.flightpath) - 1:
                        target_index += 1
                        self.trapezoid.set_target(self.flightpath[target_index])
                    else:
                        target_index = 0
                        self.completed_flightpath = True
                        u = [0, 0, 0, 0]
                        # TODO: send u to drone
                now = time.time()
                dt = now - previous_time
                if dt > interval:
                    # TODO: ask new position from ArUco and pass along to Trapezoid controller
                    u = self.trapezoid.calculate()
                    # TODO: send u to drone
                    self.trapezoid.update_position_estimate(dt, *u)

    def fetch_new_flightpath(self):
        # TODO: fetch new flightpath from ArUco
        return None
