import socket

from DroneSwarm.src.Control.FlightPathController import FlightPathController


class Drone:

    def __init__(self, number, flightpath, offset, interface_name, leader_drone):
        self.number = number
        self.controller = FlightPathController(self, flightpath, offset)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, 25, interface_name.encode())
        self.socket.bind(('', 9000))
        self.busy = False
        self.error = False
        self.leader_drone = leader_drone

    def wait(self):
        while True:
            busy = False
            for drone in drones:
                if drone.busy:
                    busy = True
                    break
            if not busy:
                break
        check_error()

    def send(self, message):
        try:
            self.socket.sendto(message.encode(), ('192.168.10.1', 8889))
            self.busy = True
        except ValueError as e:
            self.error = True
            print("Error sending \"" + message + "\" to drone #" + str(self.number) + ": " + str(e))

    def send_rc(self, u):
        tmp_u = []
        for vel in u:
            tmp_u.append(max(-100, min(100, vel)))
        u = tmp_u
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

    # def update_flightpath(self, leader_flightpath):
    #     self.flightpath = []
    #     for leader_coordinate in leader_flightpath:
    #         coordinate = [0]*4
    #         coordinate[0] = leader_coordinate[0] + self.offset[0]
    #         coordinate[1] = leader_coordinate[1] + self.offset[1]
    #         coordinate[2] = leader_coordinate[2] + self.offset[2]
    #         coordinate[3] = leader_coordinate[3] + self.offset[3]
    #         self.flightpath.append(coordinate)
    #     self.controller.trapezoid.set_target(np.array(self.flightpath[0]))

    # def flight(self):
    #     target_index = 0
    #     interval = 0.2  # 200 ms
    #     previous_time = time.time()
    #     while True:
    #         if not self.completed_flightpath:
    #             if self.trapezoid.reached:
    #                 if target_index < len(self.flightpath) - 1:
    #                     target_index += 1
    #                     self.trapezoid.set_target(np.array(self.flightpath[target_index]))
    #                 else:
    #                     target_index = 0
    #                     self.completed_flightpath = True
    #                     u = [0, 0, 0, 0]
    #                     self.send_rc_command(u)
    #             else:
    #                 now = time.time()
    #                 dt = now - previous_time
    #                 if dt > interval:
    #                     previous_time = now
    #                     # TODO: ask new position from ArUco and pass along to Trapezoid controller
    #                     u = self.trapezoid.calculate()
    #                     self.send_rc_command(u)
    #                     self.trapezoid.update_position_estimate(dt, *u)  # TODO: Remove when Trapezoid receives position from ArUco
    #                     print("Position: ", self.trapezoid.position, "Target: ", self.trapezoid.target, "Control: ", u)

    # def fetch_new_flightpath(self):
    #     # TODO: fetch new flightpath from ArUco
    #     return None
