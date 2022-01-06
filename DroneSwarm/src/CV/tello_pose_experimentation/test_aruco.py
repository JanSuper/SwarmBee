import time
import socket
from multiprocessing import Process, Pipe
from DroneSwarm.src.CV.tello_pose_experimentation.ArucoLoc import detect


def send(receiving_drone, message):
    try:
        receiving_drone.sendto(message.encode(), ('192.168.10.1', 8889))
    except ValueError:
        print("Error sending \"" + message + "\" to drone")


def fetch_position(senders):
    no_drones = len(senders)
    initial = [True] * no_drones
    while True:
        for i in range(no_drones):
            position = senders[i].recv()
            if position is not None:
                if initial[i]:
                    print(f"Drone #{i+1}: {position} (initial)")
                    initial[i] = False
                else:
                    print(f"Drone #{i+1}: {position}")


drones = []

drone1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
drone1.setsockopt(socket.SOL_SOCKET, 25, 'wlxd03745f79670'.encode())
drone1.bind(('', 9000))
drones.append(drone1)

drone2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
drone2.setsockopt(socket.SOL_SOCKET, 25, 'wlxd0374572e205'.encode())
drone2.bind(('', 9000))
drones.append(drone2)

no_drones = len(drones)
senders = []
processes = []
udp_ports = [11111, 11113]
for i in range(no_drones):
    drone = drones[i]
    send(drone, "command")
    time.sleep(1)
    send(drone, "streamoff")
    send(drone, "streamon")

    receiver, sender = Pipe()
    senders.append(sender)
    p = Process(target=detect, args=(receiver, udp_ports[i], i+1, ))
    processes.append(p)

p = Process(target=fetch_position, args=(senders,))
processes.append(p)

for process in processes:
    process.start()

commands = ["takeoff", "forward 100", "cw 180", "forward 100", "cw 180", "land"]
sleep_values = [10, 5, 5, 5, 5, 0]
for i in range(len(commands)):
    for drone in drones:
        send(drone, commands[i])
    time.sleep(sleep_values[i])

for process in processes:
    process.join()
