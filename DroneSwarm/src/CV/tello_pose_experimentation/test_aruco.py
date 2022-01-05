import socket
import time
from lib.tello import Tello

from DroneSwarm.src.CV.tello_pose_experimentation.ArucoLoc import ArucoLoc


def send(receiver, message):
    try:
        receiver.sendto(message.encode(), ('192.168.10.1', 8889))
    except ValueError:
        print("Error sending \"" + message + "\" to drone")


drones = []

drone = Tello()
drones.append(drone)

# drone1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# drone1.setsockopt(socket.SOL_SOCKET, 25, 'wlxd03745f79670'.encode())
# drone1.bind(('', 9000))
# drones.append(drone1)

# drone2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# drone2.setsockopt(socket.SOL_SOCKET, 25, 'wlxd0374572e205'.encode())
# drone2.bind(('', 9000))
# drones.append(drone2)

aruco_threads = []
for drone in drones:
    # send(drone, "command")
    drone.send("command")
    time.sleep(1)
    # send(drone, "streamoff")
    # send(drone, "streamon")
    drone.send("streamoff")
    drone.send("streamon")
    aruco_thread = ArucoLoc()
    aruco_thread.start()
    aruco_threads.append(aruco_thread)

# for aruco_thread in aruco_threads:
#     aruco_thread.start()

initial = True
interval = 0.5
previous_time = time.time()
while True:
    current_time = time.time()
    if current_time - previous_time > interval:
        for i in range(len(aruco_threads)):
            aruco_thread = aruco_threads[i]
            if aruco_thread.initial_position is not None:
                if initial:
                    print(f"Initial position drone #{i+1}: {aruco_thread.initial_position}")
                    initial = False
                print(f"Current position drone #{i+1}: {aruco_thread.current_position}")
        previous_time = current_time
