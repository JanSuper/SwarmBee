import threading
from DroneSwarm.src.Swarm.drone import Drone


def send(message):
    print("\nSending message: " + message)
    for drone in drones:
        drone.send(message)
    if message != "land":
        wait()


def receive():
    while True:
        for drone in drones:
            drone.receive()


def wait():
    while True:
        busy = False
        for drone in drones:
            if drone.busy:
                busy = True
                break
        if not busy:
            break
    check_error()


def check_error():
    for drone in drones:
        if drone.error:
            print("An error occurred: killing program")
            land()
            close_sockets()
            exit()


def close_sockets():
    for drone in drones:
        drone.socket.close()


def takeoff():
    send("command")
    send("battery?")
    send("streamon")
    send("streamoff")
    send("takeoff")


def land():
    send("land")


def update_flightpath(leader_flightpath):
    leader_drone.flightpath = leader_flightpath
    for follower_drone in follower_drones:
        follower_drone.update_flightpath(leader_flightpath)


def flight():
    for drone in drones:
        thread = threading.Thread(target=drone.flight())
        thread.daemon = True
        thread.start()


initial_positions = [[0, 0, 0, 0], [0, 0, 0, 0]]
offsets = [[0, 0, 0, 0], [0, 0, 0, 0]]
interface_names = ['wlxd0374572e205', 'wlxd03745f79670']
initial_leader_flightpath = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]

drones = []
for i in range(len(initial_positions)):
    drones.append(Drone(i+1, initial_positions[i], offsets[i], interface_names[i]))
no_drones = len(drones)
leader_drone = drones[0]
follower_drones = drones[1:]
update_flightpath(initial_leader_flightpath)

receiveThread = threading.Thread(target=receive)
receiveThread.daemon = True
receiveThread.start()

takeoff()

flight()

land()

close_sockets()
