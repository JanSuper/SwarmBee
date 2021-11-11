
import time
import numpy as np
import DroneSwarm.src.Utilities.KeyPressModule as kp
from DroneSwarm.src.Control.Trapezoid import Trapezoid
from DroneSwarm.src.Utilities.utils import *

is_flying = True

kp.init()
# connect to drone
myDrone = initialize_tello()

# initialize Trapezoid controller
trapezoid = Trapezoid()
# takeoff
if is_flying:
    myDrone.takeoff()

# set first desired position
target = np.array([150, 200, 100, 90], dtype=int)
trapezoid.set_target(target)

# TODO: run PID on separate thread
interval = 0.25  # 25 ms
previous_time = time.time()
query_time = time.time()
while True:
    now = time.time()
    dt = now - previous_time
    if dt > interval:
        u = trapezoid.calculate()
        if kp.get_key("q"):
            myDrone.land()
            break
        if kp.get_key("f"):
            myDrone.emergency()
            break
        # Send desired velocity values to the drone
        if myDrone.send_rc_control:
            myDrone.send_rc_control(*u)
        # Use the real time velocities of the drone
        y = [myDrone.get_speed_x(), myDrone.get_speed_y(), myDrone.get_speed_z(), myDrone.get_yaw()]
        trapezoid.update_position_estimate(dt, *u)
        print("Position: ", trapezoid.position, "Target: ", trapezoid.target, "Time: ", now - query_time, "\n",
              "Control: ", u, "Read: ", y)
        previous_time = now
