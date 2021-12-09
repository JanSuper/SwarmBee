# How to run the code
Before doing anything, please install the required packages from the requirements.txt file.

As the modules have not been merged yet, there is no single file that runs everything.

Instead you can run each module individually:
1. Flight path controller: control/FlightPathController.py
2. ArUco detection: CV/tello_pose_experimentation/detect_position_from_marker.py
3. Swarm flight: Swarm/simple_swarm.py

To change the target position of the flight path controller, the numpy array on line 66 in 'control/FlightPathController.py' must be edited. The format is [x, y, z, yaw].

To change the set of commands that is executed by the swarm, the list on line 107 in 'Swarm/simple_swarm.py' must be edited. Any command from the Tello SDK works. An overview of all commands can be found here: https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf

The code in 'Swarm/simple_swarm.py' is made to work with two drones, so two WiFi-adapters are required. You must make sure that the interface names on lines 16 and 18 in 'Swarm/simple_swarm.py' match those of your WiFi-adapters. Simply replace 'wlxd0374572e205' and 'wlxd03745f79670' with what your Terminal outputs. We recommend using Linux, as it makes it easy to find WiFi interface names.
