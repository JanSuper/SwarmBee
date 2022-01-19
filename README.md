# How to run the code
Requirements:
- A Linux environment
- One WiFi adapter for each connected drone
- Packages from requirements.txt 

There are two runnable scripts:
- DroneSwarm/src/Swarm/autonomous_swarm.py contains the Swarm Formation and Autonomous Singular Drone Flight
- DroneSwarm/src/CV/HandTracking/HandTrackingModule.py contains the Gesture Recognition

Swarm Formation:
- Set no_drones on line 268 to 2
- Modify the list of follower offsets on line 272 at your will
- Update the list of WiFi interface names on line 282 to match yours
- Make sure the leader drone connects to interface_names[0]
- Modify DroneSwarm/src/Swarm/adapter_configuration.sh by replacing the WiFi interface names with yours
- Run the program
- Press "s" (soft landing) or "e" (emergency landing) on your keyboard to stop the program

Autonomous Singular Drone Flight:
- Set no_drones on line 268 to 1
- Modify the list of waypoints on line 271 at your will
- Update the list of WiFi interface names on line 282 to match yours
- Make sure the drone connects to interface_names[0]
- Run the program
- Press "s" (soft landing) or "e" (emergency landing) on your keyboard to stop the program

Gesture Recognition:
- Set no_drones on line 361 to whatever you desire
- Update the list of WiFi interface names on line 365 to match yours
- Run the program
