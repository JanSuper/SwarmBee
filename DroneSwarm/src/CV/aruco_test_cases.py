# copy this import to the autonomous_swarm.py:
# import DroneSwarm.src.CV.aruco_test_cases as ArucoTestCases

# use the following to give test flightpath to drone
# leader_initial_flightpath = ArucoTestCases.test_x[0]

test_x = [[25,0,0,0],[50,0,0,0],[100,0,0,0],[150,0,0,0],[200,0,0,0],[225,0,0,0],[300,0,0,0]]
test_y = [[0,25,0,0],[0,50,0,0],[0,100,0,0],[0,150,0,0],[0,200,0,0],[0,225,0,0]]
test_z = [[0,0,25,0],[0,0,50,0],[0,0,100,0],[0,0,-25,0],[0,0,-50,0],[0,0,-100,0]]
test_xy = [[100,100,0,0],[100,200,0,0],[200,100,0,0],[50,125,0,0],[125,50,0,0]]
test_xyz = [[100,100,100,0],[100,200,50,0],[200,100,50,0],[50,125,100,0],[125,50,100,0]]