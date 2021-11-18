import numpy as np
import cv2
import time
import cv2.aruco as aruco
import socket
from tello_pose_experimentation.lib.tello import Tello

# IP and port of Tello
tello_address = ('192.168.10.1', 8889)
# Create a UDP connection that we'll send the command to
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Let's be explicit and bind to a local port on our machine where Tello can send messages
sock.bind(('', 9000))
tello = Tello()

COMMAND_IN_PROGRESS = False


def send(message):
  try:
    sock.sendto(message.encode(), tello_address)
    print("Sending message: " + message)
  except Exception as e:
    print("Error sending: " + str(e))

# Function that listens for messages from Tello and prints them to the screen
def receive():
  try:
    response, ip_address = sock.recvfrom(128)
    print("Received message: " + response.decode(encoding='utf-8') + " from Tello with IP: " + str(ip_address))
  except Exception as e:
    print("Error receiving: " + str(e))

# Initiate tello connection
send("command")

time.sleep(3)

send("takeoff")

#send("land")

# test aruco code from here

aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()
send("streamon")
#cap = cv2.VideoCapture('udp://0.0.0.0:11111') # use tello onboard camera instead of webcam

while(True):
    # Capture frame-by-frame
    # ret, frame = cap.read()

    frame = tello.get_frame_read().frame

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

    for index, id in np.ndenumerate(ids):
        if id == 3 and not COMMAND_IN_PROGRESS:
            send("flip f")
            COMMAND_IN_PROGRESS = True

    # Display the resulting frame
    cv2.imshow('frame', markers)

    time.sleep(.100)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
frame.cap.release()
cv2.destroyAllWindows()

