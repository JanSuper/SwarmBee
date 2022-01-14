import cv2
import mediapipe as mp
import time
import math
import imutils
import socket

from queue import Queue


class handDetector():

    def __init__(self, mode=False, maxHands=1, modelComplexity=1, detectionCon=0.5, trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.modelComplex = modelComplexity
        self.detectionCon = detectionCon
        self.trackCon = trackCon
        self.mpDraw = mp.solutions.drawing_utils
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.modelComplex, self.detectionCon,
                                        self.trackCon)
        self.tipIds = [4, 8, 12, 16, 20]
        self.fingers = []
        self.rectSizes = Queue(maxsize=100)
        self.max_width, self.min_width, self.max_height, self.min_height = 0, 1000, 0, 1000
        self.spinCommand = False
        self.pointUp = False

    def findHands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        # print(results.multi_hand_landmarks)
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
        return img

    def findPosition(self, img, handNo=0, draw=True):
        xList = []
        yList = []
        bbox = []
        self.lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                xList.append(cx)
                yList.append(cy)
                self.lmList.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 5, (255, 0, 255), cv2.FILLED)
            xmin, xmax = min(xList), max(xList)
            ymin, ymax = min(yList), max(yList)
            bbox = xmin, ymin, xmax, ymax
            # print("bbox x width: " + str(xmax-xmin) + "  y length: " + str(ymax-ymin))
            if self.pointUp is True:  # only save rect sizes if finger pointing up
                self.modifyRecSizeQueue(xmax - xmin, ymax - ymin)
            else:  # clear queue if other gestures shown and clear max min values
                self.rectSizes = Queue(maxsize=100)
                self.max_width, self.min_width, self.max_height, self.min_height = 0, 1000, 0, 1000
                # print("Queue cleared")
                # print(list(self.rectSizes.queue))
            if draw:
                cv2.rectangle(img, (bbox[0] - 20, bbox[1] - 20),
                              (bbox[2] + 20, bbox[3] + 20), (0, 255, 0), 2)
        return self.lmList, bbox

    def fingersUp(self):
        self.fingers = []

        orientation = self.Orientation()
        self.fingers.append(orientation)
        if orientation == 0:  # pointing up
            # Thumb
            if self.lmList[0][1] < self.lmList[1][1]:  # thumb pointing right
                if self.lmList[self.tipIds[0]][1] > self.lmList[self.tipIds[0] - 1][1]:
                    self.fingers.append(2)
                else:
                    self.fingers.append(0)
            else:  # thumb pointing left
                if self.lmList[self.tipIds[0]][1] < self.lmList[self.tipIds[0] - 1][1]:
                    self.fingers.append(1)
                else:
                    self.fingers.append(0)
                # 4 Fingers
            for id in range(1, 5):
                if self.lmList[self.tipIds[id]][2] < self.lmList[self.tipIds[id] - 2][2]:
                    self.fingers.append(1)
                else:
                    self.fingers.append(0)
        elif orientation == 1:  # pointing left
            # Thumb
            if self.lmList[0][2] < self.lmList[1][2]:  # thumb pointing right
                if self.lmList[self.tipIds[0]][2] > self.lmList[self.tipIds[0] - 1][2]:
                    self.fingers.append(2)
                else:
                    self.fingers.append(0)
            else:  # thumb pointing left
                if self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[0] - 1][2]:
                    self.fingers.append(1)
                else:
                    self.fingers.append(0)
                # 4 Fingers
            for id in range(1, 5):
                if self.lmList[self.tipIds[id]][1] < self.lmList[self.tipIds[id] - 2][1]:
                    self.fingers.append(1)
                else:
                    self.fingers.append(0)
        elif orientation == 2:  # pointing down
            # Thumb
            if self.lmList[0][1] > self.lmList[1][1]:  # thumb pointing up
                if self.lmList[self.tipIds[0]][1] < self.lmList[self.tipIds[0] - 1][1]:
                    self.fingers.append(1)
                else:
                    self.fingers.append(0)
            else:  # thumb pointing down
                if self.lmList[self.tipIds[0]][1] > self.lmList[self.tipIds[0] - 1][1]:
                    self.fingers.append(2)
                else:
                    self.fingers.append(0)
            # 4 Fingers
            for id in range(1, 5):
                if self.lmList[self.tipIds[id]][2] > self.lmList[self.tipIds[id] - 2][2]:
                    self.fingers.append(1)
                else:
                    self.fingers.append(0)
        elif orientation == 3:  # pointing right
            # Thumb
            if self.lmList[0][2] < self.lmList[1][2]:  # thumb pointing down
                if self.lmList[self.tipIds[0]][2] > self.lmList[self.tipIds[0] - 1][2]:
                    self.fingers.append(2)
                else:
                    self.fingers.append(0)
            else:  # thumb pointing up
                if self.lmList[self.tipIds[0]][2] < self.lmList[self.tipIds[0] - 1][2]:
                    self.fingers.append(1)
                else:
                    self.fingers.append(0)
                # 4 Fingers
            for id in range(1, 5):
                if self.lmList[self.tipIds[id]][1] > self.lmList[self.tipIds[id] - 2][1]:
                    self.fingers.append(1)
                else:
                    self.fingers.append(0)

        return self.fingers

    # Method that finds the orientation of the hand
    def Orientation(self):
        # 0 = pointing up
        # 1 = pointing left
        # 2 = pointing down
        # 3 = pointing right

        yCoorList = [self.lmList[5][2], self.lmList[9][2], self.lmList[13][2], self.lmList[17][2]]
        ymax, ymin = max(yCoorList), min(yCoorList)

        if ymin - 20 < self.lmList[0][2] < ymax + 20:
            if self.lmList[0][1] < self.lmList[5][1]:
                # print("Pointing right")
                return 3
            else:
                # print("pointing left")
                return 1
        else:
            if self.lmList[0][2] > self.lmList[5][2]:
                # print("pointing up")
                return 0
            else:
                # print("pointing down")
                return 2

    def gestureFinder(self):
        # print(self.fingers[1:])
        f = self.fingers[1:]
        if f == [0, 0, 0, 0, 0]:
            return "land"

        if self.fingers[0] == 0:  # hand pointing up
            if f == [1, 1, 1, 1, 1] or f == [2, 1, 1, 1, 1]:
                # TODO make so that only "stop" command stops the spin tracking of the drone(s)
                return "stop"
            elif f == [1, 0, 0, 0, 1]:
                return "go forward"
            elif f == [2, 0, 0, 0, 1]:
                return "go backward"
            elif f == [0, 1, 0, 0, 0] or f == [1, 1, 0, 0, 0] or f == [2, 1, 0, 0, 0]:
                return "go up"
            elif f == [2, 0, 0, 0, 0]:
                return "spin left"
            elif f == [1, 0, 0, 0, 0]:
                return "spin right"
            elif f == [0, 0, 1, 0, 0]:
                return "offensive"
            elif f == [1, 0, 1, 0, 0]:
                return "offensive"
            elif f == [2, 0, 1, 0, 0]:
                return "offensive"
            elif f == [0, 1, 0, 0, 1]:
                return "rock"
        elif self.fingers[0] == 1:  # hand pointing left
            if f == [2, 0, 0, 0, 0]:
                return "thumbs down"
            elif f == [1, 0, 0, 0, 0]:
                return "thumbs up"
            elif f == [0, 1, 0, 0, 0] or f == [1, 1, 0, 0, 0] or f == [2, 1, 0, 0, 0]:
                return "go left"
        elif self.fingers[0] == 2:  # hand pointing down
            if f == [0, 1, 0, 0, 0] or f == [1, 1, 0, 0, 0] or f == [2, 1, 0, 0, 0]:
                return "go down"
        elif self.fingers[0] == 3:  # hand pointing right
            if f == [2, 0, 0, 0, 0]:
                return "thumbs down"
            elif f == [1, 0, 0, 0, 0]:
                return "thumbs up"
            elif f == [0, 1, 0, 0, 0] or f == [1, 1, 0, 0, 0] or f == [2, 1, 0, 0, 0]:
                return "go right"

    def findDistance(self, p1, p2, img, draw=True):
        x1, y1 = self.lmList[p1][1], self.lmList[p1][2]
        x2, y2 = self.lmList[p2][1], self.lmList[p2][2]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        if draw:
            cv2.circle(img, (x1, y1), 15, (255, 0, 255), cv2.FILLED)
            cv2.circle(img, (x2, y2), 15, (255, 0, 255), cv2.FILLED)
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
            cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
        length = math.hypot(x2 - x1, y2 - y1)
        return length, img, [x1, y1, x2, y2, cx, cy]

    def modifyRecSizeQueue(self, width, height):
        # if finger is pointing up
        # save rectangle sizes around hand in a queue
        # if differences are big enough then circular movement of hand is detected
        # (circular movement is done parallel to ground, i.e. toward camera=away from body; away from cam=toward body)
        # active spin command (Circle.py) then
        if width > self.max_width:
            self.max_width = width
        if width < self.min_width:
            self.min_width = width
        if height > self.max_height:
            self.max_height = height
        if height < self.min_height:
            self.min_height = height
        if self.rectSizes.full():
            self.rectSizes.get()
        self.rectSizes.put([width, height])
        diff_width = self.max_width - self.min_width
        diff_height = self.max_height - self.min_width
        # print("Diff width: " + str(diff_width))
        # print("Diff height: " + str(diff_height))
        # TODO need to define accurate tresholds for activation (do experiments)
        if diff_width > 75 and diff_height > 200:  # activate spin command and clear queue and max min values
            # print("Activate spin command")
            self.spinCommand = True
            self.rectSizes = Queue(maxsize=100)
            self.max_width, self.min_width, self.max_height, self.min_height = 0, 1000, 0, 1000
            # TODO call actual spin command (Circle.py I think or sth)
        # print("Queue: ")
        # print(list(self.rectSizes.queue))
        # print(self.rectSizes.qsize())


def send(drones, command):
    print(f"Sending {command} to drones")
    for drone in drones:
        drone.sendto(command.encode(), ('192.168.10.1', 8889))


def continue_flight(drones):
    send(drones, "rc 0 0 0 0")

    # Give drones some time to become stationary
    start_time = time.time()
    while time.time() - start_time < 1.0:
        pass

    for drone in drones:
        drone.controller.restore_position_before_interruption()

    # Continue the flight
    for drone in drones:
        drone.controller.flight_interrupted = False


def interrupt_flight(drones):
    for drone in drones:
        drone.controller.flight_interrupted = True

    # Wait until all controllers have finished sending new velocities
    start_time = time.time()
    while time.time() - start_time < 0.3:
        pass

    send(drones, "rc 0 0 0 0")

    # Give drones some time to become stationary
    start_time = time.time()
    while time.time() - start_time < 1.0:
        pass

    for drone in drones:
        drone.controller.save_current_position()


def execute_gesture(drones, gesture):
    # if not drones[0].controller.flight_interrupted:
    #     interrupt_flight(drones)

    velocity = 15
    if gesture == "go left":
        send(drones, f"rc -{velocity} 0 0 0")
    elif gesture == "go right":
        send(drones, f"rc {velocity} 0 0 0")
    elif gesture == "go backward":
        send(drones, f"rc 0 -{velocity} 0 0")
    elif gesture == "go forward":
        send(drones, f"rc 0 {velocity} 0 0")
    elif gesture == "go down":
        send(drones, f"rc 0 0 -{velocity} 0")
    elif gesture == "go up":
        send(drones, f"rc 0 0 {velocity} 0")
    elif gesture == "spin left":
        send(drones, f"rc 0 0 0 -{velocity}")
    elif gesture == "spin right":
        send(drones, f"rc 0 0 0 {velocity}")
    elif gesture == "land":
        send(drones, "land")
    else:
        send(drones, f"rc 0 0 0 0")


def detect_gesture(drones):
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    detector = handDetector()
    previous_gesture = None
    while True:
        success, img = cap.read()
        img = imutils.resize(img, height=480)
        img = detector.findHands(img)
        lmList, bbox = detector.findPosition(img, draw=True)
        if len(lmList) != 0:
            fingers = detector.fingersUp()
            # print(fingers)
            current_gesture = detector.gestureFinder()
            cv2.putText(img, f'{current_gesture}', (bbox[0], bbox[3] + 60), cv2.FONT_HERSHEY_COMPLEX,
                        1, (0, 0, 255), 3)
            if current_gesture != previous_gesture:
                execute_gesture(drones, current_gesture)
            previous_gesture = current_gesture
        else:
            send(drones, "rc 0 0 0 0")
        #     if drones[0].controller.flight_interrupted:
        #         continue_flight(drones)
        cv2.imshow("Image", img)
        cv2.waitKey(1)


drones = []

drone1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
drone1.setsockopt(socket.SOL_SOCKET, 25, 'wlxd03745f79670'.encode())
drone1.bind(('', 9000))
drones.append(drone1)

drone2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
drone2.setsockopt(socket.SOL_SOCKET, 25, 'wlxd0374572e205'.encode())
drone2.bind(('', 9000))
drones.append(drone2)

send(drones, "command")
send(drones, "takeoff")

detect_gesture(drones)
