# """custom path visualizer"""
# import math
#
# import cv2
#
# from time import sleep
#
#
# from mpl_toolkits import mplot3d
# import numpy as np
# import matplotlib.pyplot as plt
#
#
# # TODO: Do manual testing, update the values
#
#
# dInterval = fSpeed * interval
# aInterval = aSpeed * interval
# # Position       x,   y,  z, yaw (in degrees)
# imu = np.array([500, 500, 0, 0], dtype=np.int16)
#
# points = [(0, 0, 0), (0, 0, 0)]
#
# class Map:
#
#     def __init__(self):
#
#
#
#
#
#     # a += yaw
#     # x += int(d * math.cos(math.radians(a)))
#     # y += int(d * math.sin(math.radians(a)))
#     #
#     # return [lr, fb, ud, yv, x, y]
#
#     def calculate(self):
#         a += yaw
#         x += int(d * math.cos(math.radians(a)))
#         y += int(d * math.sin(math.radians(a)))
#
#     def draw_points(self, img, points):
#         for point in points:
#             cv2.circle(img, point, 5, (0, 0, 255), cv2.FILLED)  # BGR direction
#         cv2.circle(img, points[-1], 8, (0, 255, 0), cv2.FILLED)  # heading
#         cv2.putText(img, f'({(points[-1][0] - 500) / 100}, {(points[-1][1] - 500) / 100} )m',
#                     (points[-1][0] + 10, points[-1][1] + 30), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 1)    # create points
#
#     def display_map(self):
#         img = np.zeros((1000, 1000, 3, np.uint8))  # 2^8 = 256 0 to 255
#         if self.points[-1][0] != imu[0] or self.points[-1][1] != imu[1] or self.points[-1][0] != imu[2]:
#             self.points.append((imu[0], imu[1]), imu[2])
#         draw_points(img, self.points)  # function draw points
#         cv2.imshow("Output:", img)
#         cv2.waitKey(1)
#
#
#
#
