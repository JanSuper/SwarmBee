#!/usr/bin/env python3
"""Printing values are commented out to test only bluetooth uncomment them on line 123"""
import asyncio
# from ros_comm.clients.rospy.src import rospy as rospy
# from sensor_msgs.msg import LaserScan
from threading import Thread
from bleak import BleakClient
from bleak.exc import BleakError
import time

CHARACTERISTIC_UUID = "B5AF1711-6486-4104-8DBE-84B66CF6E1AD"

ADDRESS = '84:CC:A8:2F:E9:32'

current_package = [0, 0, 0]


class BackgroundBluetoothSensorRead:
    """
       This class read bluetooth values from a PacketStream in background. Use
       BackgroundBluetoothSensorRead.current_package to get the current values.
       """

    def __init__(self):
        self.uuid = CHARACTERISTIC_UUID
        self.address = ADDRESS
        self.current_package = current_package
        # self.value = [0,0,0]
        # rospy.init_node('tello_tof_publisher')
        self.loop = asyncio.get_event_loop()
        self.loop.set_debug(False)
        self.connection = Connection(self.address, self.uuid, self.loop)
        #
        # try:
        #     Tello.LOGGER.debug('trying to grab video frame...')
        #     self.container = av.open(self.address + '?timeout=1000000', timeout=Tello.FRAME_GRAB_TIMEOUT)
        # except av.error.ExitError:
        #     raise Exception('Failed to grab video frame from video stream')

        self.stopped = False
        self.worker = Thread(target=self.update_frame, args=(), daemon=True)

    def start(self):
        """Start the frame update worker
        Internal method, you normally wouldn't call this yourself.
        """
        self.worker.start()

    def update_frame(self):
        """Thread worker function to retrieve frames using PyAV
        Internal method, you normally wouldn't call this yourself.
        """
        if not self.stopped:
            try:
                self.loop.run_until_complete(self.connection.manager())
            except KeyboardInterrupt:
                print("exiting...")
            finally:
                self.loop.run_until_complete(self.connection.cleanup())
            time.sleep(0.1)

    def stop(self):
        self.stopped = True
        self.worker.join()


class Package:

    def __init__(self, sensor_1=0, sensor_2=0, sensor_3=0) -> None:
        self.sensor_1 = sensor_1  # left
        self.sensor_2 = sensor_2  # front
        self.sensor_3 = sensor_3  # right
        pass

    def parse_second_part(self, package):
        self.sensor_3 = ((package >> 20) & 0xFFF) / 1000.0

    @staticmethod
    def create_from_first_part(data):
        return Package(
            sensor_1=((data >> 12) & 0xFFF) / 1000.0,
            sensor_2=((data >> 0) & 0xFFF) / 1000.0
        )


class Connection:

    def __init__(self, address, characteristic, loop) -> None:
        self.address = address
        self.characteristic = characteristic
        self.client = None
        self.loop = loop

        self.current_package = None
        # self.scan_msg = LaserScan()
        # self.scan_msg.header.seq = -1
        # self.scan_msg.range_min = 0.04  # [m]
        # self.scan_msg.range_max = 2.00  # [m]
        # self.pub = rospy.Publisher('/tello/arduino/tof', LaserScan, queue_size=10)

        self.connected = False
        self.reconnecting = False

    def notification_handler(self, sender, data):
        package = int.from_bytes(data, byteorder='little', signed=False)
        first_package = (package >> 24) == 0xAD
        second_package = (package & 0xFF) == 0xDA

        if not first_package and not second_package:
            return

        elif self.current_package is None and first_package:
            self.current_package = Package.create_from_first_part(package)

        elif self.current_package is not None and second_package:
            self.current_package.parse_second_part(package)

            # self.scan_msg.header.seq += 1
            # self.scan_msg.header.stamp = rospy.Time.now()
            # print([self.current_package.sensor_1, self.current_package.sensor_2,
            #        self.current_package.sensor_3])
            current_package[0] = self.current_package.sensor_1
            current_package[1] = self.current_package.sensor_2
            current_package[2] = self.current_package.sensor_3
            # self.pub.publish(self.scan_msg)
            self.current_package = None

    def disconnect_handler(self, client):
        # rospy.logwarn("Client disconnected...")
        self.connected = False
        self.client = None

    async def manager(self):
        while True:
            if self.client is not None:
                try:
                    await self.connect()
                except BleakError:
                    pass
                    # rospy.logerr("Failed to establish connection...")
            else:
                # rospy.loginfo("Create BleakClient")
                self.client = BleakClient(self.address, disconnected_callback=self.disconnect_handler)

    async def connect(self):
        if self.connected:
            return False

        # rospy.loginfo("Connecting...")
        await self.client.connect(use_cached=False)
        self.connected = self.client.is_connected
        if self.connected:
            # rospy.loginfo("Connected...")
            await self.client.start_notify(self.characteristic, self.notification_handler)
            while self.connected:
                await asyncio.sleep(1)
        # else:
        # rospy.logerr("connect() called but connection failed")

    async def cleanup(self):
        if self.client and self.connected:
            await self.client.stop_notify(self.characteristic)
            await self.client.disconnect()


if __name__ == "__main__":
    # rospy.init_node('tello_tof_publisher')
    loop = asyncio.get_event_loop()
    loop.set_debug(True)
    connection = Connection(ADDRESS, CHARACTERISTIC_UUID, loop)
    try:
        loop.run_until_complete(connection.manager())
    except KeyboardInterrupt:
        print("exiting...")
    finally:
        loop.run_until_complete(connection.cleanup())
