#!/usr/bin/env python3
import asyncio
from DroneSwarm.src.Utilities import utils
from threading import Thread
from bleak import BleakClient
from bleak.exc import BleakError
import time
# import bluetooth

characteristic_uuid = "B5AF1711-6486-4104-8DBE-84B66CF6E1AD"
var_threshold = 0.5
interval_threshold = 15


class BackgroundBluetoothSensorRead:
    """
       This class read bluetooth values from a PacketStream in background. Use
       ObjectName.current_package to get the current values.
       """

    def __init__(self, address):
        self.characteristic_uuid = characteristic_uuid
        self.address = address
        self.current_package = [0, 0, 0]
        self.loop = asyncio.get_event_loop()
        self.loop.set_debug(False)
        self.connection = Connection(self)
        self.accept = False
        self.acceptL = False
        self.acceptF = False
        self.acceptR = False
        self.avg_package = [0, 0, 0]
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

    def wait_until_works(self):
        zero_package = [0, 0, 0]
        while self.current_package == zero_package:
            pass


class Package:

    def __init__(self, sensor_1=-1, sensor_2=-1, sensor_3=-1) -> None:
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

    def __init__(self, sensor_read) -> None:
        self.characteristic_uuid = sensor_read.characteristic_uuid
        self.address = sensor_read.address
        self.client = None
        self.loop = sensor_read.loop
        self.current_package = None
        self.connected = False
        self.reconnecting = False
        self.sensor_read = sensor_read
        self.history = [[], [], []]

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

            self.sensor_read.current_package[0] = self.current_package.sensor_1
            self.sensor_read.current_package[1] = self.current_package.sensor_2
            self.sensor_read.current_package[2] = self.current_package.sensor_3
            self.history[0].append(self.sensor_read.current_package[0])
            self.history[1].append(self.sensor_read.current_package[1])
            self.history[2].append(self.sensor_read.current_package[2])
            norm = False
            if len(self.history[0]) > 10:
                for ls in self.history:
                    ls.pop(0)
                norm = True
            # print([self.current_package.sensor_1, self.current_package.sensor_2, self.current_package.sensor_3])
            if norm:
                self.sensor_read.accept = utils.normalize(self.history, var_threshold=var_threshold,
                                                          interval_threshold=interval_threshold, print_out=True)
                self.sensor_read.acceptL = utils.normalize(self.history[0], var_threshold=var_threshold,
                                                           interval_threshold=interval_threshold, print_out=True,
                                                           dir=True)
                self.sensor_read.acceptF = utils.normalize(self.history[1], var_threshold=var_threshold,
                                                           interval_threshold=interval_threshold, print_out=True,
                                                           dir=True)
                self.sensor_read.acceptR = utils.normalize(self.history[2], var_threshold=var_threshold,
                                                           interval_threshold=interval_threshold, print_out=True,
                                                           dir=True)

                if self.sensor_read.accept:
                    self.sensor_read.avg_package = utils.roll_average(self.history)
                    # print(self.sensorRead.avg_package)
            self.current_package = None

    def disconnect_handler(self, client):
        self.connected = False
        self.client = None

    async def manager(self):
        while True:
            if self.client is not None:
                try:
                    await self.connect()
                except BleakError:
                    pass
            else:
                self.client = BleakClient(self.address, disconnected_callback=self.disconnect_handler)

    async def connect(self):
        if self.connected:
            return False

        await self.client.connect(use_cached=False)
        self.connected = self.client.is_connected
        if self.connected:
            await self.client.start_notify(self.characteristic_uuid, self.notification_handler)
            while self.connected:
                await asyncio.sleep(1)

    async def cleanup(self):
        if self.client and self.connected:
            await self.client.stop_notify(self.characteristic_uuid)
            await self.client.disconnect()


if __name__ == "__main__":
    address_EDAD6F = "84:CC:A8:2F:E9:32"
    address_EDB02F = "84:CC:A8:2E:9C:B6"
    address_60FF08 = "9C:9C:1F:E1:B0:62"

    # nearby_devices = bluetooth.discover_devices(lookup_names=True)
    # print("Found {} devices.".format(len(nearby_devices)))
    #
    # for addr, name in nearby_devices:
    #     print("  {} - {}".format(addr, name))

    bluetooth_modules = []

    bluetooth_EDAD = BackgroundBluetoothSensorRead(address_EDAD6F)
    bluetooth_EDAD.start()
    bluetooth_EDAD.wait_until_works()
    bluetooth_modules.append(bluetooth_EDAD)

    # bluetooth_EDB0 = BackgroundBluetoothSensorRead(address_EDB02F)
    # bluetooth_EDB0.start()
    # bluetooth_EDB0.wait_until_works()
    # bluetooth_modules.append(bluetooth_EDB0)

    # bluetooth_60FF = BackgroundBluetoothSensorRead(address_60FF08)
    # bluetooth_60FF.start()
    # bluetooth_60FF.wait_until_works()
    # bluetooth_modules.append(bluetooth_60FF)

    previous_time = time.time()
    interval = 0.1
    while True:
        current_time = time.time()
        if current_time - previous_time > interval:
            for bluetooth_module in bluetooth_modules:
                print(f"{bluetooth_module.address}: {bluetooth_module.current_package}")
                previous_time = current_time
