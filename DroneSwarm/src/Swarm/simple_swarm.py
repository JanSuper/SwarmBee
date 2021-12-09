# This file uses DroneBlocks' UDP send-receiver example code as a base
# Source: https://github.com/dbaldwin/DroneBlocks-Tello-Python/blob/master/lesson3-udp-send-receive/UDPSendReceive.py

# Import the necessary modules
import socket
import threading

# IP and port of Tello
tello_address = ('192.168.10.1', 8889)

# IP and port of local computer
local_address = ('', 9000)

# Create a UDP connection that we'll send the command to
sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock1.setsockopt(socket.SOL_SOCKET, 25, 'wlxd0374572e205'.encode())
sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock2.setsockopt(socket.SOL_SOCKET, 25, 'wlxd03745f79670'.encode())

# Bind to the local address and port
sock1.bind(local_address)
sock2.bind(local_address)


# Send the message to Tello
def send(message):
    # Try to send the message otherwise print the exception
    try:
        sock1.sendto(message.encode(), tello_address)
        busy[0] = True
        sock2.sendto(message.encode(), tello_address)
        busy[1] = True
        print("Sending message: " + message)
    except Exception as e:
        print("Error sending: " + str(e))

    if message != "land":
        print("Waiting...")
        wait()


# Receive the message from Tello
def receive():
    # Continuously loop and listen for incoming messages
    while True:
        # Try to receive the message otherwise print the exception
        try:
            response, ip_address1 = sock1.recvfrom(128)
            decoded_response = response.decode(encoding='utf-8')
            print("Received message from drone #1: " + decoded_response)
            if decoded_response != "":
                busy[0] = False
                if "error" in decoded_response:
                    error[0] = True
            response, ip_address2 = sock2.recvfrom(128)
            decoded_response = response.decode(encoding='utf-8')
            print("Received message from drone #2: " + decoded_response)
            if decoded_response != "":
                busy[1] = False
                if "error" in decoded_response:
                    error[1] = True
        except Exception as e:
            # If there's an error close the socket and break out of the loop
            print("Error receiving: " + str(e))
            stop()
            break


def wait():
    # If at least one drone is busy, wait
    while busy[0] or busy[1]:
        pass
    print("Continue")
    # If at least one drone encountered an error, stop
    if error[0] or error[1]:
        print("A drone encountered an error")
        stop()


def takeoff():
    send("command")
    send("battery?")
    send("streamon")
    send("streamoff")
    send("takeoff")


def stop():
    send("land")
    sock1.close()
    sock2.close()
    exit()


# Create and start a listening thread that runs in the background
# This utilizes our receive function and will continuously monitor for incoming messages
receiveThread = threading.Thread(target=receive)
receiveThread.daemon = True
receiveThread.start()

error = [False, False]  # captures a drone's error status; error[0] = drone 1, error[1] = drone 2
busy = [False, False]  # captures a drone's busy status; busy[0] = drone 1, busy[1] = drone 2

takeoff()

# list of commands to be executed one by one; currently describes flying in a square
list_of_commands = ['forward 50', 'cw 90', 'forward 50', 'cw 90', 'forward 50', 'cw 90', 'forward 50', 'cw 90']
for command in list_of_commands:
    send(command)

stop()
