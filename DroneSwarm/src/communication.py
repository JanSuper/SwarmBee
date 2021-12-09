# This file uses TerryKim's code as a base. Source: http://tellopilots.com/threads/tello-drone-swarm.288/post-7007

# Import the necessary modules
import socket
import threading

# IP and port of Tello
tello_address = ('192.168.10.1', 8889)

# IP and port of local computer
local_address = ('', 9000)

# Create a UDP connection that we'll send the command to
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, 25, 'wlxd0374572e205'.encode())

# Bind to the local address and port
sock.bind(local_address)


# Send the message to Tello and allow for a delay in seconds
def send(message):
    # Try to send the message otherwise print the exception
    try:
        sock.sendto(message.encode(), tello_address)
        print("Sending message: " + message)
    except Exception as e:
        print("Error sending: " + str(e))


# Receive the message from Tello
def receive():
    # Continuously loop and listen for incoming messages
    while True:
        # Try to receive the message otherwise print the exception
        try:
            response, ip_address1 = sock.recvfrom(128)
            decoded_response = response.decode(encoding='utf-8')
            print("Received message from drone #1: " + decoded_response)
        except Exception as e:
            # If there's an error close the socket and break out of the loop
            print("Error receiving: " + str(e))
            break


# Create and start a listening thread that runs in the background
# This utilizes our receive function and will continuously monitor for incoming messages
receiveThread = threading.Thread(target=receive)
receiveThread.daemon = True
receiveThread.start()

while True:
    command = input()
    send(command)
    if command == "quit":
        break

# Close the socket
sock.close()
