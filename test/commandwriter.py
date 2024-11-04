"""Outputs commands to zmq to test the cameramanager"""

import zmq
import time

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:7777") # ensure same address as in cpp

# Send a message using the socket and cli input
# command = input("Enter command as exact string: ")
command1 = "0qu02id01"
command3 = "0qu02id03"
command_end1 = "201"
commandstr = "1qu02id01" # stream camera 1 over ffmpeg
count = 0
while False:
    socket.send_string(command1)
    print("Sent: ", command1)
    time.sleep(1)
    socket.send_string(command3)
    print("Sent: ", command3)
    time.sleep(1)
    if count == 4:
        socket.send_string(command_end1)
        print("Sent: ", command_end1)
        break
    count += 1


for _ in range(3):
    socket.send_string(commandstr)
    print("Sent: ", commandstr)
    time.sleep(1)
    