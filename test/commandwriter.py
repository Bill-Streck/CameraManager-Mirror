"""Outputs commands to zmq to test the cameramanager"""

import zmq
import time
import subprocess

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:7777") # ensure same address as in cpp

# Send a message using the socket and cli input
# command = input("Enter command as exact string: ")
command1 = "0qu02id01" # start camera 1 local
command3 = "0qu02id03" # start camera 3 local
command_end1 = "2id01" # end camera 1 local - should close thread
commandstr = "1qu02id03" # stream camera 3 over ffmpeg
command_end3 = "2id03" # end camera 3 local - thread should stay open and camera 3 should still stream, but not be seen locally
command_endstr = "3id03" # end camera 3 ffmpeg stream - should close thread

process_str = "ffplay -fflags nobuffer -flags low_delay -strict experimental -analyzeduration 0 -probesize 32 -sync ext -framedrop -f mpegts udp://localhost:9999"
proc = subprocess.Popen(process_str.split())

socket.send_string(command1)
time.sleep(1)
socket.send_string(command1) # thread should NOT open
time.sleep(1)
socket.send_string(command3)
time.sleep(1)
socket.send_string(command3) # thread should NOT open
time.sleep(1)
socket.send_string(command_end1) # should see a thread close
time.sleep(5)
socket.send_string(commandstr) # should find camera 3 streaming (will now clutter console)
time.sleep(5)
socket.send_string(command_end3) # thread should stay open
time.sleep(60)
socket.send_string(command_endstr) # should see stream close and thread end
print("Done")

proc.kill()
    