import json
import time
from communication import *
from command import *
from client import client
robot = client()
robot.start(12345, "localhost")

drive_instruction = Command.drive(1, 100, 2000)

while 1:
    robot.client.sendall("TRANSMIT".encode())
    data = robot.client.recv(4096).decode("utf-8")
    if (data == "ACK"):
        print ("GOT ACK")
        transmitCommand(drive_instruction, robot.client)
    time.sleep(1)
