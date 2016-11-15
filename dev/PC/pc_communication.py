import json
import time
from modules import*
from communication import*
from client import client
robot = client()
robot.start(12345, "localhost")

time_high, time_low = splitTime(2000)
driveInstruction = Drive(1, 100, time_high, time_low)
foo = ReadLeftBackIr()

while 1:
	robot.client.sendall("TRANSMIT".encode())
	data = robot.client.recv(4096).decode("utf-8")
	if (data == "ACK"):
		print ("GOT ACK")
		transmit_function(foo, robot.client)
	time.sleep(1)
