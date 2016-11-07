import json
import time
from client import client
robot = client()
robot.start(12345, "192.168.43.190")

commands = [["MOVE_MS", {"DIRECTION" : 1, "SPEED" : 10, "TIME" : 1000}], ["MOVE_MS", {"DIRECTION" : 1, "SPEED" : 50, "TIME" : 2000}], ["MOVE_MS", {"DIRECTION" : 1, "SPEED" : 100, "TIME" : 3000}]]



for command in commands:
	data_string = json.dumps(command) #data serialized
	robot.client.sendall("TEST".encode())
	data = robot.client.recv(4096).decode("utf-8")
	if (data == "ACK"):
		print ("GOT ACK")
		robot.client.sendall(data_string.encode())
	time.sleep(0.01)
	#data = robot.client.recv(1024).decode("utf-8") 
	#data_loaded = json.loads(data) #data loaded
	#print (data_loaded, type(data_loaded))

robot.client.sendall("KILL".encode())
