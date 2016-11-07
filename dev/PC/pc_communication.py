import json
import time
from client import client
robot = client()
robot.start(12345, "192.168.43.190")

while 1:
	data_string = json.dumps(["MOVE_MS", {"DIRECTION" : 1, "SPEED" : 100, "TIME" : 2000}]) #data serialized
	robot.client.send(data_string.encode())
	time.sleep(5)
	#data = robot.client.recv(1024).decode("utf-8") 
	#data_loaded = json.loads(data) #data loaded
	#print (data_loaded, type(data_loaded))
