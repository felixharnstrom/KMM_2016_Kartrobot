import json
import time
from client import client
robot = client()
robot.start()

while 1:
	data_string = json.dumps(["MOVE_SQUARES", {"DIRECTION" : 1, "SQUARES" : 2}]) #data serialized
	robot.client.send(data_string.encode())
	time.sleep(2)
	#data = robot.client.recv(1024).decode("utf-8") 
	#data_loaded = json.loads(data) #data loaded
	#print (data_loaded, type(data_loaded))
