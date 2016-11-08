import json
from client import client
robot = client()
robot.start()

while 1:
	
	data = robot.client.recv(1024).decode("utf-8") 
	data_loaded = json.loads(data) #data loaded

	print (data_loaded, type(data_loaded))