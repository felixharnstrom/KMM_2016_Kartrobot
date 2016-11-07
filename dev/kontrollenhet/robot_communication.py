import server
import json
from UART import UART
from modules import *
import numpy as np

s = server.server()
s.start()
s.connect()

uart = UART("ttyUSB0")
while 1:
	data = s.client.recv(1024).decode("utf-8") 
	data_loaded = json.loads(data) #data loaded
	#data_string = json.dumps({"uartData": uart.receive_packet().decode("utf-8")}) #data serialized
	#s.client.send(data_string.encode())
	instruction_type = data_loaded[0]
	argument_dict = data_loaded[1]
	direction = argument_dict['DIRECTION']
	speed = argument_dict['SPEED']
	time = np.uint16(argument_dict['TIME'])
	mask1 = 0xFF00
	mask2 = 0x00FF
	time1 = (time & mask1) >> 8
	time2 = (time & mask2)
	driveInstruction = Drive(direction, speed, time1, time2)
	uart.send_function(driveInstruction)
	
	print (data_loaded, type(data_loaded))
	print (instruction_type)
	print (direction)
	print (speed)
	print (time1)
	print (time2)
	
uart.close()
s.close()
