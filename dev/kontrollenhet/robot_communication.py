import server
import json
from UART import UART
from modules import *
import numpy as np
import time

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
	foo = np.uint16(argument_dict['TIME'])
	mask1 = 0xFF00
	mask2 = 0x00FF
	time1 = np.uint8((foo & mask1) >> 8)
	time2 = np.uint8((foo & mask2))
	driveInstruction = Drive(0, 100, 7, 208)
	uart.send_function(driveInstruction)
	print (data_loaded, type(data_loaded))
	print (instruction_type, type(instruction_type))
	print (direction, type(direction))
	print (speed, type(speed))
	print (time1, type(time1))
	print (time2, type(time2))
	#time.sleep(1)
	
uart.close()
s.close()
