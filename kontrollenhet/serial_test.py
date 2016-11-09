from UART import UART
from modules import *

uart = UART("ttyUSB1")
driveInstruction = Drive(1, 2)

uart.send_function(driveInstruction)

while 1:
	print (uart.receive_packet())

uart.close()
