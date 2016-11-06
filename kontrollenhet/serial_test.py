from UART import UART
from modules import *

uart = UART("ttyUSB0")
scaninstruction = Scan(10,0,100)

uart.send_function(scaninstruction)

while 1:
	print (uart.receive_packet())

uart.close()