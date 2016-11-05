from UART import UART
from modules import *

uart = UART("ttyUSB0")
print (uart.create_metapacket_hex(Adress.SENSORENHET,0,StyrenhetFunctions.DRIVE))
uart.send_packet(uart.create_metapacket(Adress.SENSORENHET,0,StyrenhetFunctions.DRIVE))
while 1:
	print (uart.receive_packet())
uart.close()