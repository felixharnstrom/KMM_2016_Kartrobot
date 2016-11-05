from UART import UART
uart = UART("ttyUSB0")
print (uart.create_metapacket_hex(0,3,2))
uart.send_packet(uart.create_metapacket(1,0,1))
while 1:
	print (uart.receive_packet())
uart.close()