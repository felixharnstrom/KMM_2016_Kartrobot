import server
import json
from UART import UART

s = server.server()
s.start()
s.connect()

uart = UART("ttyUSB0")
while 1:
	data_string = json.dumps({"uartData": uart.receive_packet().decode("utf-8")}) #data serialized
	s.client.send(data_string.encode())

s.close()