import server
import json
from UART import UART

s = server.server()
s.start()
s.connect()

#uart = UART("ttyUSB0")
while 1:
	data = s.client.recv(1024).decode("utf-8") 
	data_loaded = json.loads(data) #data loaded
	#data_string = json.dumps({"uartData": uart.receive_packet().decode("utf-8")}) #data serialized
	#s.client.send(data_string.encode())
	print (data_loaded, type(data_loaded))

s.close()
