import serial
ser = serial.Serial('/dev/ttyUSB0', 9600)  # open serial port
while True:
	print (ser.read())
ser.close()             # close port