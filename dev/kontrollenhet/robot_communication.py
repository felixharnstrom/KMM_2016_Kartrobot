import server
import json
from UART import UART
from modules import *
import numpy as np
import time

uart = UART("ttyUSB0")

s = server.server()
s.start()
s.connect()

while 1:
    data = s.client.recv(4096).decode("utf-8")
    
    if (data == "TEST"):
        s.client.sendall("ACK".encode())
        print ("SENT ACK")
        data = s.client.recv(4096).decode("utf-8")
        data_loaded = json.loads(data)
        instruction_type = data_loaded[0]
        argument_dict = data_loaded[1]
        direction = argument_dict['DIRECTION']
        speed = argument_dict['SPEED']
        time_16 = np.uint16(argument_dict['TIME'])
        mask_high = 0xFF00
        mask_low = 0x00FF
        time_high = np.uint8((time_16 & mask_high) >> 8)
        time_low = np.uint8((time_16 & mask_low))
        print ("WIFI")
        print (direction)
        print (speed)
        print (time_high)
        print (time_low)
        print ("UART")
        driveInstruction = Drive(direction, speed, time_high, time_low)
        print (driveInstruction.ARGUMENTS)
        ack_packet = (1,1,1)
        while (ack_packet[2] != 15):
            ack_packet = uart.decode_metapacket(uart.receive_packet())
        time.sleep(0.1)
        
    if (data == "KILL"):
        break
        
uart.close()
s.close()
