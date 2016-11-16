import server
import json
#from UART import UART
from modules import *
from function import *
from communication import *
from UART import UART
import numpy as np
import time

uart = UART("ttyUSB0")

s = server.server()
s.start()
s.connect()

while 1:
    # The messages are made with json which appends extra "" - cut them off
    data = s.client.recv(4096).decode("utf-8")[1:-1].upper()
    
    if (data == "TRANSMIT"):
        # Acknowledge client
        s.client.sendall("ACK".encode())
        # Receive function
        function = receive_function(s.client)
        print (type(function))
        print (function.ADRESS, function.LENGTH, function.TYPE, function.ARGUMENTS)
        # Send over uart
        uart.send_function(function)
        # Wait for acknowledge
        ack = (1, 1, 1)
        while ack[2] != 0:
            ack = uart.decode_metapacket(uart.receive_packet())
        print("done")

    if (data == "GET_MOTOR_DIAG"):
        s.client.sendall("ACK".encode())
        func = ControllerInformation(0,0,0,0,0,0)
        # TODO: Should properly have a seperate function type for responses
        # For now, do this
        func.ARGUMENTS = []
        func.LENGTH = 0
        uart.send_function(func)
        ack = uart.receive_packet()
        ret = uart.receive_function()
        transmit_function(ret, s.client)
        
uart.close()
s.close()
