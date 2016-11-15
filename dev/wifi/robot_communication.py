import server
import json
#from UART import UART
from modules import *
from function import *
from communication import *
import numpy as np
import time

#uart = UART("ttyUSB0")

s = server.server()
s.start()
s.connect()

while 1:
    data = s.client.recv(4096).decode("utf-8")
    
    if (data == "TRANSMIT"):
        s.client.sendall("ACK".encode())
        print ("SENT ACK")
        function = receive_function(s.client)
        print (type(function))
        print (function.ADRESS, function.LENGTH, function.TYPE, function.ARGUMENTS)
        
#uart.close()
s.close()
