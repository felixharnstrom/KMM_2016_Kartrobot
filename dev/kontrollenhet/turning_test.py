import modules
import time
from sensorenhet_functions import *
from UART import UART

uart = UART("ttyUSB0")
instr = ReadGyro()

angle = 0
clk = time.time()

while True:
    uart.send_function(instr)
    uart.receive_packet()
    adress, length, typ = uart.decode_metapacket(uart.receive_packet())
    #print(adress, length, typ)
    highest = uart.receive_packet()
    lowest = uart.receive_packet()
    num = int.from_bytes(highest + lowest, byteorder="big", signed=True)
    timeBetween = time.time() - clk
    clk = time.time()
    angle += num * timeBetween / 100.0
    print(angle)
    #highest = int.from_bytes(uart.receive_packet(), byteorder='big')*2**8
    #lowest = int.from_bytes(uart.receive_packet(), byteorder='big')
    #print(highest+lowest)
    #print(65536-(highest+lowest))
    #for i in range(length):
    #print(uart.receive_packet())
    print("")

uart.close()

