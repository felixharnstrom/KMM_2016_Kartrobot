import modules
from sensorenhet_functions import *
from UART import UART

uart = UART("ttyUSB0")
instr = ReadLeftFrontIr()


while True:
        uart.send_function(instr)
        adress, length, typ = uart.decode_metapacket(uart.receive_packet())
        print(adress, length, typ)
        uart.receive_packet()
        uart.receive_packet()
        #for i in range(length):
                #print(uart.receive_packet())
        print("")

uart.close()

