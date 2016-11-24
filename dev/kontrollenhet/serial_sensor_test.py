from command import Command
from sensorenhet_functions import *
from UART import UART

uart = UART("ttyUSB0")
instr = Command.read_left_front_ir()


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

