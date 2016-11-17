from UART import UART
from command import Command

uart = UART("ttyUSB1")
drive_instruction = Command.drive(1, 2, 1000)

uart.send_function(drive_instruction)

while 1:
    print (uart.receive_packet())

uart.close()
