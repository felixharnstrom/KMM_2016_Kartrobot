import server
import json
from command import *
from communication import *
from UART import UART
import numpy as np
import time

#uart = UART("ttyUSB0")

s = server.server()
s.start()
s.connect()

#Send command to controller to retrieve and return motor data
def get_motor_diagnostics(command : Command):
    pwm_to_speed = 2.5 #Constant for getting motor pwm -> motor speed percentage
    UART.send_command(command)
    ack = UART.receive_packet() #Receive ack
    #Sorry for unreadable code!
    (left_direction, left_pwm, right_direction, right_pwm, servo_pwm_msb, servo_pwm_lsb) = UART.receive_packets()
    #TODO: Send ack
    servo_angle = ((servo_pwm_msb*(2**8)+servo_pwm_lsb)-708)/8.45 #Formula for translating servo pwm to servo angle
    print("LEFT: ", 
          "forward " if left_direction else "backward ",  left_pwm/pwm_to_speed, "%\n",
          "RIGHT: ",
          "forward " if right_direction else "backward ", right_pwm/pwm_to_speed, "%\n",
          "SERVO: ", servo_angle, " degrees\n")
    #TODO: We probably want to pass this information to something that can bring it to the PC

#We can handle pc -> rpi -> pc in shell methods
def handle_command(command : Command):
    c_enum = command.get_enum()
    if(c_enum == CommandEnums.CONTROLLER_INFORMATION):
        get_motor_diagnostics(command)
    else:
        #Temporary solution for all non-special commands (eg. just send)
        #Does not handle incorrectly parsed commands
        UART.send_command(command)
        ack = UART.receive_packet() #Receive ack
    return


while 1:
    # The messages are made with json which appends extra "" - cut them off
    data = s.client.recv(4096).decode("utf-8")
    print (data)
    
    if (data == "TRANSMIT"):
        # Acknowledge client
        s.client.sendall("ACK".encode())
        # Receive function
        command = receive_command(s.client)
        print (type(command))
        print (command.address, len(command.arguments), command.command_type, command.arguments)
        # Send over uart
        #uart.send_function(function)
        # Wait for acknowledge
        #ack = (1, 1, 1)
        #while ack[2] != 0:
        #    ack = uart.decode_metapacket(uart.receive_packet())
        #print("done")
    """
    if (data == "GET_MOTOR_DIAG"):
        s.client.sendall("ACK".encode())
        func = Command.controllerInformation()
        # TODO: Should properly have a seperate function type for responses
        # For now, do this
        func.ARGUMENTS = []
        func.LENGTH = 0
        uart.send_function(func)
        ack = uart.receive_packet()
        ret = uart.receive_function()
        transmit_function(ret, s.client)"""
        
#uart.close()
s.close()
