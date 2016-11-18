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

#Contains all data for motors, servo and sensors
motor_data = {"LEFT_SIDE_DIRECTION":0, "LEFT_SIDE_DIRECTION":0,"LEFT_SIDE_SPEED":0,"RIGHT_SIDE_DIRECTION":0,"RIGHT_SIDE_SPEED":0,"SERVO_ANGLE":0}
sensor_data = {"IR_LEFT_FRONT":0,"IR_LEFT_BACK":0,"IR_RIGHT_FRONT":0,"IR_RIGHT_BACK","IR_BACK":0,"IR_LIDAR":0,"GYRO":0}

#If someone finds a better way, please change this.
def get_sensor_dict_key(c_enum : CommandEnums):
    if(c_enum == CommandEnums.READ_IR_LEFT_FRONT):
        return "IR_LEFT_FRONT"
    elif(c_enum == CommandEnums.READ_IR_LEFT_BACK):
        return "IR_LEFT_BACK"
    elif(c_enum == CommandEnums.READ_IR_RIGHT_FRONT):
        return "IR_RIGHT_FRONT"
    elif(c_enum == CommandEnums.READ_IR_RIGHT_BACK):
        return "IR_RIGHT_BACK"
    elif(c_enum == CommandEnums.READ_IR_BACK):
        return "IR_BACK"
    elif(c_enum == CommandEnums.READ_IR_LIDAR):
        return "IR_LIDAR"
    elif(c_enum == CommandEnums.READ_GYRO):
        return "GYRO"
    else:
        return False
#Reads IR-sensor and sets corresponding sensor_data key.
def read_ir_sensor(command : Command):
    c_enum = command.get_enum()
    UART.send_command(command)
    ack = UART.receive_packet() #Receive ack (TODO: implement response check/resend if we implement timeout on receive)
    (ir_value) = UART.receive_payload()
    sensor_data[get_sensor_dict_key(c_enum)] = ir_value

#Send command to controller to retrieve and return motor data
def get_motor_diagnostics(command : Command):
    pwm_to_speed = 2.5 #Constant for getting motor pwm -> motor speed percentage
    UART.send_command(command)
    ack = UART.receive_packet() #Receive ack
    #Sorry for unreadable code!
    (left_direction, left_pwm, right_direction, right_pwm, servo_pwm_msb, servo_pwm_lsb) = uart.receive_payload()
    #TODO: Send ack
    servo_angle = ((servo_pwm_msb*(2**8)+servo_pwm_lsb)-708)/8.45 #Formula for translating servo pwm to servo angle
    print("LEFT: ", 
          "forward " if left_direction else "backward ",  left_pwm/pwm_to_speed, "%\n",
          "RIGHT: ",
          "forward " if right_direction else "backward ", right_pwm/pwm_to_speed, "%\n",
          "SERVO: ", servo_angle, " degrees\n")
    #TODO: We probably want to pass this information to something that can bring it to the PC
    return

def 

#We can handle pc -> rpi -> pc in shell methods
def handle_command(command : Command):
    c_enum = command.get_enum()
    if(c_enum == CommandEnums.CONTROLLER_INFORMATION):
        get_motor_diagnostics(command)
    elif(c_enum == CommandEnums.READ_IR_LEFT_FRONT or 
            c_enum == CommandEnums.READ_IR_LEFT_BACK or
            c_enum == CommandEnums.READ_IR_RIGHT_FRONT or
            c_enum == CommandEnums.READ_IR_RIGHT_BACK or
            c_enum == CommandEnums.READ_IR_BACK):
        read_ir_sensor(command)  
    elif(c_enum == CommandEnums.READ_IR_LIDAR):
    
    elif(c_enum == CommandEnums.READ_GYRO):
    
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
