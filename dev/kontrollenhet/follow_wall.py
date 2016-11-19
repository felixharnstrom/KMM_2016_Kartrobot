from pid import Pid
from command import Command
from UART import UART
import time
import math
import atexit

# Return a sensor value
def read_sensor(sensor_instr : Command):
    #TODO: define general uart variables
    uart_sensorenhet = UART("tty.usbserial-FT94SB1D")
    uart_sensorenhet.send_command(sensor_instr)
    uart_sensorenhet.receive_packet()
    adress, length, typ = uart_sensorenhet.decode_metapacket(uart_sensorenhet.receive_packet())

    highest = uart_sensorenhet.receive_packet()
    lowest = uart_sensorenhet.receive_packet()

    uart_sensorenhet.close()
    return int.from_bytes(highest + lowest, byteorder="big", signed=True)

#Take a average value from a given sensor
def mean_sensor(it : int, sensor_instr : Command):
    distance = 0
    for i in range(it):
        distance += read_sensor(sensor_instr)
    return distance / it

def drive(ratio : int, base_speed : int):
    uart_styrenhet = UART("tty.usbserial-FT94S3SE")

    if (ratio < 0):
        print("LEFT")
        left_speed = base_speed * ((90 - (ratio * (-1))) / 100)
        right_speed = base_speed

    elif (ratio > 0):
        print("RIGHT")
        left_speed = base_speed
        right_speed = base_speed * ((90 - ratio) / 100)
        
    else:
        right_speed = base_speed
        left_speed = base_speed
    
    print("LEFT: ", left_speed)
    print("RIGHT: ", right_speed)
    drive_instr = Command.side_speeds(1, round(right_speed), 1, round(left_speed))
    uart_styrenhet.send_command(drive_instr)


controller = Pid() 
controller.setpoint = 100
controller.output_data = 0
controller.set_tunings(2,0.1,0.1)
controller.set_sample_time(100)
controller.set_output_limits(-90,90)
controller.set_mode(1)
uart_styrenhet = UART("tty.usbserial-FT94S3SE")

while 1:
    ir_left_front = mean_sensor(10, Command.read_left_back_ir())
    ir_left_back = mean_sensor(10, Command.read_left_front_ir())

    dist = (ir_left_front + ir_left_back) / 2
    error = dist - controller.setpoint
    print ("ERROR: ", error)
    controller.input_data = error
    controller.compute()
    print ("OUTPUT: ", controller.output_data)
    drive(controller.output_data, 25)