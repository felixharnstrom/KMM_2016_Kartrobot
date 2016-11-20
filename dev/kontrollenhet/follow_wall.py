from pid import Pid
from command import Command
from UART import UART
import time
import math
import timeit

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
    print ("RATIO: ", ratio)
    left_speed = base_speed / ratio
    right_speed = base_speed * ratio
    
    print("LEFT: ", round(left_speed))
    print("RIGHT: ", round(right_speed))
    drive_instr = Command.side_speeds(1, round(right_speed), 1, round(left_speed))
    uart_styrenhet.send_command(drive_instr)


controller = Pid() 
controller.setpoint = 80
controller.output_data = 0
controller.set_tunings(1.5,5,10)
controller.set_sample_time(160)
controller.set_output_limits(75,125)
controller.set_mode(1)

while 1:
    # Get sensor values
    ir_right_back = mean_sensor(5, Command.read_right_back_ir())
    ir_right_front = mean_sensor(5, Command.read_right_front_ir())

    # We need to get the distance from the center of the robot perpendicular to the wall
    dist = (ir_right_front + ir_right_back) / 2
    angle = math.atan2(ir_right_back - ir_right_front, 95)  # 95 = distance between sensors
    perpendicular_dist = dist * math.cos(angle)

    print ("DIST: ", dist)
    print ("PERPENDICULAR DIST: ", perpendicular_dist)
    controller.input_data = perpendicular_dist
    controller.compute()
    print ("OUTPUT: ", controller.output_data)
    drive(controller.output_data / 100, 50)
    print ("------")