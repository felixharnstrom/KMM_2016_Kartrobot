from pid import Pid
from command import Command
from UART import UART
import time
import math

#Define directions
class Direction:
    LEFT = 0
    RIGHT = 1

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

def turn(direction : Direction, degrees : int):
    #TODO: define general uart variables
    uart_styrenhet = UART("tty.usbserial-FT94S3SE")

    #Set the current direction to zero
    current_dir = 0
    
    #Set time to zero to turn untill stopped
    turn_instr = Command.turn(direction, 30, 0)
    uart_styrenhet.send_command(turn_instr)

    while (abs(current_dir) < degrees):
        #Use a reimann sum to add up all the gyro rates
        #Area = TimeDiff * turnRate 
        clk = time.time()
        turn_rate = read_sensor(Command.read_gyro()) / 100
        current_dir += (time.time() - clk) * turn_rate
    
        print("Current dir: " + str(current_dir) + " -- Turn rate: " + str(turn_rate))
        print("")

    #Turning is completed, stop the motors
    turn_instr = Command.stop_motors()
    uart_styrenhet.send_command(turn_instr)
    uart_styrenhet.close()
    

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
    drive_instr = Command.side_speeds(1, round(left_speed), 1, round(right_speed))
    uart_styrenhet.send_command(drive_instr)

"""
uart_styrenhet = UART("tty.usbserial-FT94S3SE")
turn_instr = Command.turn(1, 30, 1500)
uart_styrenhet.send_command(turn_instr)
uart_styrenhet.close()
"""
controller = Pid() 
controller.setpoint = 90
controller.output_data = 0
controller.set_tunings(0.7,0,0.6)
controller.set_sample_time(160)
controller.set_output_limits(-25,25)
controller.set_mode(1)

last_val = mean_sensor(5, Command.read_right_front_ir())
#turn(Direction.LEFT, 90)

while 1:
    
    # Get sensor values
    ir_right_front = mean_sensor(5, Command.read_right_front_ir())
    ir_right_back = mean_sensor(5, Command.read_right_back_ir())
    lidar = read_sensor(Command.read_lidar())
    #print ("DIST: ", ir_right_front)
    
    if(lidar < 200):
        print ("OBSTACLE")
        uart_styrenhet = UART("tty.usbserial-FT94S3SE")
        stop = Command.stop_motors()
        uart_styrenhet.send_command(stop)
        time.sleep(1)
        turn(Direction.LEFT, 90)
        time.sleep(1)
        uart_styrenhet.close()
        last_val = mean_sensor(5, Command.read_right_back_ir())
        continue
    
    if (ir_right_front > (2 * last_val)):
        print ("EDGE")
        uart_styrenhet = UART("tty.usbserial-FT94S3SE")
        stop = Command.stop_motors()
        uart_styrenhet.send_command(stop)
        time.sleep(1)
        drive_instr = Command.drive(1, 50, 500)
        #uart_styrenhet.send_command(drive_instr)
        #time.sleep(1)
        turn(Direction.RIGHT, 90)
        time.sleep(1)
        uart_styrenhet.send_command(drive_instr)
        #time.sleep(1)
        last_val = mean_sensor(5, Command.read_right_back_ir())
        uart_styrenhet.close()
        continue
    last_val = ir_right_front
    
    # We need to get the distance from the center of the robot perpendicular to the wall
    dist = (ir_right_front + ir_right_back) / 2
    angle = math.atan2(ir_right_back - ir_right_front, 95)  # 95 = distance between sensors
    perpendicular_dist = dist * math.cos(angle)

    print ("DIST: ", dist)
    print ("PERPENDICULAR DIST: ", perpendicular_dist)
    controller.input_data = perpendicular_dist
    controller.compute()
    controller.output_data += 100
    print ("OUTPUT: ", controller.output_data)
    drive(controller.output_data / 100, 30)
    print ("------")

