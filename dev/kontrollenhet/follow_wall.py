from command import Command
import time
import math
from UART import UART
from pid import Pid
import numpy as np

#Define your USB ports
uart_sensorenhet = UART("ttyUSB0")
uart_styrenhet = UART("ttyUSB1")
BLOCK_SIZE = 400 #millimeters

#TODO: Use Daniels enum for MANUAL, AUTONOM
class ControllerMode:
    MANUAL = 0
    AUTONOM = 1

#Define directions
class Direction:
    LEFT = 0
    RIGHT = 1

class DriveStatus:
    OBSTACLE_DETECTED = 0
    RIGHT_CORRIDOR_DETECTED = 1
    DONE = 2

class Robot:
    def __init__(self, mode : ControllerMode):
        self.driven_distance = 0
        self.current_angle = 0
        self.help_angle = 0
        self.control_mode = mode
        self.path_trace = [] # (Angle, LengthDriven), with this list we can calculate our position
        self.path_queue = [] # (Blocks_To_Drive, Direction)
        self.last_dist = 0

        #Initiatee PID controller
        self.pid_controller = Pid()
        self.pid_controller.setpoint = 0
        self.pid_controller.output_data = 0
        self.pid_controller.set_tunings(0.7,0,0.3)
        self.pid_controller.set_sample_time(33)
        self.pid_controller.set_output_limits(-50,50)
        self.pid_controller.set_mode(1)

    # Return a sensor value
    def read_sensor(self, sensor_instr : Command):
        uart_sensorenhet.send_command(sensor_instr)
        uart_sensorenhet.receive_packet()
        adress, length, typ = uart_sensorenhet.decode_metapacket(uart_sensorenhet.receive_packet())

        highest = uart_sensorenhet.receive_packet()
        lowest = uart_sensorenhet.receive_packet()
        return int.from_bytes(highest + lowest, byteorder="big", signed=True)

    #Take a average value from a given sensor
    def median_sensor(self, it : int, sensor_instr : Command):
        distance = []
        for i in range(it):
            distance.append(self.read_sensor(sensor_instr))
        if len(distance) == 1:
            return distance[0]
        else:
            return np.median(distance)

    def turn(self, direction : Direction, degrees : int):
        # TODO: Consider adjusting to wall if we have tomething on the side
        #Set the current direction to zero
        current_dir = 0

        #Set time to zero to turn untill stopped
        standstill_rate = self.median_sensor(32, Command.read_gyro()) / 130
        turn_instr = Command.turn(direction, 40, 0)
        uart_styrenhet.send_command(turn_instr)

        while (abs(current_dir) < degrees):
            #Use a reimann sum to add up all the gyro rates
            #Area = TimeDiff * turnRate
            clk = time.time()
            turn_rate = self.read_sensor(Command.read_gyro()) / 130 - standstill_rate
            current_dir += (time.time() - clk) * turn_rate

            #print("Current dir: " + str(current_dir) + " -- Turn rate: " + str(turn_rate))

        #Turning is completed, stop the motors
        turn_instr = Command.stop_motors()
        uart_styrenhet.send_command(turn_instr)

        #Add turned degrees to current_angle
        self.current_angle += degrees
        return

    #Save the collected data to the trace list
    def save_position(self):
        self.path_trace += [(self.current_angle, self.driven_distance)]

    def follow_wall_help(self, ratio : int, base_speed : int):
            left_speed = max(min(base_speed / ratio, 100), 0)
            right_speed = max(min(base_speed * ratio, 100), 0)
            drive_instr = Command.side_speeds(1, round(left_speed), 1, round(right_speed))
            uart_styrenhet.send_command(drive_instr)
            return

    def drive_distance(self, dist : int, speed : int):
        uart_styrenhet.send_command(Command.drive(1,speed,0))
        lidar_init = self.read_sensor(Command.read_lidar())
        lidar_cur = lidar_init
        print ("INIT: ", lidar_init)  
        while(lidar_init - lidar_cur < dist):
            lidar_cur = self.read_sensor(Command.read_lidar())
        print ("CURRENT: ", lidar_cur)    

    def wait_for_empty_corridor(self):
        uart_styrenhet.send_command(Command.drive(1,30,0))
        lidar = self.read_sensor(Command.read_lidar())
        while(lidar > 150):
            lidar = self.read_sensor(Command.read_lidar())
            continue
        uart_styrenhet.send_command(Command.stop_motors())

    #Follow the walls until the robot gets a unexpected stop command or
    #the given distance to drive is reached.
    def follow_wall(self, distance : int):
        MEDIAN_ITERATIONS = 3
        self.help_angle = 0

        #Set time to zero to turn untill stopped
        standstill_rate = self.median_sensor(32, Command.read_gyro()) / 130

        #Read start values from sensors
        lidar = self.read_sensor(Command.read_lidar())
        start_lidar = self.read_sensor(Command.read_lidar())

        self.last_dist = self.median_sensor(MEDIAN_ITERATIONS, Command.read_right_front_ir())

        #Drive untill the wanted distance is reached
        while (start_lidar - lidar <= distance):
            # Keep track of current angle
            #Use a reimann sum to add up all the gyro rates
            #Area = TimeDiff * turnRate
            clk = time.time()
            turn_rate = self.read_sensor(Command.read_gyro()) / 130 - standstill_rate
            self.help_angle += (time.time() - clk) * turn_rate

            # Get sensor values
            ir_right_front = self.median_sensor(MEDIAN_ITERATIONS, Command.read_right_front_ir())
            ir_right_back = self.median_sensor(MEDIAN_ITERATIONS, Command.read_right_back_ir())
            ir_left_front = self.median_sensor(MEDIAN_ITERATIONS, Command.read_left_front_ir())
            ir_left_back = self.median_sensor(MEDIAN_ITERATIONS, Command.read_left_back_ir())
            lidar = self.median_sensor(MEDIAN_ITERATIONS, Command.read_lidar())
            print ("IR_RIGHT_BACK: " + str(ir_right_back) + " IR_RIGHT_FRONT: " + str(ir_right_front) + " IR_LEFT_BACK: " + str(ir_left_back) + " IR_LEFT_FRONT: " + str(ir_left_front) + " LIDAR: " + str(lidar) + " GYRO: " + str(self.help_angle))

            #Detect corridor to the right
            if ((ir_right_front == 300) or (ir_right_front > 100 and ir_right_front > 2 * self.last_dist)):
                #Save the given length driven
                #self.wait_for_empty_corridor()
                self.drive_distance(100, 20)
                self.driven_distance += start_lidar - lidar
                self.save_position()
                self.last_dist = self.median_sensor(MEDIAN_ITERATIONS, Command.read_right_front_ir())
                return DriveStatus.RIGHT_CORRIDOR_DETECTED

            # Obstacle detected, and no turn to the right
            elif(lidar < 200):
                #Save the given length driven
                uart_styrenhet.send_command(Command.stop_motors())
                self.driven_distance += start_lidar - lidar
                self.save_position()
                return DriveStatus.OBSTACLE_DETECTED

            # We need to get the distance from the center of the robot perpendicular to the wall
            dist_right = (ir_right_front + ir_right_back) / 2
            angle_right = math.atan2(ir_right_back - ir_right_front, 95)  # 95 = distance between sensors
            perpendicular_dist_right = dist_right * math.cos(angle_right)
            dist_left = (ir_left_front + ir_left_back) / 2
            angle_left = math.atan2(ir_left_back - ir_left_front, 95)  # 95 = distance between sensors
            perpendicular_dist_left = dist_left * math.cos(angle_left)
            self.pid_controller.input_data = perpendicular_dist_right - perpendicular_dist_left
            self.pid_controller.compute()
            self.pid_controller.output_data += 100
            self.follow_wall_help(self.pid_controller.output_data / 100, 30)
            self.last_dist = self.median_sensor(MEDIAN_ITERATIONS, Command.read_right_front_ir())

        #Save the given length driven
        uart_styrenhet.send_command(Command.stop_motors())
        self.driven_distance += distance
        self.save_position()
        return DriveStatus.DONE

    def drive_until_wall_on_right(self):
        uart_styrenhet.send_command(Command.drive(1,30,0))
        ir_right_back = self.read_sensor(Command.read_right_back_ir())
        ir_right_front = self.read_sensor(Command.read_right_front_ir())
        while((ir_right_back > 200) and (ir_right_front > 200)):
            ir_right_back = self.read_sensor(Command.read_right_back_ir())
            ir_right_front = self.read_sensor(Command.read_right_front_ir())
        uart_styrenhet.send_command(Command.stop_motors())
        self.last_dist = self.median_sensor(3, Command.read_right_front_ir())

    
    #TODO: Add the correct code for updating the path_queue when the code is ready
    #Updates path_queue
    def find_next_destination(self):
        return [] #Return random path to drive

    #Scan the room at this position
    def scan(self):
        recorded_data = [] #(Angle, distance)
        #Place LIDAR to 0 degrees
        servo_instr = Command.servo(0)
        uart_styrenhet.send_command(servo_instr)

        #Sleep 1 second to wait for LIDAR to be in position
        time.sleep(1)

        for i in range(180):
            #Point LIDAR in the right angle
            servo_instr = Command.servo(i)
            uart_styrenhet.send_command(servo_instr)

            #Read data from the LIDAR sensor
            lidar_distance = self.read_sensor(Command.read_lidar())
            recorded_data += [(i,lidar_distance)]

        return recorded_data

    #Drive independent through the maze
    def autonom_mode(self):
        #Update the path_queue for positions to drive
        find_next_destination()
        return

def sensor_test(robot):
    while 1:
        MEDIAN_ITERATIONS = 3
        ir_right_front = robot.median_sensor(MEDIAN_ITERATIONS, Command.read_right_front_ir())
        ir_right_back = robot.median_sensor(MEDIAN_ITERATIONS, Command.read_right_back_ir())
        ir_left_front = robot.median_sensor(MEDIAN_ITERATIONS, Command.read_left_front_ir())
        ir_left_back = robot.median_sensor(MEDIAN_ITERATIONS, Command.read_left_back_ir())
        ir_back = robot.median_sensor(MEDIAN_ITERATIONS, Command.read_back_ir())
        lidar = robot.median_sensor(1, Command.read_lidar())
        print ("IR_RIGHT_BACK: " + str(ir_right_back) + " IR_RIGHT_FRONT : " + str(ir_right_front) + " LIDAR: " + str(lidar), "IR_LEFT_BACK", ir_left_back, "IR_LEFT_FRONT", ir_left_front, "IR_BACK", ir_back)
    

robot = Robot(ControllerMode.MANUAL)
#robot.turn(Direction.LEFT, 90)
#sensor_test(robot)

#Place LIDAR to 90 degrees
servo_instr = Command.servo(90)
uart_styrenhet.send_command(servo_instr)

#Wait fot LIDAR to be in position
time.sleep(1)

while 0:
    ir_right_front = robot.median_sensor(3, Command.read_right_front_ir())
    ir_right_back = robot.median_sensor(3, Command.read_right_back_ir())
    ir_left_front = robot.median_sensor(3, Command.read_left_front_ir())
    ir_left_back = robot.median_sensor(3, Command.read_left_back_ir())
    dist_left = (ir_left_front + ir_left_back) / 2
    angle_left = math.atan2(ir_left_back - ir_left_front, 95)
    print(math.degrees(angle_left))
    
#Try driving in infinite loop around the maze
while 1:
        #Drive forward without crashing in wall
        status = robot.follow_wall(9999999)
        if (status == DriveStatus.OBSTACLE_DETECTED):
                print ("---------- OBSTACLE DETECTED!")
                print ("---------- TURNING LEFT 90 degrees")
                robot.turn(Direction.LEFT, 90)
                robot.pid_controller.kp = 0
                robot.pid_controller.ki = 0
                robot.pid_controller.kd = 0
                robot.pid_controller.set_tunings(0.7,0,0.3)
        elif (status == DriveStatus.RIGHT_CORRIDOR_DETECTED):
                print ("---------- DETECTED CORRIDOR TO RIGHT!")
                print ("---------- TURNING RIGHT 90 degrees")
                print (robot.help_angle)
                robot.turn(Direction.RIGHT, 90 + robot.help_angle)
                robot.drive_distance(200, 20)
                robot.pid_controller.kp = 0
                robot.pid_controller.ki = 0
                robot.pid_controller.kd = 0
                robot.pid_controller.set_tunings(0.7,0,0.3)
        print("==========")






