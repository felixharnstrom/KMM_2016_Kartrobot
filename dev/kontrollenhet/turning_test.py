from command import Command
import time
import math
from UART import UART
from pid import Pid

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
        LEFT_CORRIDOR_DETECTED = 1
        RIGHT_CORRIDOR_DETECTED = 2
        DONE = 3

class Robot:
        def __init__(self, mode : ControllerMode):
                self.driven_distance = 0
                self.current_angle = 0
                self.control_mode = mode
                self.path_trace = [] # (Angle, LengthDriven), with this list we can calculate our position
                self.path_queue = [] # (Blocks_To_Drive, Direction)

                #Initiatee PID controller
                self.pid_controller = Pid() 
                self.pid_controller.setpoint = 90
                self.pid_controller.output_data = 0
                self.pid_controller.set_tunings(0.7,0,0.6)
                self.pid_controller.set_sample_time(160)
                self.pid_controller.set_output_limits(-25,25)
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
def mean_sensor(self, it : int, sensor_instr : Command):
        distance = 0
        for i in range(it):
                distance += read_sensor(sensor_instr)
        return distance / it

def turn(self, direction : Direction, degrees : int):
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

        #Add turned degrees to current_angle
        self.current_angle += degrees
        return

#Calculate in what direction the robot is standing

#Calculate total distance driven
def calculate_distance_driven(self):
        return sum([t[1] for t in self.path_trace])

#Save the collected data to the trace list
def save_position(self):
        self.path_trace += [(self.current_angle, self.driven_distance)]
        self.current_angle = 0
        self.driven_distance = 0
        return

#TODO: TEST THIS CODE
#Save the collected data to the trace list
def reached_dead_end(self):
        #Read ir sensors
        ir_left_front = read_sensor(Command.read_left_front_ir())
        ir_left_back = read_sensor(Command.read_left_back_ir())
        ir_right_front = read_sensor(Command.read_right_front_ir())
        ir_right_back = read_sensor(Command.read_right_back_ir())

        #Point LIDAR in a 90 degreee angle
        servo_instr = Command.servo(90)
        uart_styrenhet.send_command(servo_instr)

        #Wait for LIDAR to adjust itself
        time.sleep(1)

        #Read LIDAR sensor
        lidar_sensor = read_sensor(Command.read_lidar())

        if (ir_left_front > 300 and ir_left_back > 300):
                return True
        elif (ir_right_front < 300 and ir_right_back < 300):
                return True
        elif (lidar_sensor < 200):
                return True

        #Returns if not a dead end
        return False

#TODO: TEST THIS CODE
#Follow the walls until the robot gets a unexpected stop command or
#the given distance to drive is reached.
def follow_wall(self, distance : int):
        #Read start values from sensors
        start_lidar = read_sensor(Command.read_lidar())
        right_last_val = mean_sensor(5, Command.read_right_front_ir())
        left_last_val = mean_sensor(5, Command.read_left_front_ir())

        #Drive untill the wanted distance is reached
        while (start_lidar - lidar >= distance):
                # Get sensor values
                ir_right_front = mean_sensor(5, Command.read_right_front_ir())
                ir_right_back = mean_sensor(5, Command.read_right_back_ir())
                ir_left_front = mean_sensor(5, Command.read_left_front_ir())
                lidar = read_sensor(Command.read_lidar())

                #Obstacle detected
                if(lidar < 200):
                        #Save the given length driven
                        self.driven_distance += start_lidar - lidar
                        save_position()
                        return DriveStatus.OBSTACLE_DETECTED

                #Detect corridor to the right
                if (ir_right_front > (2 * right_last_val)):
                        #Save the given length driven
                        self.driven_distance += start_lidar - lidar
                        save_position()
                        return DriveStatus.RIGHT_CORRIDOR_DETECTED
                elif (ir_left_front > (2 * left_last_val)):
                        #Save the given length driven
                        self.driven_distance += start_lidar - lidar
                        save_position()
                        return DriveStatus.LEFT_CORRIDOR_DETECTED

                #Save old IR values
                right_last_val = ir_right_front
                left_last_val = ir_left_front

                # We need to get the distance from the center of the robot perpendicular to the wall
                dist = (ir_right_front + ir_right_back) / 2
                angle = math.atan2(ir_right_back - ir_right_front, 95)  # 95 = distance between sensors
                perpendicular_dist = dist * math.cos(angle)

                controller.input_data = perpendicular_dist
                controller.compute()
                controller.output_data += 100
                drive(controller.output_data / 100, 30)

        #Save the given length driven
        self.driven_distance += distance
        save_position()
        return DriveStatus.DONE

#TODO: Add the correct code for updating the path_queue when the code is ready
#Updates path_queue
def find_next_destination(self):
        return

#TODO: TEST THIS CODE
#Scan the room at this position
def scan(self):
        recorded_data = [] #(Angle, distance)
        for i in range(180):
                #Point LIDAR in the right angle
                servo_instr = Command.servo(i)
                uart_styrenhet.send_command(servo_instr)

                #Read data from the LIDAR sensor
                lidar_distance = read_sensor(Command.read_lidar())
                recorded_data += [(i,lidar_distance)]
        return recorded_data
        
#Drive independent through the maze
def autonom_mode(self):
        #Update the path_queue for positions to drive
        find_next_destination()
        return

# The highest level loop for this robot.
# Under this loop, the manual and independent mode is handled here
robot = Robot(ControllerMode.MANUAL)
#robot.turn(Direction.LEFT, 90)


