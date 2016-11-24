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
        self.pid_controller.set_tunings(1.4,0,0.3)
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
            turn_rate = self.read_sensor(Command.read_gyro()) / 100
            current_dir += (time.time() - clk) * turn_rate

            #print("Current dir: " + str(current_dir) + " -- Turn rate: " + str(turn_rate))
            #print("")

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
            left_speed = base_speed / ratio
            right_speed = base_speed * ratio
            drive_instr = Command.side_speeds(1, round(left_speed), 1, round(right_speed))
            uart_styrenhet.send_command(drive_instr)
            return

    def wait_for_empty_corridor(self, dir : Direction):
        while True:
            if (dir == Direction.RIGHT):
                back = self.read_sensor(Command.read_right_back_ir())
                front = self.read_sensor(Command.read_right_front_ir())
            else:
                back = self.read_sensor(Command.read_left_back_ir())
                front = self.read_sensor(Command.read_left_front_ir())

            if front > 200 and back > 200:
                return

    #Follow the walls until the robot gets a unexpected stop command or
    #the given distance to drive is reached.
    def follow_wall(self, distance : int):
        #Place LIDAR to 90 degrees
        servo_instr = Command.servo(90)
        uart_styrenhet.send_command(servo_instr)

        #Wait fot LIDAR to be in position
        time.sleep(1)
        
        #Read start values from sensors
        lidar = self.read_sensor(Command.read_lidar())
        start_lidar = self.read_sensor(Command.read_lidar())

        #Drive untill the wanted distance is reached
        while (start_lidar - lidar <= distance):
            # Get sensor values
            ir_right_front = self.read_sensor(Command.read_right_front_ir())
            ir_right_back = self.read_sensor(Command.read_right_back_ir())
            ir_left_front = self.read_sensor(Command.read_left_front_ir())
            ir_left_back = self.read_sensor(Command.read_left_back_ir())
            lidar = self.read_sensor(Command.read_lidar())

            print ("IR_BACK: " + str(ir_right_back) + " IR_FRONT : " + str(ir_right_front) + " LIDAR: " + str(lidar), "IR_LEFT_BACK", ir_left_back, "IR_LEFT_FRONT", ir_left_front)
            
            #Obstacle detected
            """if(lidar < 150):
                #Save the given length driven
                uart_styrenhet.send_command(Command.stop_motors())
                self.driven_distance += start_lidar - lidar
                self.save_position()
                return DriveStatus.OBSTACLE_DETECTED
            #Detect corridor to the right
            if (ir_right_front > 2* ir_right_back and ir_right_front > 200):
                #Save the given length driven
                self.wait_for_empty_corridor(Direction.RIGHT)
                uart_styrenhet.send_command(Command.stop_motors())
                self.driven_distance += start_lidar - lidar
                self.save_position()
                return DriveStatus.RIGHT_CORRIDOR_DETECTED
            elif (ir_left_front > 2 * ir_left_back and ir_left_front > 200):
                #Save the given length driven
                self.wait_for_empty_corridor(Direction.LEFT)
                uart_styrenhet.send_command(Command.stop_motors())
                self.driven_distance += start_lidar - lidar
                self.save_position()
                return DriveStatus.LEFT_CORRIDOR_DETECTED"""

            # We need to get the distance from the center of the robot perpendicular to the wall
            dist = (ir_right_front + ir_right_back) / 2
            angle = math.atan2(ir_right_back - ir_right_front, 95)  # 95 = distance between sensors
            perpendicular_dist = dist * math.cos(angle)
            self.pid_controller.input_data = perpendicular_dist
            self.pid_controller.compute()
            self.pid_controller.output_data += 100
            self.follow_wall_help(self.pid_controller.output_data / 100, 30)

        #Save the given length driven
        uart_styrenhet.send_command(Command.stop_motors())
        self.driven_distance += distance
        self.save_position()
        return DriveStatus.DONE

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

robot = Robot(ControllerMode.MANUAL)

#Try driving in infinite loop around the maze
while 1:
	#Drive forward without crashing in wall
	status = robot.follow_wall(1000)	
	if (status == DriveStatus.OBSTACLE_DETECTED):
		print ("---------- OBSTACLE DETECTED!")
		print ("---------- TURNING 180 degrees")
		robot.turn(Direction.LEFT, 90)
		robot.turn(Direction.LEFT, 90)
	elif (status == DriveStatus.LEFT_CORRIDOR_DETECTED):
		print("---------- DETECTED CORRIDOR TO LEFT!")
		print("---------- TURNING LEFT 90 degrees")
		robot.turn(Direction.LEFT, 90)
	elif (status == DriveStatus.RIGHT_CORRIDOR_DETECTED):
		print ("---------- DETECTED CORRIDOR TO RIGHT!")
		print ("---------- TURNING RIGHT 90 degrees")
		robot.turn(Direction.RIGHT, 90)





