from command import Command
import time
import math
from UART import UART
from pid import Pid

#Define your USB ports
global uart_sensorenhet = UART("ttyUSB0")
global uart_styrenhet = UART("ttyUSB1")

#Define directions
class Direction:
	LEFT = 0
	RIGHT = 1
	
class Robot:
	controller = 0
	driven_distance = 0

	def __init__(self):
		#Initialize PID
		self.controller = Pid() 
		self.controller.setpoint = 80
		self.controller.output_data = 0
		self.controller.set_tunings(1.5,5,10)
		self.controller.set_sample_time(160)
		self.controller.set_output_limits(75,125)
		self.controller.set_mode(1)
		
		#The current driven distance
		self.driven_distance = 0

	# Return a sensor value
	def read_sensor(self, sensor_instr : Command):
		uart_sensorenhet.handle_command(sensor_instr)
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
		uart_styrenhet.handle_command(turn_instr)

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
		uart_styrenhet.handle_command(turn_instr)

	#Drive the given distance in millimeters
	#This code do not care about the walls on your left or right side
	def drive_distance(self, distance : int):
		#Point the LIDAR sensor straight ahead
		servo_instr = Command.servo(90)
		uart_styrenhet.handle_command(servo_instr)

		#Wait for LIDAR to be in position
		time.sleep(1)

		#Start the engines
		drive_instr = Command.drive(1,30,0)
		uart_styrenhet.handle_command(drive_instr)

		#Read the current distance
		average_start = mean_sensor(5, Command.read_lidar())
		goal_distance = average_start - distance
		current_distance = average_start

		while (current_distance > goal_distance):
			current_distance = read_sensor(Command.read_lidar())
			print("Lidar: " + str(current_distance))
		
		#Stop the motors
		drive_instr = Command.stop_motors()
		uart_styrenhet.handle_command(drive_instr)	
		
	#Follow the walls until the robot gets a unexpected stop command or
	#the given distance to drive is reached. 
	def follow_wall(self, controller : Pid, distance : int):
		return
		
	#Return instruction for what to do next
	def what_to_do_now(self):

	#Scan the room at this position
	def scan(self):
		return

	# Drive independent through the maze
	def	autonom_mode(self):
		return
	
# The highest level loop for this robot.
# Under this loop, the manual and independent mode is handled here
while 1:
	
		


