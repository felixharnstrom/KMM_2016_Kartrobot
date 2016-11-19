from command import Command
import time
import math
from UART import UART

#Define directions
class Direction:
	LEFT = 0
	RIGHT = 1

# Return a sensor value
def read_sensor(sensor_instr : Command):
	#TODO: define general uart variables
	uart_sensorenhet = UART("ttyUSB0")
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

def turn(direction : Direction, degrees : int):
	#TODO: define general uart variables
	uart_styrenhet = UART("ttyUSB1")

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

#Drive the given distance in millimeters
def drive(distance : int):
	#TODO: define general uart variables
	#Point the LIDAR sensor straight ahead
	uart_styrenhet = UART("ttyUSB1")
	servo_instr = Command.servo(90)
	uart_styrenhet.send_command(servo_instr)

	#Wait for LIDAR to be in position
	time.sleep(1)

	#Start the engines
	drive_instr = Command.drive(1,30,0)
	uart_styrenhet.send_command(drive_instr)

	#Read the current distance
	average_start = mean_sensor(5, Command.read_lidar())
	goal_distance = average_start - distance
	current_distance = average_start

	while (current_distance > goal_distance):
		current_distance = read_sensor(Command.read_lidar())
		print("Lidar: " + str(current_distance))
	
	#Stop the motors
	drive_instr = Command.stop_motors()
	uart_styrenhet.send_command(drive_instr)	

#Turn the robot so that the IR sensor are almost equal on both sides
def adjust_direction():
	while (True):
		print("Gyro: " + str(read_sensor(Command.read_gyro()) / 1000))
		#Read all IR sensors on the left and right side
		#ir_left_front = read_sensor(Command.read_left_front_ir())
		#ir_left_back = read_sensor(Command.read_left_back_ir())
		#ir_right_front = read_sensor(Command.read_right_front_ir())
		#ir_right_back = read_sensor(Command.read_right_back_ir())
		#print ("LEFT_FRONT: " + str(ir_left_front) + " LEFT_BACK: " + str(ir_left_back) + " RIGHT_FRONT: " + str(ir_right_front) + " RIGHT_BACK : " + str(ir_right_back))
			

adjust_direction()


