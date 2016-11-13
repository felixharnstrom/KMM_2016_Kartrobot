from enum import Enum
import numpy as np

class Adress(Enum):
	STYRENHET = 0
	SENSORENHET = 1

class Functions(Enum):
	DRIVE = 1
	TURN = 2
	SIDE_SPEED = 3
	SERVO = 4
	STOP_MOTORS = 5
	CONTROLLER_INFORMATION = 6
	
class Function():
	ADRESS = None
	TYPE = None
	LENGTH = 0
	ARGUMENTS = []
	
class StyrenhetFunction(Function):
	ADRESS = 0

def splitTime(times): #Taken from robot_com
	time_16 = np.uint16(times)
	mask_high = 0xFF00
	mask_low = 0x00FF
	time_high = np.uint8((time_16 & mask_high) >> 8)
	time_low = np.uint8((time_16 & mask_low))
	return [time_high, time_low]
	
#Returns a StyrenhetFunction object for the correct integer (params = dict) 
def GetExecutableFunction(function_number : int, params={}):
	function_enum = Functions(function_number)    #Should work
	#Params should be supplied correctly, otherwise we will have to handle it (TODO: implement check)
	if(function_enum == Functions.DRIVE):
		times = splitTime(params['TIME'])
		return Drive(params['DIRECTION'], params['SPEED'], times[0], times[1])
	elif(function_enum == Functions.TURN):
		times = splitTime(params['TIME'])
		return Turn(params['DIRECTION'], params['SPEED'], times[0], times[1])
	elif(function_enum == Functions.SIDE_SPEED):
		return SideSpeed(params['SIDE'], params['DIRECTION'], params['SPEED'])
	elif(function_enum == Functions.SERVO):
		return Servo(params['DEGREES'])
	elif(function_enum == Functions.STOP_MOTORS):
		return StopMotors()
	elif(function_enum == Functions.CONTROLLER_INFORMATION):
		return ControllerInformation()
	else:
		#Something has gone wrong, print that.
		print("An incorrect function number has been given, number=",function_number)


class Drive(StyrenhetFunction):
	TYPE = 1
	ARGUMENTS = []

	def __init__(self, direction : int, speed : int, time1 : int, time2 : int):
		self.ARGUMENTS.append(direction)
		self.ARGUMENTS.append(speed)
		self.ARGUMENTS.append(time1)
		self.ARGUMENTS.append(time2)
		self.LENGTH = len(self.ARGUMENTS)
		
		
class Turn(StyrenhetFunction):
	TYPE = 2
	ARGUMENTS = []
	#0 left, 1 right	
	#0-100
	#time = 2^8*time1 + time2 ms
				
	def __init__(self, turn : int, speed : int, time1 : int, time2 : int):
		self.ARGUMENTS.append(turn)
		self.ARGUMENTS.append(speed)
		self.ARGUMENTS.append(time1)
		self.ARGUMENTS.append(time2)
		self.LENGTH = len(self.ARGUMENTS)


class SideSpeed(StyrenhetFunction):
	TYPE = 3
	ARGUMENTS = []
	#0 left, 1 right
	#0 backward, 1 forward
	#0 - 100

	def __init__(self, side : int, direction : int, speed : int):
		self.ARGUMENTS.append(side)
		self.ARGUMENTS.append(direction)
		self.ARGUMENTS.append(speed)
		self.LENGTH = len(self.ARGUMENTS)
	

class Servo(StyrenhetFunction):
	TYPE = 4
	ARGUMENTS = []

	def __init__(self, degrees : int):
		self.ARGUMENTS.append(degrees) #0 - 180
		self.LENGTH = len(self.ARGUMENTS)
		

class StopMotors(StyrenhetFunction):
	TYPE = 5
	ARGUMENTS = []

	def __init__(self):
		self.LENGTH = 0
		
class ControllerInformation(StyrenhetFunction):
	TYPE = 6
	ARGUMENTS = []
	#TODO: Implement pack receive method on python side
	#Packet 0, 1 contains left side information
	#Packet 2, 3 contains right side information
	#0,2 contains motor direction, 0=backward, 1=forward
	#1,3 contains motor PWM(M) (M/2.5 for speed percentage)
	#4,5 contains servo PWM (S) 4 being MSB, 5 LSB eg S=[4]*2^8+[5] ((S-708)/8.45 for gyro angle)
	
	def __init__(self):
		self.LENGTH = 0