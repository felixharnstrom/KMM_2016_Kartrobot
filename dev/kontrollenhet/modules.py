from enum import Enum

class Adress(Enum):
	STYRENHET = 0
	SENSORENHET = 1

	
class Function():
	ADRESS = None
	TYPE = None
	LENGTH = 0
	ARGUMENTS = []

	
class StyrenhetFunction(Function):
	ADRESS = 0

	
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
				
	def __init__(self, direction : int, speed : int, time1 : int, time2 : int):
		self.ARGUMENTS.append(direction)
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

	enum 
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
		self.LENGTH = len()
