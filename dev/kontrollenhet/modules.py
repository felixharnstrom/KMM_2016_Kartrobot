from enum import Enum



class Adress(Enum):
	STYRENHET = 0
	SENSORENHET = 1


class Function():
	ADRESS = None
	TYPE = None
	LENGTH = None
	ARGUMENTS = {}
	LENGTH = len(ARGUMENTS)

class StyrenhetFunction(Function):
	ADRESS = 0

	

class Drive(StyrenhetFunction):
	TYPE = 1
	ARGUMENTS = []

	def __init__(self, direction : int, speed : int, time1 : int, time2 : int):
		self.ARGUMENTS = []
		self.ARGUMENTS.append(direction)
		self.ARGUMENTS.append(speed)
		self.ARGUMENTS.append(time1)
		self.ARGUMENTS.append(time2)
		self.LENGTH = len(self.ARGUMENTS)

		
class Turn(StyrenhetFunction):
	TYPE = 2
	ARGUMENTS = {"direction" : 0, #0 left, 1 right
				"speed" : 0, #0-100
				"time1" : 0, #2^8*time1 + time2 ms
				"time2" : 0}

	def __init__(self, direction : int, speed : int, time1 : int, time2 : int):
		self.ARGUMENTS["direction"] = direction
		self.ARGUMENTS["speed"] = speed
		self.ARGUMENTS["time1"] = time1
		self.ARGUMENTS["time2"] = time2
		self.LENGTH = len(self.ARGUMENTS)


class SideSpeed(StyrenhetFunction):
	TYPE = 3
	ARGUMENTS = {"side" : 0, #0 left, 1 right
				"direction" : 0, #0 backward, 1 forward
				"speed" : 0 #0 - 100
				}

	def __init__(self, side : int, direction : int, speed : int):
		self.ARGUMENTS["side"] = side
		self.ARGUMENTS["direction"] = direction
		self.ARGUMENTS["speed"] = speed
		self.LENGTH = len(self.ARGUMENTS)


class Servo(StyrenhetFunction):
	TYPE = 4
	ARGUMENTS = {"degrees" : 0}

	enum 
	def __init__(self, degrees : int):
		self.ARGUMENTS["degrees"] = degrees #0 - 180
		self.LENGTH = len(self.ARGUMENTS)
		

class StopMotors(StyrenhetFunction):
	TYPE = 5
	ARGUMENTS = {}

	def __init__(self):
		self.LENGTH = 0