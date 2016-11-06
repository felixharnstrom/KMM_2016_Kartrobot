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

class Scan(StyrenhetFunction):
	TYPE = 0
	ARGUMENTS = {"degrees" : 0,
				"distance" : 0,
				"speed" : 0}

	def __init__(self, degrees : int, distance : int, speed : int):
		self.ARGUMENTS["degrees"] = degrees
		self.ARGUMENTS["distance"] = distance
		self.ARGUMENTS["speed"] = speed
		self.LENGTH = len(self.ARGUMENTS)