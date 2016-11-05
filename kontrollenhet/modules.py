from enum import Enum

class Functions(Enum):
	pass

class StyrenhetFunctions(Functions):
	SCAN = 0
	DRIVE = 1

class Adress(Enum):
	STYRENHET = 0
	SENSORENHET = 1