from function import *

class SensorenhetFunction(Function):
    ADRESS = 1
    
class ReadLeftFrontIr(SensorenhetFunction):
    TYPE = 1
    ARGUMENTS = []
    
class ReadLeftBackIr(SensorenhetFunction):
    TYPE = 2
    ARGUMENTS = []
    
class ReadRightFrontIr(SensorenhetFunction):
    TYPE = 3
    ARGUMENTS = []

    
class ReadRightBackIr(SensorenhetFunction):
    TYPE = 4
    ARGUMENTS = []

    
class ReadBackIr(SensorenhetFunction):
    TYPE = 5
    ARGUMENTS = []

class ReadLidar(SensorenhetFunction):
    TYPE = 6
    ARGUMENT = []

class ReadGyro(SensorenhetFunction):
    TYPE = 7
    ARGUMENT = []
