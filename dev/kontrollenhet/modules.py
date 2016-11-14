from styrenhet_functions import *
import numpy as np

class Functions(Enum):
    DRIVE = 1
    TURN = 2
    SIDE_SPEED = 3
    SERVO = 4
    STOP_MOTORS = 5
    CONTROLLER_INFORMATION = 6
    READ_IR_LEFT_FRONT = 11
    READ_IR_LEFT_BACK = 12
    READ_IR_RIGHT_FRONT = 13
    READ_IR_RIGHT_BACK = 14
    READ_IR_BACK = 15
    READ_LIDAR = 16
    READ_GYRO = 17

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
    elif(function_enum == Functions.READ_IR_LEFT_FRONT):
        return ReadLeftFrontIr()
    elif(function_enum == Functions.READ_IR_LEFT_BACK):
        return ReadLeftBackIr()
    elif(function_enum == Functions.READ_IR_RIGHT_FRONT):
        return ReadRightFrontIr()
    elif(function_enum == Functions.READ_IR_RIGHT_BACK):
        return ReadRightBackIr()
    elif(function_enum == Functions.READ_IR_BACK):
        return ReadBackIr()
    elif(function_enum == Functions.READ_LIDAR):
        return ReadLidar()
    elif(function_enum == Functions.READ_GYRO):
        return ReadGyro()
    else:
    #Something has gone wrong, print that.
        print("An incorrect function number has been given, number=",function_number)

