import numpy as np
from enum import Enum

class CommandEnums(Enum):
    ACK = 0
    DRIVE = 1
    TURN = 2
    SIDE_SPEEDS = 3
    SERVO = 4
    STOP_MOTORS = 5
    CONTROLLER_INFORMATION = 6
    READ_IR_LEFT_FRONT = 17
    READ_IR_LEFT_BACK = 18
    READ_IR_RIGHT_FRONT = 19
    READ_IR_RIGHT_BACK = 20
    READ_IR_BACK = 21
    READ_LIDAR = 22
    READ_GYRO = 23

class Command():
    def __init__(self, command_type : Enum, arguments = []):
        if (command_type.value < 16):
            self.address = 0
        else:
            self.address = 1

        self.command_type = command_type.value - (self.address * 16)
        self.arguments = arguments

    @staticmethod
    def ack():
        return Command(CommandEnums.ACK)
        
    @staticmethod
    def drive(direction : int, speed : int, time : int):
        time1, time2 = splitTime(time)
        return Command(Command_enums.DRIVE, [direction, speed, time1, time2])

    @staticmethod
    def turn(turn : int, speed : int, time : int):
        time1, time2 = splitTime(time)
        return Command(Command_enums.TURN, [turn, speed, time1, time2])

    @staticmethod
    def sideSpeeds(leftDirection : int, leftSpeed: int, rightDirection : int, rightSpeed : int):
        return Command(Command_enums.SIDE_SPEEDS, [leftDirection, leftSpeed, rightDirection, rightSpeed])

    @staticmethod
    def servo(degrees : int):
        return Command(Command_enums.SERVO, [degrees])

    @staticmethod
    def stopMotors():
        return Command(Command_enums.STOP_MOTORS)

    @staticmethod
    def controllerInformation():
        return Command(Command_enums.CONTROLLER_INFORMATION)

    @staticmethod
    def readLeftFrontIr():
        return Command(Command_enums.READ_IR_LEFT_FRONT)

    @staticmethod
    def readLeftBackIr():
        return Command(Command_enums.READ_IR_LEFT_BACK)

    @staticmethod
    def readRightFrontIr():
        return Command(Command_enums.READ_IR_RIGHT_FRONT)

    @staticmethod
    def readRightBackIr():
        return Command(Command_enums.READ_IR_RIGHT_BACK)

    @staticmethod
    def readBackIr():
        return Command(Command_enums.READ_IR_BACK)

    @staticmethod
    def readLidar():
        return Command(Command_enums.READ_LIDAR)

    @staticmethod
    def readGyro():
        return Command(Command_enums.READ_GYRO)

    def getEnum(self):
        return Command_enums((self.address * 16) + self.command_type)

def splitTime(time):
    time_16 = np.uint16(time)
    mask_high = 0xFF00
    mask_low = 0x00FF
    time_high = np.uint8((time_16 & mask_high) >> 8)
    time_low = np.uint8((time_16 & mask_low))
    return int(time_high), int(time_low)

def getExecutableCommand(command_number : int, params = []):
    command_enum = Command_enums(command_number)
    return Command(command_enum, params)

