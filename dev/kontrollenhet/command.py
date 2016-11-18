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
    def ack()
        return Command(CommandEnums.ACK)
        
    @staticmethod
    def drive(direction : int, speed : int, time : int):
        time1, time2 = split_time(time)
        return Command(CommandEnums.DRIVE, [direction, speed, time1, time2])

    @staticmethod
    def turn(turn : int, speed : int, time : int):
        time1, time2 = split_time(time)
        return Command(CommandEnums.TURN, [turn, speed, time1, time2])

    @staticmethod
    def side_speeds(left_direction : int, left_speed: int, right_direction : int, right_speed : int):
        return Command(CommandEnums.SIDE_SPEEDS, [left_direction, left_speed, right_direction, right_speed])

    @staticmethod
    def servo(degrees : int):
        return Command(CommandEnums.SERVO, [degrees])

    @staticmethod
    def stop_motors():
        return Command(CommandEnums.STOP_MOTORS)

    @staticmethod
    def controller_information():
        return Command(CommandEnums.CONTROLLER_INFORMATION)

    @staticmethod
    def read_left_front_ir():
        return Command(CommandEnums.READ_IR_LEFT_FRONT)

    @staticmethod
    def read_left_back_ir():
        return Command(CommandEnums.READ_IR_LEFT_BACK)

    @staticmethod
    def read_right_front_ir():
        return Command(CommandEnums.READ_IR_RIGHT_FRONT)

    @staticmethod
    def read_right_back_ir():
        return Command(CommandEnums.READ_IR_RIGHT_BACK)

    @staticmethod
    def read_back_ir():
        return Command(CommandEnums.READ_IR_BACK)

    @staticmethod
    def read_lidar():
        return Command(CommandEnums.READ_LIDAR)

    @staticmethod
    def read_gyro():
        return Command(CommandEnums.READ_GYRO)

    def get_enum(self):
        return CommandEnums((self.address * 16) + self.command_type)

def split_time(time):
    time_16 = np.uint16(time)
    mask_high = 0xFF00
    mask_low = 0x00FF
    time_high = np.uint8((time_16 & mask_high) >> 8)
    time_low = np.uint8((time_16 & mask_low))
    return int(time_high), int(time_low)

def get_executable_command(command_number : int, params = []):
    command_enum = CommandEnums(command_number)
    return Command(command_enum, params)

