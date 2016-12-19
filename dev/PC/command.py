import numpy as np
from enum import Enum

class CommandEnums(Enum):
    """
    Each enum represents a different command, identified in the UART library by 4 bits.
    This limits us to 16 different commands for each unit, with the address bit used for determining the corresponding enum number according to:

        enum number = address bit * 16 + UART command number

    The ACK command is unit-agnostic and thus only needs one representation.
    """

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
    """
    Represents a single command that can be executed by the sensor or control unit.

    Args:
        :param command_type (CommandEnums): The command type.
        :param arguments    (list of int):  A list of arguments, the meaning of which is dependent on the command type. 

    Attributes:
        :attribute address      (int):          0 for control unit, 1 for sensor unit.
        :attribute command_type (CommandEnums): The command type.
        :attribute arguments    (list of int):  Command arguments.
    """

    def __init__(self, command_type : CommandEnums, arguments = []):
        if (command_type.value < 16):
            self.address = 0
        else:
            self.address = 1

        self.command_type = command_type.value - (self.address * 16)
        self.arguments = arguments

    @staticmethod
    def ack():
        """
        Returns an acknowledge.

        Returns:
            :return (Command): Acknowledge command.
        """
        return Command(CommandEnums.ACK)

    @staticmethod
    def drive(direction : int, speed : int, time : int):
        """
        Drive in the given direction at the given speed and time.

        Args:
            :param direction    (int): 1 for forward, 0 for backward.
            :param speed        (int): Speed as a percentage value between 0-100.
            :param time         (int): Time in milliseconds.

        Returns:
            :return (Command): Drive command.
        """
        time1, time2 = split_time(time)
        return Command(CommandEnums.DRIVE, [direction, speed, time1, time2])

    @staticmethod
    def turn(turn : int, speed : int, time : int):
        """
        Turn in the given direction at the given speed and time.

        Args:
            :param direction    (int): 1 for right, 0 for left.
            :param speed        (int): Speed as a percentage value between 0-100.
            :param time         (int): Time in milliseconds.

        Returns:
            :return (Command): Turn command.
        """
        time1, time2 = split_time(time)
        return Command(CommandEnums.TURN, [turn, speed, time1, time2])

    @staticmethod
    def side_speeds(left_direction : int, left_speed: int, right_direction : int, right_speed : int):
        """
        Set direction and speed for each motor individually.

        Args:
            :param left_direction    (int): 1 for forward, 0 for backward.
            :param left_speed        (int): Speed as a percentage value between 0-100.
            :param right_direction    (int): 1 for forward, 0 for backward.
            :param right_speed        (int): Speed as a percentage value between 0-100

        Returns:
            :return (Command): Side speeds command.
        """
        return Command(CommandEnums.SIDE_SPEEDS, [left_direction, left_speed, right_direction, right_speed])

    @staticmethod
    def servo(degrees : int):
        """
        Turn the LIDAR sensor.

        Args:
            :param degrees    (int): ? TODO: Absolute or relative?

        Returns:
            :return (Command): Servo command.
        """
        return Command(CommandEnums.SERVO, [degrees])

    @staticmethod
    def stop_motors():
        """
        Stops both motors.

        Returns:
            :return (Command): Stop command.
        """
        return Command(CommandEnums.STOP_MOTORS)

    @staticmethod
    def controller_information():
        """
        Query control unit for diagnostics.

        Returns:
            :return (Command): Controller information command.
        """
        return Command(CommandEnums.CONTROLLER_INFORMATION)

    @staticmethod
    def read_left_front_ir():
        """
        Query current left front IR sensor value.

        Returns:
            :return (Command): Read left front IR command.
        """
        return Command(CommandEnums.READ_IR_LEFT_FRONT)

    @staticmethod
    def read_left_back_ir():
        """
        Query current left back IR sensor value.

        Returns:
            :return (Command): Read left back IR command.
        """
        return Command(CommandEnums.READ_IR_LEFT_BACK)

    @staticmethod
    def read_right_front_ir():
        """
        Query current right front IR sensor value.

        Returns:
            :return (Command): Read right front IR command.
        """
        return Command(CommandEnums.READ_IR_RIGHT_FRONT)

    @staticmethod
    def read_right_back_ir():
        """
        Query current right back IR sensor value.

        Returns:
            :return (Command): Read right back IR command.
        """
        return Command(CommandEnums.READ_IR_RIGHT_BACK)

    @staticmethod
    def read_back_ir():
        """
        Query current back IR sensor value.

        Returns:
            :return (Command): Read back IR command.
        """
        return Command(CommandEnums.READ_IR_BACK)

    @staticmethod
    def read_lidar():
        """
        Query current LIDAR value.

        Returns:
            :return (Command): Read LIDAR command.
        """
        return Command(CommandEnums.READ_LIDAR)

    @staticmethod
    def read_gyro():
        """
        Query current gyrovalue.

        Returns:
            :return (Command): Read gyro command.
        """
        return Command(CommandEnums.READ_GYRO)

    def get_enum(self):
        """
        Returns the enum corresponding to this command.

        Returns:
            :return (CommandEnums): This command's corresponding CommandEnum.
        """
        return CommandEnums((self.address * 16) + self.command_type)

def split_time(time : int):
    """
    Splits a 16 bit integer in two 8-bit integers containing the 8 most and least significant bits, respectively.

    Args:
        :param time (int): A 16-bit unsigned integer.

    Returns:
        :return (int): 8 most significant bits.
        :return (int): 8 least significant bits.
    """
    time_16 = np.uint16(time)
    mask_high = 0xFF00
    mask_low = 0x00FF
    time_high = np.uint8((time_16 & mask_high) >> 8)
    time_low = np.uint8((time_16 & mask_low))
    return int(time_high), int(time_low)

def get_executable_command(command_number : int, params = []):
    """
    Constructs a Command from the given command number and command parameters.

    Args:
        :param command_number   (int):          The command number, corresponding to the CommandEnum values.
        :param params           (list of int):  Command parameters.

    Returns:
        :return (Command): Command with the corresponding CommandEnum and parameters.
    """
    command_enum = CommandEnums(command_number)
    return Command(command_enum, params)