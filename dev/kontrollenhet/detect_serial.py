import sys
from UART import UART
from robot_communication import read_gyro_sensor, init_UARTs
from command import Command
from timeout import *

@timeout(1)
def try_reading_gyro():
    """
    Try to read a sensor with a uniqe msg_type.
    Sensorunit will return instantly, controlunit will timeout.
    """
    read_gyro_sensor(Command.read_gyro())


if __name__ == "__main__":
    serials = []
    # Read all USB serial ports.
    # In practice, this will never be more than two, and will not work with more
    for line in sys.stdin:
        serials.append(line.rstrip())

    init_UARTs(serials[0], serials[1])
    try:
        # If we get a valid response, the ports are correct.
        try_reading_gyro()
        print("Sensor unit is at", serials[0])
        print("Control unit is at", serials[1])
    except TimeoutError:
        # If the request times out, the ports are reversed.
        init_UARTs(serials[1], serials[0])
        print("Sensor unit is at", serials[1])
        print("Control unit is at", serials[0])
