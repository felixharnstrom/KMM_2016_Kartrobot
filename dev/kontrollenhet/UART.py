import serial
from command import *
from bitstring import BitArray
import time

class UART:
    """
    Provides high-level UART-funcitonality.

    Args:
        :param port (str): Device name of the USB<->Serial converter for this unit.

    Attributes:
        :attribute port (str):      Device name of the USB<->Serial converter for this unit.
        :attribute ser  (Serial):   The serial port in use.
    """

    def __init__(self, port : str):
        # Public attributes
        self.port = port
        self.ser = serial.Serial('/dev/' + port, 38400)  # Open serial port

    def create_metapacket_hex(self, command : Command):
        """
        Create a hex packet consisting of adress, length and type

        Args:
            :param command (Command): The command to create the metapacket from.

        Returns:
            :return (str): A package of the command's metadata in form of a a hex string with two digits.
        """
        return '{:02X}'.format(command.address * (2 ** 7) + len(command.arguments) * (2 ** 4) + command.command_type)

    def create_metapacket(self, command : Command):
        """
        Create a byte-sized packet consisting of adress, length and type.

        Args:
            :param command (Command): The command to create the metapacket from.

        Returns:
            :return (bytes): a package of the command's metadata in form of a single byte
        """
        return bytes.fromhex(self.create_metapacket_hex(command))

    def send_arguments(self, command : Command):
        """
        Loops through all arguments and sends them (in the order specified in the functions arguments dictionary)

        Args:
            :param command (Command): The command of which to send the arguments.
        """
        for value in command.arguments:
            self.send_packet(bytes.fromhex('{:02X}'.format(int(value))))

    def send_packet(self, packet : bytes):
        """
        Sends a single packet.

        Args:
            :param packet (bytes): The packet to send.

        Returns:
            :return (int): Number of bytes written.
        """
        return self.ser.write(packet)

    def receive_packet(self):
        """
        Receives a single packet.

        Returns:
            :return (bytes): Bytes read from the port.
        """
        return self.ser.read()

    def send_command(self, command : Command):
        """
        Sends a command including arguments.

        Args:
            :param command (Command): The command to send.
        """
        self.send_packet(self.create_metapacket(command))
        self.send_arguments(command)

    def receive_command(self):
        """
        Receive a command.

        Returns:
            :return (Command): The received command.
        """
        (adr, length, msg_type) = self.decode_metapacket(self.receive_packet())
        arguments = []
        for i in range(length):
            arguments.append(int.from_bytes(self.receive_packet(), byteorder = 'big'))
        constructed = get_executable_command((msg_type + adr * 16), arguments)
        return constructed

    def receive_payload(self):
        """
        Receive packets as a list of integers.

        Returns:
            :return (list of int): Integer representation of all payload packets in order.
        """
        (adr, length, msg_type) = self.decode_metapacket(self.receive_packet())
        arguments = []
        for i in range(length):
            arguments.append(int.from_bytes(self.receive_packet(), byteorder = 'big'))
        return arguments   

    def decode_metapacket(self, packet : bytes):
        """
        Decodes metapacket to its components.

        Args:
            :param packet (bytes): The packet to decode.

        Returns:
            :return (tuple of int, int, int): tuple with (adress, length, type).
        """
        binary_packet = BitArray(uint = int.from_bytes(packet, byteorder = 'big', signed = False), length = 8)
        return (int(binary_packet.bin[0], 2), int(binary_packet.bin[1:4], 2), int(binary_packet.bin[4:], 2))

    def close(self):
        """
        Closes the connection.
        """
        self.ser.close()
