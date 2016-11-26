import serial
from command import *
from bitstring import BitArray
import time

class UART:
    def __init__(self, port : str):
        self.port = port
        self.ser = serial.Serial('/dev/'+port, 38400)  # open serial port

    def create_metapacket_hex(self, command : Command):
        """
        Create a hex packet consisting of adress, length and type

        :command: The command to create the metapacket from.
        :return: a package of the command's metadata in form of a a hex string with two digits
        """
        return '{:02X}'.format(command.address*(2**7)+len(command.arguments)*(2**4)+command.command_type)

    def create_metapacket(self, command : Command):
        """
        Create a byte-sized packet consisting of adress, length and type

        :command: The command to create the metapacket from.
        :return: a package of the command's metadata in form of a single byte
        """
        return bytes.fromhex(self.create_metapacket_hex(command))

    def send_arguments(self, command : Command):
        """
        Loops through all arguments and sends them (in the order specified in the functions arguments dictionary)

        :function: The function of which to send the arguments.
        """
        for value in command.arguments:
            self.send_packet(bytes.fromhex('{:02X}'.format(value)))

    def send_packet(self, packet : bytes):
        """
        Sends a single packet.

        :packet: The packet to send
        """
        # Debug print
        #print(int.from_bytes(packet,byteorder = 'big'))
        #time.sleep(0.1)
        return self.ser.write(packet)

    def receive_packet(self):
        """
        Receives a single packet
        """
        return self.ser.read()

    def send_command(self, command : Command):
        """
        Sends a command including arguments

        :command: The command to send
        """
        self.send_packet(self.create_metapacket(command))
        self.send_arguments(command)
        #self.ser.flush()

    def receive_command(self):
        """Receive a command."""
        (adr, length, msg_type) = self.decode_metapacket(self.receive_packet())
        arguments = []
        for i in range(length):
            arguments.append(int.from_bytes(self.receive_packet(), byteorder='big'))
        constructed = get_executable_command((msg_type + adr*16), arguments)
        return constructed

    def receive_payload(self):
        """Receive packets as a list of integers"""
        (adr, length, msg_type) = self.decode_metapacket(self.receive_packet())
        arguments = []
        for i in range(length):
            arguments.append(int.from_bytes(self.receive_packet(), byteorder='big'))
        return arguments   

    def decode_metapacket(self, packet : bytes):
        """
        Decodes metapacket to its components.

        :packet: The packet to decode
        :return: tuple with (adress, length, type)
        """
        binary_packet = BitArray(uint=int.from_bytes(packet, byteorder='big', signed=False), length=8)
        return (int(binary_packet.bin[0], 2), int(binary_packet.bin[1:4], 2), int(binary_packet.bin[4:], 2))

    def close(self):
        """
        Closes the connection
        """
        self.ser.close()
