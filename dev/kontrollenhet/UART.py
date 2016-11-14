import serial
import modules
from bitstring import BitArray
import time

class UART:
    def __init__(self, port : str):
        self.port = port
        self.ser = serial.Serial('/dev/'+port, 9600)  # open serial port

    def create_metapacket_hex(self, function : modules.Function):
        """
        Create a hex packet consisting of adress, length and type

        :function: The function to create the metapacket from.
        :return: a package of the functions metadata in form of a a hex string with two digits
        """
        return '{:02X}'.format(function.ADRESS*(2**7)+function.LENGTH*(2**4)+function.TYPE)

    def create_metapacket(self, function : modules.Function):
        """
        Create a byte-sized packet consisting of adress, length and type

        :function: The function to create the metapacket from.
        :return: a package of the functions metadata in form of a single byte
        """
        return bytes.fromhex(self.create_metapacket_hex(function))

    def send_arguments(self, function : modules.Function):
        """
        Loops through all arguments and sends them (in the order specified in the functions arguments dictionary)

        :function: The function of which to send the arguments.
        """
        for value in function.ARGUMENTS:
            self.send_packet(bytes.fromhex('{:02X}'.format(value)))

    def send_packet(self, packet : bytes):
        """
        Sends a single packet.

        :packet: The packet to send
        """
        # Debug print
        #print(int.from_bytes(packet,byteorder = 'big'))
        time.sleep(0.1)
        return self.ser.write(packet)

    def receive_packet(self):
        """
        Receives a single packet
        """
        return self.ser.read()

    def send_function(self, function : modules.Function):
        """
        Sends a function command including arguments

        :function: The function to send
        """
        self.send_packet(self.create_metapacket(function))
        self.send_arguments(function)
        #self.ser.flush()

    def decode_metapacket(self, packet : bytes):
        """
        Decodes metapacket to its components.

        :packet: The packet to decode
        :return: tuple with (adress, length, type)
        """
        binary_packet = BitArray(uint=int.from_bytes(packet, byteorder='big', signed=False),  length=8)
        return (int(binary_packet.bin[0], 2), int(binary_packet.bin[1:4], 2), int(binary_packet.bin[4:], 2))

    def close(self):
        """
        Closes the connection
        """
        self.ser.close()
