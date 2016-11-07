import serial
import modules

class UART:
    def __init__(self, port : str):
        self.port = port
        self.ser = serial.Serial('/dev/'+port, 9600)  # open serial port


    def create_metapacket_hex(self, function : modules.Function):
        return '{:02X}'.format(function.ADRESS*(2**7)+function.LENGTH*(2**4)+function.TYPE)

    def create_metapacket(self, function : modules.Function):
        return bytes.fromhex(self.create_metapacket_hex(function))

    def send_arguments(self, function : modules.Function):
        for value in function.ARGUMENTS:
            self.send_packet(bytes.fromhex('{:02X}'.format(function.ARGUMENTS[value])))

    def send_packet(self, packet : bytes):
        # Debug print
        # print(int.from_bytes(packet,byteorder = 'big'))
        self.ser.write(packet)

    def receive_packet(self):
        return self.ser.read()

    def send_function(self, function : modules.Function):
        self.send_packet(self.create_metapacket(function))
        self.send_arguments(function)

    def close(self):
        self.ser.close()