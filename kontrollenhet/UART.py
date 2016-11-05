import serial

class UART:
    def __init__(self, port : str):
        self.port = port
        self.ser = serial.Serial('/dev/'+port, 9600)  # open serial port


    def create_metapacket(self, addr : int, length : int, action : int):
        return bytes.fromhex(hex(addr*(2**7)+length*(2**4)+action).split('x')[1])


    def create_metapacket_hex(self, addr : int, length : int, action : int):
        return hex(addr*(2**7)+length*(2**4)+action).split('x')[1]

    def send_packet(self, packet : bytes):
        self.ser.write(packet)

    def receive_packet(self):
        return self.ser.read()

    def close(self):
        self.ser.close()