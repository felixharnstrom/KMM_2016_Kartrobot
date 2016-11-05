import serial
import modules

class UART:
    def __init__(self, port : str):
        self.port = port
        self.ser = serial.Serial('/dev/'+port, 9600)  # open serial port


    def create_metapacket_hex(self, addr : modules.Adress, length : int, action : modules.Functions):
        return '{:02X}'.format(addr.value*(2**7)+int(length)*(2**4)+action.value)

    def create_metapacket(self, addr : int, length : int, action : int):
        return bytes.fromhex(self.create_metapacket_hex(addr, length, action))

    def send_packet(self, packet : bytes):
        self.ser.write(packet)

    def receive_packet(self):
        return self.ser.read()

    def close(self):
        self.ser.close()