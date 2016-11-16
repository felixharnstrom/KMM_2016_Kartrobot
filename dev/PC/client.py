import socket

class client():
    client = socket.socket()         # Create a socket object

    def start(self, port = 12345, ip = "localhost"):
        port = 12345                # Reserve a port for your service.
        self.client.connect((ip, port))        # Bind to the port

    def close(self):
        self.client.close()