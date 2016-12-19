import socket

class client():
    """
    Creates a simple socket which can be connected and closed.
    
    Attributes:
        :attribute client (socket): The client socket.
    """
    def __init__(self):
        self.client = socket.socket()
    
    def start(self, port = 12345, ip = "localhost"):
        """
        Connects the client socket to the given ip and port number.
        
        Args:
            :param port (int): The port number to use, defaults to 12345.
            :param ip   (str): The ip-adress to use, defaults to 'localhost'.
        """
    
        port = 12345                # Reserve a port for your service.
        self.client.connect((ip, port))        # Bind to the port

    def close(self):
        """
        Closes the client socket connection.
        """
        self.client.close()