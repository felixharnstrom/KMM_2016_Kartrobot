from bluetooth import *
from .communication import *

class Client:
    
    """The server socket the client is connected to."""
    _sock = None

    
    def connected(self):
        """Returns True if connected to a server."""
        return not (self._sock is None)
    
    def connect(self, addr=None):
        """Returns true if succesful.

        Attempts to connect to an advertised server.
        Fails if already connected.

        Raises BluetoothError if the connection is refused by the server.
        """
        if self.connected():
            return False
        
        self._sock = connect_rfcomm_client(addr)
        return self.connected()

    def send(self, byte_data):
        """Returns server response as bytes, or None.

        Send a message to the connected server and return answer.
        Returns None if not connected.

        Raises BluetoothError if sending fails.
        """
        if not self.connected():
            return None
        send_bytes(self._sock, byte_data)
        return receive_bytes(self._sock)

    def close(self):
        """Close the connection, if connected."""
        if self.connected():
            self._sock.close()
            self._sock = None

        
