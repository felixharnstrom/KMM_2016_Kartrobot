from bluetooth import *
from .communication import *

class Client:
    """A bluetooth client.

    A client can connect() to an advertising Server. Once connected, it
    is able to send() messages and get responses.

    Once the client is done, it can (and should) close() to disconnect. It
    can then connect to other Server's. Note that the server won't be
    automatically notified of the Client invoking close(), and will wait
    indefinetly for messages unless handled.
    """

    
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

        
class IPClient(Client):
    _port = 8080
    _addr = None

    def __init__(self, addr, port = 8080):
        self._addr = addr
        self._port = port


    def connect(self):
        """Returns true if succesful.

        Attempts to connect to server on "addr".
        Fails if already connected.
        """
        if self.connected():
            return False
        
        self._sock = connect_rfcomm_client_ip(self._addr, self._port)
        return self.connected()
