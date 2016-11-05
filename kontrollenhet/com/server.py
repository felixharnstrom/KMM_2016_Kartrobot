from bluetooth import *
from .communication import *


class Server:
    """A bluetooth server.

    After connecting to a Client, it is able to talk to said Client using
    the send() and receive() methods.

    The client can't receive a sent message at any time. Instead, the server
    is meant to process requests from the client.

    The server is to receive() a message from the client. Once that arrives
    and it finishes processing it, it can (and should) send() a reply. Repeat.

    Once the server has finished it's duty, close() it to disconnect. A closed
    server is then able to connect to another Client. Note that the Client()
    won't be automatically notified of this, and will raise BluetoothErrors as
    it attempts to send to a non-connected server, unless handled.
    """

    
    """The server socket"""
    _server_sock = None


    """The client socket."""
    _client_sock = None

    
    def connected(self):
        """Returns True if connected to a server."""
        return not (self._server_sock is None)
    
    def advertise_and_connect(self):
        """Advertises itself and waits for a client to connect.
        
        Raises BluetoothError bluetooth error if accepting connection fails.
        """
        if self.connected():
            return        
        self._server_sock, self._client_sock = connect_rfcomm_server()

    def send(self, byte_data):
        """Returns False if not connected, True otherwise.

        Send a message to the connected server.

        Raises BluetoothError if sending fails.
        """
        if not self.connected():
            return False
        send_bytes(self._client_sock, byte_data)
        return True

    def receive(self):
        """Returns received bytes.
        
        Raises BluetoothError on failure.
        """
        return receive_bytes(self._client_sock)

    def close(self):
        """Close the connection, if connected."""
        if self.connected():
            self._client_sock.close()
            self._client_sock = None
            self._server_sock.close()
            self._server_sock = None

        
