from bluetooth import *
from .communication import *


class Server:

    
    """The server socket"""
    _server_sock = None


    """The client socket"""
    _client_sock = None

    
    def connected(self):
        """Returns True if connected to a server"""
        return not (self._server_sock is None)
    
    def advertise_and_connect(self):
        """Advertises itself and waits for a client to connect."""
        if self.connected():
            return        
        self._server_sock, self._client_sock = connect_rfcomm_server()

    def send(self, byte_data):
        """Returns False if not connected, True otherwise

        Send a message to the connected server."""
        if not self.connected():
            return False
        send_bytes(self._client_sock, byte_data)
        return True

    def receive(self):
        """Returns received bytes"""
        return receive_bytes(self._client_sock)

    def close(self):
        """Close the connection, if connected"""
        if self.connected():
            self._client_sock.close()
            self._client_sock = None
            self._server_sock.close()
            self._server_sock = None

        
