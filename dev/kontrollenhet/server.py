import socket

class server():
	socket = socket.socket()         # Create a socket object

	client = None

	def start(self, port = 12345, ip = "localhost"):
		self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

		self.socket.bind(("", port))        # Bind to the port

		self.socket.listen(5)                 # Now wait for client connection.

	def connect(self):
		self.client, addr = self.socket.accept() 

	def close(self):
		self.client.close()