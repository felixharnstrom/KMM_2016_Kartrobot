
from com.server import *
from com.client import *

# A test for the current functionality

msg = (1337).to_bytes(8, byteorder="big")

if input() == "":
    print("client")
    client = Client()
    if client.connect():
        print("sending", msg)
        print(client.send(msg))
        client.close()
    else:
        print("Connection failed")
else:
    print("server")
    server = Server()
    server.advertise_and_connect()
    print(server.receive())
    server.send(msg)
    server.close()
