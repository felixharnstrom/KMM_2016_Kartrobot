
from server import *
from client import *
import time, sys
from random import randrange

# A test for the current functionality

def test_client():
    print("client")
    client = Client()
    if client.connect():
        while True:
            num = randrange(0, 65535)
            msg = num.to_bytes(1000, byteorder="big")
            print("sending", num)
            time.sleep(1)
            back = client.send(msg)
            bnum = int.from_bytes(back, byteorder="big")
            if num != bnum:
                print("ERROR", bnum)
                sys.exit(0)
            
        print("sent")
        client.close()
    else:
        print("Connection failed")

def test_server():
    print("server")
    server = Server()
    server.advertise_and_connect()
    while True:
        if server.messages_queued():
            print("Message!")
            msg = server.receive()
            server.send(msg)
        else:
            print("No message!")
    server.close()
