
from server import *
from client import *
import time, sys
from random import randrange

# A test for the current functionality

def test_client_ip():
    print("client")
    client = IPClient("localhost")
    if client.connect():
        while True:
            num = randrange(0, 65535)
            msg = num.to_bytes(1000, byteorder="big")
            print("sending", num)
            back = client.send(msg)
            bnum = int.from_bytes(back, byteorder="big")
            if num != bnum:
                print("ERROR", bnum)
                sys.exit(0)
            
        print("sent")
        client.close()
    else:
        print("Connection failed")

def test_server_ip():
    print("server")
    server = IPServer()
    server.advertise_and_connect()
    while True:
        msg = server.receive()
        #time.sleep(0.1)
        server.send(msg)
    server.close()
