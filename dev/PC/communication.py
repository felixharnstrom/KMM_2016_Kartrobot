from modules import *
from function import *
import socket
import json

def construct_msg(function : Function):
    return json.dumps([function.ADRESS, function.TYPE, function.ARGUMENTS])

def construct_function(msg : list):
    address = msg[0]
    msg_type = msg[1]
    args = msg[2]
    return GetExecutableFunction((msg_type + address*16), args)

def transmit_function(function : Function, socket : socket):
    json_msg = construct_msg(function)
    socket.sendall(json_msg.encode())

def receive_function(socket : socket):
    data = socket.recv(4096).decode("utf-8")
    json_msg = json.loads(data)
    return construct_function(json_msg)