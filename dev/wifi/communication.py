from command import *
import socket
import json

def construct_msg(command : Command):
    return json.dumps([command.address, command.command_type, command.arguments])

def construct_command(msg : list):
    address = msg[0]
    msg_type = msg[1]
    args = msg[2]
    return get_executable_command(msg_type + (address * 16), args)

def transmit_command(command : Command, socket : socket):
    json_msg = construct_msg(command)
    socket.sendall(json_msg.encode())

def receive_command(socket : socket):
    data = socket.recv(4096).decode("utf-8")
    json_msg = json.loads(data)
    return construct_command(json_msg)
