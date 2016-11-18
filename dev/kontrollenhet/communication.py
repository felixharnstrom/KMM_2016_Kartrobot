from command import *
import socket
import json

def constructMsg(command : Command):
    return json.dumps([command.address, command.command_type, command.arguments])

def constructCommand(msg : list):
    address = msg[0]
    msg_type = msg[1]
    args = msg[2]
    return getExecutableCommand(msg_type + (address * 16), args)

def transmitCommand(command : Command, socket : socket):
    json_msg = constructMsg(command)
    socket.sendall(json_msg.encode())

def receiveCommand(socket : socket):
    data = socket.recv(4096).decode("utf-8")
    json_msg = json.loads(data)
    return constructCommand(json_msg)