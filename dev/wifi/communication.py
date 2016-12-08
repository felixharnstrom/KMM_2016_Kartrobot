from command import *
import socket
import json

def construct_msg(command : Command):
    """
    Construct a JSON message with the given command to transmit over WIFI.

    Args:
        :param command (Command): The command to construct a message from.

    Returns:
        :return (str): String respresenting a JSON message containing the command address, type, and arguments.
    """
    return json.dumps([command.address, command.command_type, command.arguments])+"\n"

def receive_data(socket : socket):
    """
    Receives the next data delimited by '\n' from the given socket.
    
    Args:
        :param socket (socket): Socket to read data from.
    """
    data_list = []
    while True:
        c = socket.recv(1)
        if c == "\n" or c == "":
            break
        data_list.append(c)
    return ''.join(data_list)
    
def construct_command(msg : list):
    """
    Construct a command from the given message.

    Args:
        :param msg (list): List containing the message address, type, and list of args (in that order).

    Returns:
        :return (Command): Command corresponding to the given message.
    """
    address = msg[0]
    msg_type = msg[1]
    args = msg[2]
    return get_executable_command(msg_type + (address * 16), args)

def transmit_command(command : Command, socket : socket):
    """
    Encodes the given command and transmits it over the given socket.

    Args:
        :param command  (Command):  The command to transmit.
        :param socket   (socket):   Socket to use when transmitting.
    """
    json_msg = construct_msg(command)
    socket.sendall(json_msg.encode())

def receive_command(socket : socket):
    """
    Receives a command from the given socket.

    Args:
        :param socket (socket): Socket to receive command from.
    """
    data = receive_data(socket)
    json_msg = json.loads(data)
    return construct_command(json_msg)
    