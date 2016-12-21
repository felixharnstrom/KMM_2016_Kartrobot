from command import *
import socket
import json

"""The socket input buffer. Messages are stored, and read by using '\n' as delimiter."""
_input_buffer = bytearray()

def send_data(socket: socket, msg : str):
    """
    Simple wrapper for socket.sendall with a delimiter.
    
    Args:
        :param socket (socket): Socket to send on.
        :param msg (str): The message to send.
    
    """
    socket.sendall((msg+"\n").encode())

def construct_msg(command : Command):
    """
    Construct a JSON message with the given command to transmit over WIFI.

    Args:
        :param command (Command): The command to construct a message from.

    Returns:
        :return (str): String respresenting a JSON message containing the command address, type, and arguments.
    """
    return json.dumps([command.address, command.command_type, command.arguments])

def _retrieve_message_from_buffer():
    """
    Retrieves a message from the _input_buffer if there is one waiting,
    otherwise return empty.

    Returns:
        :return (str): The received message without delimiter, if there was a message.
    """
    global _input_buffer
    delimiter_index = 0
    #Do we already have a message in the buffer?
    try:
        delimiter_index = _input_buffer.index(b'\n')
    except ValueError:
        #No message to be found
        return ""
    msg = _input_buffer[:delimiter_index]
    _input_buffer = _input_buffer[delimiter_index+1:]
    return msg.decode()
    
def receive_data(socket : socket):
    """
    Receives the next data from the buffer,
    if there is no message - retrieve it from the socket.
    
    Args:
        :param socket (socket): Socket to read data from.
    """
    global _input_buffer
    msg = _retrieve_message_from_buffer()
    if msg:
        return msg
    #If a message isn't in buffer, we will retrieve it.
    while True:
        data = socket.recv(4096)
        _input_buffer.extend(data)
        msg = _retrieve_message_from_buffer()
        if msg:
            return msg
    
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
    send_data(socket, json_msg)

def receive_command(socket : socket):
    """
    Receives a command from the given socket.

    Args:
        :param socket (socket): Socket to receive command from.
    """
    data = receive_data(socket)
    json_msg = json.loads(data)
    return construct_command(json_msg)
    
