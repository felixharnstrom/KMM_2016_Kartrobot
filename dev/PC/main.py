import queue
import gui
import time
import json
import os
from command import Command
from communication import *
from client import client


def send_command(command, socket, guit):
    """Transmits 'TRANSMIT' followed by the respective function
    for a command, then waits for acknowledge.

    "command" are the strings from the gui thread."""
    '''
    if command == "forward":
        socket.sendall("TRANSMIT".encode())
        ack = socket.recv(4096)
        transmit_command(Command.drive(0, 50, 0), socket)
    elif command == "back":
        socket.sendall("TRANSMIT".encode())
        ack = socket.recv(4096)
        transmit_command(Command.drive(1, 50, 0), socket)
    elif command == "left":
        socket.sendall("TRANSMIT".encode())
        ack = socket.recv(4096)
        transmit_command(Command.turn(0, 50, 0), socket)
    elif command == "right":
        socket.sendall("TRANSMIT".encode())
        ack = socket.recv(4096)
        transmit_command(Command.turn(1, 50, 0), socket)
    elif command == "stop_motors":
        socket.sendall("TRANSMIT".encode())
        ack = socket.recv(4096)
        transmit_command(Command.stop_motors(), socket)
    '''
    if len(command) > 3 and command[:4] == "key_": #Fulhack that will save us many rows.
        socket.sendall("KEY_EVENT".encode())
        ack = socket.recv(4096)
        socket.sendall(command[4:].encode())
    elif len(command) > 4 and command [:5] == "mode_":
        socket.sendall("TOGGLE_MODE".encode())
        ack = socket.recv(4096)
        socket.sendall(command[5:].encode())
    elif command == "get_diagnostics":
        socket.sendall("FORWARD_CTRL_INFO".encode())
        ack = socket.recv(4096)
        motor_data = json.loads(socket.recv(4096).decode("utf-8"))
        dir_mod_left = 1 if motor_data["LEFT_SIDE_DIRECTION"] else -1
        dir_mod_right = 1 if motor_data["RIGHT_SIDE_DIRECTION"] else -1
        speed_left = motor_data["LEFT_SIDE_SPEED"]*dir_mod_left
        speed_right = motor_data["RIGHT_SIDE_SPEED"]*dir_mod_right
        guit.receive_command(["set_motors", speed_left, speed_right])
        guit.receive_command(["set_servo", motor_data["SERVO_ANGLE"]])
        


def main():
    # Make sure to start a server before starting the gui.
    robot = client()
    robot.start(ip="localhost")

    map = [[1,0,1],[0,1,1]]

    # Send messages back and forth from the GUI using a Queue
    command_queue = queue.Queue()
    response_queue = queue.Queue()

    # Start our GUI thread
    guit = gui.gui_thread(command_queue, response_queue)
    guit.start()

    # Prevent key press from spamming on linux
    os.system("xset r off")
    
    time.sleep(0.2)
    # Use an infinite loop to check for commands from the GUI
    while True:
        guit.draw_map(map)
        if not command_queue.empty():
            # There is a command. Lets get it.
            command = command_queue.get()
            print("transmitting", command)

            if command == "quit":
                # Quit tkinter
                guit.gui.quit()
                # Close socket
                robot.close()
                # Exit the while loop
                break
            else:
                # All commands that are not used above are sent to the raspberry server.
                send_command(command, robot.client, guit)
                # TODO: Send periodically instead?
                # This works but might be spammy
                #send_command("get_diagnostics", robot.client, guit)


        # Not yet used
        if not response_queue.empty():
            # There is a command from the raspberry to the GUI. Lets get it
            command = response_queue.get()
            guit.receive_command(command)

    # Reset settings
    os.system("xset r on")
    
    print("exiting")

if __name__ == "__main__":
    main()
