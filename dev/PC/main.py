import queue
import gui
import time
import json
import os
import styrenhet_functions
from communication import *
from client import client


def transmit_command(command, socket, guit):
    """Transmits '"TRANSMIT"' followed by the respective function
    for a command, then waits for acknowledge."""
    socket.sendall(json.dumps("TRANSMIT").encode())
    if command == "forward":
        transmit_function(styrenhet_functions.Drive(0, 50, 0, 0), socket)
        guit.receive_command(["set_motors", 50, 50])
    elif command == "left":
        transmit_function(styrenhet_functions.Turn(0, 50, 0, 0), socket)
        guit.receive_command(["set_motors", 50, 50])
    elif command == "right":
        transmit_function(styrenhet_functions.Turn(1, 50, 0, 0), socket)
        guit.receive_command(["set_motors", 50, 50])
    elif command == "stop_motors":
        transmit_function(styrenhet_functions.StopMotors(), socket)
        guit.receive_command(["set_motors", 0, 0])
    ack = socket.recv(4096)

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

    # Prevent key press from spamming
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
                gui.gui.quit()
                # Close socket
                robot.close()
                # Exit the while loop
                break
            else:
                # All commands that are not used above are sent to the raspberry server.
                transmit_command(command, robot.client, guit)


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
