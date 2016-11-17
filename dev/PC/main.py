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
    for a command, then waits for acknowledge."""
    
    if command == "forward":
        socket.sendall("TRANSMIT".encode())
        transmit_command(Command.drive(0, 50, 0), socket)
    elif command == "back":
        socket.sendall("TRANSMIT".encode())
        transmit_command(Command.drive(1, 50, 0), socket)
    elif command == "left":
        socket.sendall("TRANSMIT".encode())
        transmit_command(Command.turn(0, 50, 0), socket)
    elif command == "right":
        socket.sendall("TRANSMIT".encode())
        transmit_command(Command.turn(1, 50, 0), socket)
    elif command == "stop_motors":
        socket.sendall("TRANSMIT".encode())
        transmit_command(Command.stop_motors(), socket)
    elif command == "get_diagnostics":
        socket.sendall("GET_MOTOR_DIAG".encode())
        ack = socket.recv(4096)
        responsef = receive_command(socket)
        response = responsef.ARGUMENTS
        print(responsef.TYPE)
        print(response)
        left_dir = response[0]
        left_pwm = response[1]
        right_dir = response[2]
        right_pwm = response[3]
        servo_pvm = response[4]*2**8 + response[5]
        guit.receive_command(["set_motors", left_pwm, right_pwm])
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
