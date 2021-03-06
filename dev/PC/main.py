"""
Launches the mainloop which setups and connects to the robot using the given ip. Launches the GUI afterwards.
Makes sure that the GUI is correctly synchronized with data from the robot.

Args:
    :param argv (str): -i Robot server ip-adress to use for this GUI.
"""

import queue
import gui
import time
import json
import os
import sys
import argparse
from command import Command
from communication import *
from client import client
from datetime import datetime


def transmit_command(command, socket, guit):
    """
        Transmits command type, then waits for acknowledge.
        Then transmits and/or receives the payload for that command.
        
        Args:
            :param command  (str): The command-type string which we want to execute the transmit action for.
            :param socket   (socket): The socket which we wish to transmit the data over.
            :param guit     (gui_thread): The graphic thread object that we which to update with data received from the transmission. 
    """
    if command == "get_map_update":
        send_data(socket, "SEND_MAP")
        ack = receive_data(socket)
        robot_map_data = json.loads(receive_data(socket)) # [[robot_x, robot_y], [map..]
        guit.receive_command(["update_map", robot_map_data[0], robot_map_data[1]])
    elif command == "sync_mode":
        send_data(socket, "SYNC_MODE") #0 is autonomous, 1 is manual
        ack = receive_data(socket)
        current_mode_integer = receive_data(socket)
        guit.receive_command(["update_mode", current_mode_integer])     
    elif len(command) > 3 and command[:4] == "key_": #Fulhack that will save us many rows.
        send_data(socket, "KEY_EVENT")
        ack = receive_data(socket)
        send_data(socket, command[4:])
    elif len(command) > 4 and command [:5] == "mode_":
        send_data(socket, "TOGGLE_MODE")
        ack = receive_data(socket)
        send_data(socket, command[5:])
    elif command == "get_motor_data":
        send_data(socket, "FORWARD_MOTOR_INFO")
        ack = receive_data(socket)
        motor_data = json.loads(receive_data(socket))
        dir_mod_left = 1 if motor_data["LEFT_SIDE_DIRECTION"] else -1
        dir_mod_right = 1 if motor_data["RIGHT_SIDE_DIRECTION"] else -1
        speed_left = motor_data["LEFT_SIDE_SPEED"]*dir_mod_left
        speed_right = motor_data["RIGHT_SIDE_SPEED"]*dir_mod_right
        guit.receive_command(["set_motors", speed_left, speed_right])
        guit.receive_command(["set_servo", motor_data["SERVO_ANGLE"]])
    elif command == "get_sensor_data":
        send_data(socket, "FORWARD_SENSOR_INFO")
        ack = receive_data(socket)
        sensor_data = json.loads(receive_data(socket))
        guit.receive_command(["set_sensors", sensor_data])

def main(argv):
    # Parse args
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--ip", default = "localhost", help = "Server IP address. Default is localhost.")
    args = parser.parse_args()
    ip_address = args.ip

    # Make sure to start a server before starting the gui.
    robot = client()
    robot.start(ip=ip_address)

    # Send messages received from the GUI using this Queue.
    command_queue = queue.Queue()

    # Start our GUI thread
    guit = gui.gui_thread(command_queue)
    guit.start()

    # Prevent key press from spamming on linux
    os.system("xset r off")
    
    #Timestamps to update sensor/motor values every diff_time_trigger seconds.
    current_time = datetime.now()
    last_time_diag = current_time
    last_time_map = current_time
    diff_diag_trigger = 0.5 #Trigger every 0.5s
    diff_map_trigger = 1.0 #Trigger ever 1s
    
    time.sleep(0.2)
    # Use an infinite loop to check for commands from the GUI and transmitting them.
    while True:
        current_time = datetime.now()
        diff_diag = (current_time - last_time_diag).total_seconds()
        diff_map = (current_time - last_time_map).total_seconds()
        if(diff_diag >= diff_diag_trigger): 
            #Update sensor and motor values if diff_time_trigger seconds has passed since last update
            last_time_diag = current_time
            transmit_command("sync_mode", robot.client, guit)
            transmit_command("get_motor_data", robot.client, guit)
            transmit_command("get_sensor_data", robot.client, guit)
        if(diff_map >= diff_map_trigger):
            #Update map
            last_time_map = current_time
            transmit_command("get_map_update", robot.client, guit)   
        if not command_queue.empty():
            # There is a command. Lets get it.
            command = command_queue.get()
            print("Transmitting:", command)

            if command == "quit":
                # Quit tkinter
                guit.gui.quit()
                # Close socket
                robot.close()
                # Exit the while loop
                break
            else:
                # Transmit GUI command to the robot.
                transmit_command(command, robot.client, guit)
        time.sleep(0.01)
    # Reset settings
    os.system("xset r on")
    print("Exiting.")

if __name__ == "__main__":
    main(sys.argv[1:])
