import queue
import gui
import time
import json
import os
from command import Command
from communication import *
from client import client
from datetime import datetime


def send_command(command, socket, guit):
    """
        Transmits 'TRANSMIT' followed by the respective function
        for a command, then waits for acknowledge.
        command" are the strings from the gui thread.
    """
    if len(command) > 3 and command[:4] == "key_": #Fulhack that will save us many rows.
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

def main():
    # Make sure to start a server before starting the gui.
    robot = client()
    robot.start(ip="130.236.226.142")

    map = [[1,0,1],[0,1,1]]

    # Send messages back and forth from the GUI using a Queue
    command_queue = queue.Queue()
    response_queue = queue.Queue()

    # Start our GUI thread
    guit = gui.gui_thread(command_queue, response_queue)
    guit.start()

    # Prevent key press from spamming on linux
    os.system("xset r off")
    
    #Timestamps to update sensor/motor values every diff_time_trigger seconds.
    current_time = datetime.now()
    last_time = current_time
    diff_time_trigger = 0.5 #Trigger every 1.0s
    
    time.sleep(0.2)
    # Use an infinite loop to check for commands from the GUI
    while True:
        current_time = datetime.now()
        diff = (current_time - last_time).total_seconds()
        if(diff >= diff_time_trigger): 
            #Update sensor and motor values if diff_time_trigger seconds has passed since last update
            last_time = current_time
            send_command("get_motor_data", robot.client, guit)
            send_command("get_sensor_data", robot.client, guit)
        # TODO: Update if map changed
        #guit.draw_map(map)
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
        # Not yet used
        if not response_queue.empty():
            # There is a command from the raspberry to the GUI. Lets get it
            command = response_queue.get()
            guit.receive_command(command)
        time.sleep(0.01)
    # Reset settings
    os.system("xset r on")
    
    print("exiting")

if __name__ == "__main__":
    main()
