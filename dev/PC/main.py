import queue
import gui
import time
import json
from client import client

# Make sure to start a server before starting the gui.
robot = client()
robot.start(ip="localhost")

map = [[1,0,1],[0,1,1]]

# Send messages back and forth from the GUI using a Queue
command_queue = queue.Queue()
response_queue = queue.Queue()

# Start our GUI thread
gui = gui.gui_thread(command_queue, response_queue)
gui.start()
time.sleep(0.2)
# Use an infinite loop to check for commands from the GUI
while True:
    gui.draw_map(map)
    if not command_queue.empty():
        # There is a command. Lets get it.
        command = command_queue.get()
        print (command)

        if command == "quit":
            # Quit tkinter
            gui.gui.quit()
            # Close socket
            robot.close()
            # Exit the while loop
            break
        else:
            # All commands that are not used above are sent to the raspberry server.
            gui.receive_command(["set_motors", 10, 10])
            print ("sending",command)
            robot.client.sendall(json.dumps(command).encode())

            pass

    # Not yet used
    if not response_queue.empty():
        # There is a command from the raspberry to the GUI. Lets get it
        command = response_queue.get()
        gui.receive_command(command)

print("exiting")