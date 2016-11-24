import server
from command import *
from communication import *
import time
import threading
import queue
import mode

output_queue = queue.Queue() #What should we send to the pc

def wifi_main(motor_data, sensor_data, motor_data_lock : threading.Lock,
              sensor_data_lock : threading.Lock, input_queue : queue.Queue):
    
    s = server.server()
    s.start()
    s.connect()

    while 1:
        # The messages are made with json which appends extra "" - cut them off
        data = s.client.recv(4096).decode("utf-8")
        print ("Data received from PC: ", data)
        # Acknowledge client
        s.client.sendall("ACK".encode())
        #Different cases can be handled by passing them to a different handler (or by passing an additional parameter to the current handler)
        if (data == "TRANSMIT"):
            print ("Data")
            # Receive function
            command = receive_command(s.client)
            input_queue.put(("COMMAND", command))
        elif (data == "FORWARD_CTRL_INFO"):
            # Is this the intended way?
            print ("Forward")
            motor_data_lock.acquire()
            motor_data_cpy = motor_data.copy()
            motor_data_lock.release()
            s.client.sendall(json.dumps(motor_data_cpy).encode())
        elif (data == "KEY_EVENT"):
            # Receive keyevent
            print ("Keyevent")
            key_event = s.client.recv(4096).decode("utf-8")
            if(mode.get_mode() == mode.ControlModeEnums.MANUAL): #Otherwise discard manual command
                #TODO: Notify client of this action?
                print("Hej")
                input_queue.put(("KEY_EVENT",key_event))
        elif (data == "TOGGLE_MODE"):
            print ("Toggle")
            new_mode = s.client.recv(4096).decode("utf-8")
            if (new_mode == "MANUAL"):
                mode.set_mode(ControlModeEnums.MANUAL)
            elif (new_mode == "AUTO"):
                mode.set_mode(ControlModeEnums.AUTO)
            #TODO: Make this change the mode to new_mode (using mode.py)
            

            
