import server
from command import *
from communication import *
from grid_map import *
import threading
import queue
import mode

#output_queue = queue.Queue() #What should we send to the pc

def wifi_main(motor_data, sensor_data, map_lock, grid_map, input_queue : queue.Queue):
    """
    Continously retrieves messages from and transmits messages to the PC. Messages from the PC are interpreted and handled depending on their meta-message.
    This functions prime purpose is to make the PC able to communicate with the robot, and messages may therefor be used to affect its function.
    
    Args:
        :param motor_data (dictionary): A shared access to the motor_data that is updated elsewhere.
        :param sensor_data (dictionary): A shared access to the sensor_data that is updated elsewhere.
        :param input_queue (Queue): A shared access to a thread-safe Queue which can be used to enqueue actions for the robot to execute.
    """
    s = server.server()
    s.start()
    s.connect()

    while 1:
        # The messages are made with json which appends extra "" - cut them off
        data = receive_data(s.client)
        print ("Data received from PC: ", data)
        # Acknowledge client
        send_data(s.client, "ACK")
        if (data == "TRANSMIT"):
            #Receive Command
            command = receive_command(s.client)
            input_queue.put(("COMMAND", command))
        elif (data == "FORWARD_MOTOR_INFO"):
            #Transmit motor data
            send_data(s.client, json.dumps(motor_data))
        elif (data == "FORWARD_SENSOR_INFO"):
            #Transmit sensor data
            send_data(s.client, json.dumps(sensor_data))
        elif (data == "KEY_EVENT"):
            #Receive keyevent
            key_event = receive_data(s.client)
            if(mode.get_mode() == mode.ControlModeEnums.MANUAL): #If we are not in manual mode, discard the key_event.
                #TODO: Notify client of this action? Also, stop the the robot from moving.
                input_queue.put(("KEY_EVENT",key_event))
        elif (data == "SEND_MAP"):
            map_lock.acquire()
            send_data(s.client, grid_map.gui_drawable())
            map_lock.release()
        elif (data == "TOGGLE_MODE"):
            #Change the mode
            new_mode = receive_data(s.client)
            if (new_mode == "manual"):
                print("Manual mode set")
                mode.set_mode(mode.ControlModeEnums.MANUAL)
            elif (new_mode == "autonomous"):
                input_queue.put(("RESET_KEYS_PRESSED",))
                print("Autonomous mode set")
                mode.set_mode(mode.ControlModeEnums.AUTONOMOUS)
                
