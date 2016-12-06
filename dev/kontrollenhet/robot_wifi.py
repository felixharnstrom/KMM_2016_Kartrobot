import server
from command import *
from communication import *
import threading
import queue
import mode

#output_queue = queue.Queue() #What should we send to the pc

def wifi_main(motor_data, sensor_data, input_queue : queue.Queue):
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
        data = s.client.recv(4096).decode("utf-8")
        print ("Data received from PC: ", data)
        # Acknowledge client
        s.client.sendall("ACK".encode())
        if (data == "TRANSMIT"):
            #Receive Command
            command = receive_command(s.client)
            input_queue.put(("COMMAND", command))
        elif (data == "FORWARD_MOTOR_INFO"):
            #Transmit motor data
            motor_data_cpy = motor_data.copy()
            s.client.sendall(json.dumps(motor_data_cpy).encode())
        elif (data = "FORWARD_SENSOR_INFO"):
            #Transmit sensor data
            sensor_data_cpy = sensor_data.copy()
            s.client.sendall(json.dumps(sensor_data_cpy).encode())
        elif (data == "KEY_EVENT"):
            #Receive keyevent
            key_event = s.client.recv(4096).decode("utf-8")
            if(mode.get_mode() == mode.ControlModeEnums.MANUAL): #If we are not in manual mode, discard the key_event.
                #TODO: Notify client of this action? Also, stop the the robot from moving.
                input_queue.put(("KEY_EVENT",key_event))
        elif (data == "TOGGLE_MODE"):
            #Change the mode
            new_mode = s.client.recv(4096).decode("utf-8")
            if (new_mode == "manual"):
                print("Manual mode set")
                mode.set_mode(mode.ControlModeEnums.MANUAL)
            elif (new_mode == "autonomous"):
                input_queue.put(Command.stop_motors())
                print("Autonomous mode set")
                mode.set_mode(mode.ControlModeEnums.AUTONOMOUS)
                