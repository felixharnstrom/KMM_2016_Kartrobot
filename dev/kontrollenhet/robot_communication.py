from command import *
from communication import *
from UART import UART
import threading
import robot_wifi
import queue
from timeout import *

UART_sensor = None          #The uninitiated UART object for sensor communication.
UART_motor = None           #The uninitiated UART object for motor communication.
input_queue = queue.Queue() #The Queue object which contains all the Commands enqueued from the wifi communication thread.
key_pressed = {"right":False, "left":False,
                "up":False, "down":False} #Contains the current pressed_down state for the different directions from the GUI.
motor_data = {"LEFT_SIDE_DIRECTION":0, "LEFT_SIDE_SPEED":0, 
                "RIGHT_SIDE_DIRECTION":0, "RIGHT_SIDE_SPEED":0, 
                "SERVO_ANGLE":0} #Contains the last retrieved motor data from get_motor_diagnostics.
sensor_data = {"IR_LEFT_FRONT":0, "IR_LEFT_BACK":0 ,
                "IR_RIGHT_FRONT":0, "IR_RIGHT_BACK":0, 
                "IR_BACK":0, "LIDAR":0, 
                "GYRO":0} #Contains the last retrieved sensor data for each type of sensor Command.


def init_UARTs(sensor = "", motor = ""):
    """
    Initiates the sensor and motor UART objects with the given serial communication port names.
    
    Args:
        :param sensor (str): The name of the serial communication port used for the sensor UART.
        :param motor (str): The name of the serial communication port used for the motor UART.
    """
    global UART_sensor
    global UART_motor
    if sensor == "":
        serials = get_serials()
        UART_sensor = UART(serials["sensor"])
        UART_motor = UART(serials["control"])
    else:
        UART_sensor = UART(motor)
        UART_motor = UART(sensor)
    return

def close_UARTs():
    """
    Closes the connection for the motor and sensor UART objects.
    """
    UART_motor.close()
    UART_sensor.close()
    return

def init_wifi_thread():
    """
    Initiates and executes a new thread which receives and transmits commands over wifi.
    """
    threading.Thread(target=robot_wifi.wifi_main,args=(motor_data,sensor_data,input_queue)).start()

def adjust_speeds():
    """
    Sets the speed and direction of the robot motors during manual mode depending on
    what keys that are currently pressed down in keys_pressed, retrieved from the PC side.
    """
    command = Command.stop_motors() #Set to stop_motors as that is what we want to do if we get any 'illegal' combination of keys or if no keys are pressed
    
    if key_pressed["up"]:
        if not key_pressed["left"] and not key_pressed["right"] and not key_pressed["down"]: #Forward
            command = Command.drive(1,50,0)
        elif key_pressed["left"] and not key_pressed["right"]: #Forward-left
            command = Command.side_speeds(1,25,1,75)       
        elif key_pressed["right"] and not key_pressed["left"]: #Forward-right
            command = Command.side_speeds(1,75,1,25)
    elif key_pressed["down"]:
        if not key_pressed["left"] and not key_pressed["right"]: #Backward
            command = Command.drive(0,50,0)
        elif key_pressed["left"] and not key_pressed["right"]: #Backward-right
            command = Command.side_speeds(0,75,0,25)       
        elif key_pressed["right"] and not key_pressed["left"]: #Backward-left
            command = Command.side_speeds(0,25,0,75)
    elif key_pressed["left"] and not key_pressed["right"]: #Left
        command = Command.turn(0,50,0)
    elif key_pressed["right"] and not key_pressed["left"]: #Right
        command = Command.turn(1,50,0)
    handle_command(command) #Execute the command
    
    
def reset_keys_pressed():
    """
    Sets all keys in key_pressed to false and updates speed accordingly
    """
    for key in key_pressed:
        key_pressed[key] = False
    
def handle_key(key_event : str):
    """
    Interprets the key-event string and then updates the corresponding key in key_pressed to reflect the event, 
    after which it will adjust the speeds to the new key_pressed values.
    For example "right_p" would mean that the right key is set to pressed.
    
    Args:
        :param key_event (str): The key-event to interpret, eg. "right_p" or "left_r"
    """
    key = key_event[:-2]    #All but the last 2 chars eg. "right" in "right_p"
    key_e = key_event[-1:]  #Only the last char eg. "p" in "right_p"
    if key in key_pressed.keys() and key_e in ["p", "r"]: #Is the input legal?
        key_pressed[key] = (key_e == "p")
        adjust_speeds()
    
def send_sensor_ack():
    """
    Transmits an ack Command over the sensor UART.
    """
    ack = Command.ack()           #Create ack command
    UART_sensor.send_command(ack) #Send it over uart

def send_motor_ack():
    """
    Transmits an ack Command over the motor UART.
    """
    ack = Command.ack()          #Create ack command
    UART_motor.send_command(ack) #Send it over uart


def get_sensor_dict_key(c_enum : CommandEnums):
    """
    Retrieves the dictionary key for the sensor that data will be 
    fetched for by executing the Command with the corresponding enum.
    
    Args:
        :param c_enum (CommandEnums): The enum for the Command that we wish to sensor_data key from.
    
    Returns:
        :return (str): The key for the sensor_data dictionary value that we will fetch new data for.
    """
    if(c_enum == CommandEnums.READ_IR_LEFT_FRONT):
        return "IR_LEFT_FRONT"
    elif(c_enum == CommandEnums.READ_IR_LEFT_BACK):
        return "IR_LEFT_BACK"
    elif(c_enum == CommandEnums.READ_IR_RIGHT_FRONT):
        return "IR_RIGHT_FRONT"
    elif(c_enum == CommandEnums.READ_IR_RIGHT_BACK):
        return "IR_RIGHT_BACK"
    elif(c_enum == CommandEnums.READ_IR_BACK):
        return "IR_BACK"
    elif(c_enum == CommandEnums.READ_LIDAR):
        return "LIDAR"
    elif(c_enum == CommandEnums.READ_GYRO):
        return "GYRO"
    else:
        return ""
        
def read_ir_sensor(command : Command):
    """
    Transmits a command and receives data for an IR-sensor, using the sensor UART.
    The data that is received is put into the sensor_data dictionary for the sensor 
    that was intended by the executed Command.
    
    Args:
        :param command (Command): The IR-sensor command to execute and receive data for.
    """
    c_enum = command.get_enum()
    UART_sensor.send_command(command)
    ack = UART_sensor.receive_packet() #Receive ack (TODO: implement response check/resend if we implement timeout on receive)
    (ir_value_msb, ir_value_lsb) = UART_sensor.receive_payload()
    #send_sensor_ack()
    ir_value = ir_value_msb*(2**8)+ir_value_lsb
    sensor_data[get_sensor_dict_key(c_enum)] = ir_value
    return ir_value
    
def read_gyro_sensor(command : Command):
    """
    Transmits a read_gyro command and receives data for the gyro, using the sensor UART.
    The data that is received is put into the sensor_data dictionary under the "GYRO"-key.
    
    Args:
        :param command (Command): A gyro command.
    """
    UART_sensor.send_command(command)
    ack = UART_sensor.receive_packet() #Receive ack (TODO: implement response check/resend if we implement timeout on receive)
    (gyro_value_msb, gyro_value_lsb) = UART_sensor.receive_payload()
    #send_sensor_ack()
    gyro_value = gyro_value_msb*(2**8)+gyro_value_lsb
    if(gyro_value > 0x7fff): #unsigned 16-bit -> signed 16-bit 
        gyro_value -= 65536 #0x7000
    sensor_data["GYRO"] = gyro_value
    return gyro_value

def retrieve_and_update_motor_diagnostics(command : Command):
    """
    Transmits a controller_information command and receives motor data, using the motor UART.
    The data that is received is converted and put into the motor_data dictionary.
    
    Args:
        :param command (Command): A controller_information Command.
    """
    pwm_to_speed = 2.5 #Constant for getting motor pwm -> motor speed percentage
    UART_motor.send_command(command)
    #ack = UART_motor.receive_packet() #Receive ack
    (left_direction, left_pwm, 
    right_direction, right_pwm, 
    servo_pwm_msb, servo_pwm_lsb) = UART_motor.receive_payload()
    send_motor_ack()
    servo_angle = ((servo_pwm_msb*(2**8)+servo_pwm_lsb)-773)/8.72 #Formula for translating servo pwm to servo angle
    motor_data["LEFT_SIDE_DIRECTION"] = left_direction
    motor_data["LEFT_SIDE_SPEED"] = left_pwm/pwm_to_speed
    motor_data["RIGHT_SIDE_DIRECTION"] = right_direction
    motor_data["RIGHT_SIDE_SPEED"] = right_pwm/pwm_to_speed
    motor_data["SERVO_ANGLE"] = servo_angle
    #print(motor_data)
    return

def handle_command(command : Command):
    """
    Sends the command over the correct UART and handles conversion and storing of return data for those commands that need it.
    
    Args:
        :param command (Command): The command that we want to handle.
    """
    c_enum = command.get_enum()
    if(c_enum == CommandEnums.CONTROLLER_INFORMATION):
        retrieve_and_update_motor_diagnostics(command)
    elif(c_enum == CommandEnums.READ_IR_LEFT_FRONT or
            c_enum == CommandEnums.READ_IR_LEFT_BACK or
            c_enum == CommandEnums.READ_IR_RIGHT_FRONT or
            c_enum == CommandEnums.READ_IR_RIGHT_BACK or
            c_enum == CommandEnums.READ_IR_BACK or
            c_enum == CommandEnums.READ_LIDAR or
            c_enum == CommandEnums.READ_REFLEX_LEFT or
            c_enum == CommandEnums.READ_REFLEX_RIGHT):
        return read_ir_sensor(command)
    elif(c_enum == CommandEnums.READ_GYRO):
        return read_gyro_sensor(command)
    else:
        #Handles all commands that does not need any special handling (eg. just send and ack)
        #Does >not< handle incorrectly parsed commands
        if (command.address == 1):
            UART_sensor.send_command(command)
            #ack = UART_sensor.receive_packet() #Receive ack
        else:
            UART_motor.send_command(command)
            #ack = UART_motor.receive_packet() #Receive ack
    return

def process_action():
    """
    Processes the next action in the input queue.
    
    Returns:
        :return (bool): True if an action was popped from the queue, False if there was nothing to pop.
    """
    if input_queue.empty():
        return False
    next_action = None
    try:
        next_action = input_queue.get()
    except Exception:
        next_action = None
    #Then we have a command!
    if next_action != None:
        print(next_action)
        if next_action[0] == "COMMAND":
            handle_command(next_action[1])
        elif next_action[0] == "KEY_EVENT":
            print("Handling", next_action)
            handle_key(next_action[1])
        elif next_action[0] == "RESET_KEYS_PRESSED":
            reset_keys_pressed() 
        return True
    return False

def process_actions():
    """
    Process all actions in the input_queue until it's empty.
    """
    while process_action():
        pass


@timeout(1)
def try_reading_gyro():
    """
    Try to read a sensor with a uniqe msg_type.
    Sensorunit will return instantly, controlunit will timeout.
    """
    handle_command(Command.read_gyro())


def get_serials():
    serials = []
    # Read all USB serial ports.
    # In practice, this will never be more than two, and will not work with more
    for line in os.popen("ls -1 /dev/ttyUSB* | sed 's#.*/##'").read().split():
        serials.append(line.rstrip())

    init_UARTs(serials[0], serials[1])
    try:
        # If we get a valid response, the ports are correct.
        try_reading_gyro()
        return {"sensor": serials[0], "control": serials[1]}
    except TimeoutError:
        # If the request times out, the ports are reversed.
        init_UARTs(serials[1], serials[0])
        return {"sensor": serials[0], "control": serials[1]}
