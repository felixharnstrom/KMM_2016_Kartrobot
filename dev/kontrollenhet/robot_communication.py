import server
from command import *
from communication import *
from UART import UART
import time
import threading
import robot_wifi
import queue
import mode

#Contains all data for motors, servo and sensors
motor_data = {"LEFT_SIDE_DIRECTION":0, "LEFT_SIDE_SPEED":0 ,"RIGHT_SIDE_DIRECTION":0, "RIGHT_SIDE_SPEED":0, "SERVO_ANGLE":0}
motor_data_lock = threading.Lock()
sensor_data = {"IR_LEFT_FRONT":0, "IR_LEFT_BACK":0 ,"IR_RIGHT_FRONT":0, "IR_RIGHT_BACK":0, "IR_BACK":0, "IR_LIDAR":0, "GYRO":0}
sensor_data_lock = threading.Lock()
key_pressed = {"right":False, "left":False, "up":False, "down":False}
input_queue = queue.Queue() #Execute on robot


#TODO: We need to grep and get the two serial interaces (uarts), as well as deciding which is which (by sending an echo for example)
#This method will assign the correct UART object to UART_motor and UART_sensor for pain-free execution
UART_sensor = UART("ttyUSB1") #TODO: This is not always true!
UART_motor = UART("ttyUSB0") #TODO: This is not always true!
#key_pressed = {"right":False, "left":False, "up":False, "down":False}

def init_UARTs():
    #Get serial com. names from system
    #Send dummy messages to find out which com port is assigned to which atmega
    #Set UART_motor and UART_sensor to the correct objects.
    UART_sensor = UART("ttyUSB1") #TODO: This is not always true!
    UART_motor = UART("ttyUSB0") #TODO: This is not always true!
    return

def close_UARTs():
    UART_motor.close()
    UART_sensor.close()
    s.close()

def init_wifi_thread():
    threading.Thread(target=robot_wifi.wifi_main,args=(motor_data,sensor_data,motor_data_lock,sensor_data_lock,input_queue)).start()

def adjust_speeds():
    c = Command.stop_motors() #Dummy
    if key_pressed["up"]:
        if not key_pressed["left"] and not key_pressed["right"] and not key_pressed["down"]: #Forward
            c = Command.drive(1,50,0)
        elif key_pressed["left"] and not key_pressed["right"]:
            c = Command.side_speeds(1,25,1,75)       
        elif key_pressed["right"] and not key_pressed["left"]:
            c = Command.side_speeds(1,75,1,25)
        else:
            c = Command.stop_motors() #Stop pressing like stupid
    elif key_pressed["down"]:
        if not key_pressed["left"] and not key_pressed["right"]: #Backward
            c = Command.drive(0,50,0)
        elif key_pressed["left"] and not key_pressed["right"]:
            c = Command.side_speeds(0,75,0,25)       
        elif key_pressed["right"] and not key_pressed["left"]:
            c = Command.side_speeds(0,25,0,75)
        else:
            c = Command.stop_motors() #Stop pressing like stupid
    elif key_pressed["left"] and not key_pressed["right"]: #Left
        c = Command.turn(0,50,0)
    elif key_pressed["right"] and not key_pressed["left"]: #Right
        c = Command.turn(1,50,0)
    else:
        c = Command.stop_motors() #Stop pressing like stupid
    print("sendin")
    handle_command(c)

def handle_key(key_event : str):
    key = key_event[:-2] #All but the last 2 chars ex. "right" in "right_p"
    key_e = key_event[-1:] #Only the last char ex. "p" in "right_p"
    if key in key_pressed.keys() and key_e in ["p", "r"]:
        key_pressed[key] = (key_e == "p")
        adjust_speeds()
    
def send_sensor_ack():
    ack = Command.ack() #Create ack command
    UART_sensor.send_command(ack) #Send it over uart

def send_motor_ack():
    ack = Command.ack() #Create ack command
    UART_motor.send_command(ack) #Send it over uart

    
#If someone finds a better way, please change this.
def get_sensor_dict_key(c_enum : CommandEnums):
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
    elif(c_enum == CommandEnums.READ_IR_LIDAR):
        return "IR_LIDAR"
    elif(c_enum == CommandEnums.READ_GYRO):
        return "GYRO"
    else:
        return False
        
#Reads IR-sensor and sets corresponding sensor_data key.
def read_ir_sensor(command : Command):
    c_enum = command.get_enum()
    UART_sensor.send_command(command)
    #ack = UART_sensor.receive_packet() #Receive ack (TODO: implement response check/resend if we implement timeout on receive)
    (ir_value_msb, ir_value_lsb) = UART_sensor.receive_payload()
    #send_sensor_ack()
    ir_value = ir_value_msb*(2**8)+ir_value_lsb
    sensor_data[get_sensor_dict_key(c_enum)] = ir_value
    return
    
def read_gyro_sensor(command : Command):
    UART_sensor.send_command(command)
    #ack = UART_sensor.receive_packet() #Receive ack (TODO: implement response check/resend if we implement timeout on receive)
    (gyro_value_msb, gyro_value_lsb) = UART_sensor.receive_payload()
    #send_sensor_ack()
    #We need to cast it to signed int 16 bit (due to being a gyro)
    gyro_value = gyro_value_msb*(2**8)+gyro_value_lsb
    if(gyro_value > 0x7fff): #unsigned 16-bit -> signed 16-bit 
        gyro_value -= 65536 #0x7000
    sensor_data["GYRO"] = gyro_value
    return   

#Send command to controller to retrieve and return motor data
def get_motor_diagnostics(command : Command):
    pwm_to_speed = 2.5 #Constant for getting motor pwm -> motor speed percentage
    print(command)
    print(UART)
    UART_motor.send_command(command)
    #ack = UART_motor.receive_packet() #Receive ack
    #Sorry for unreadable code!
    (left_direction, left_pwm, right_direction, right_pwm, servo_pwm_msb, servo_pwm_lsb) = UART_motor.receive_payload()
    send_motor_ack()
    servo_angle = ((servo_pwm_msb*(2**8)+servo_pwm_lsb)-708)/8.45 #Formula for translating servo pwm to servo angle
    print("LEFT: ", 
          "forward " if left_direction else "backward ",  left_pwm/pwm_to_speed, "%\n",
          "RIGHT: ",
          "forward " if right_direction else "backward ", right_pwm/pwm_to_speed, "%\n",
          "SERVO: ", servo_angle, " degrees\n")
    motor_data["LEFT_SIDE_DIRECTION"] = left_direction
    motor_data["LEFT_SIDE_SPEED"] = left_pwm/pwm_to_speed
    motor_data["RIGHT_SIDE_DIRECTION"] = right_direction
    motor_data["RIGHT_SIDE_SPEED"] = right_pwm/pwm_to_speed
    motor_data["SERVO_ANGLE"] = servo_angle
    return

#We can handle pc -> rpi -> pc in shell methods (By adding command instructions for them but utilizing c_enums > 31)
def handle_command(command : Command):
    c_enum = command.get_enum()
    print("Getting enum", command.command_type)
    if(c_enum == CommandEnums.CONTROLLER_INFORMATION):
        get_motor_diagnostics(command)
    elif(c_enum == CommandEnums.READ_IR_LEFT_FRONT or 
            c_enum == CommandEnums.READ_IR_LEFT_BACK or
            c_enum == CommandEnums.READ_IR_RIGHT_FRONT or
            c_enum == CommandEnums.READ_IR_RIGHT_BACK or
            c_enum == CommandEnums.READ_IR_BACK or
            c_enum == CommandEnums.READ_LIDAR):
        read_ir_sensor(command)      
    elif(c_enum == CommandEnums.READ_GYRO):
        read_gyro_sensor(command)
    else:
        #Temporary solution for all non-special commands (eg. just send)
        #Does >not< handle incorrectly parsed commands
        if (command.address == 1):
            print("Sensor command")
            UART_sensor.send_command(command)
            #ack = UART_sensor.receive_packet() #Receive ack
        else:
            print("Motor command")
            UART_motor.send_command(command)
            #ack = UART_motor.receive_packet() #Receive ack
    return

def process_action():
    """Process the next action on the input queue.

    Returns True if an action was popped from the queue, false if not."""
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
        return True
    return False

def process_actions():
    """Process all actions on the input queue"""
    while process_action():
        pass


            
#while 1:
#    process_action()
        
