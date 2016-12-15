from robot_communication import *
from command import Command
from datetime import datetime
import gpio_buttons as gpio
import mode

def autonomous_step():
    """Perform the next action decided by the autonomous mode."""
    # TODO
    pass

def main():
    """Main loop"""
    # Init
    init_UARTs()
    gpio.launch_poll_threads()
    init_wifi_thread()
        
    current_time = datetime.now()     
    last_time = current_time     
    diff_time_trigger = 0.3 #Trigger every 0.3s
    # Loop
    while True:
        # Process messages
        process_actions()
    
        current_time = datetime.now()         
        diff = (current_time - last_time).total_seconds()         
        if (diff >= diff_time_trigger): 
            last_time = current_time
            handle_command(Command.controller_information())
            handle_command(Command.read_gyro())
            handle_command(Command.read_lidar())
            handle_command(Command.read_front_ir())
            handle_command(Command.read_left_front_ir())
            handle_command(Command.read_left_back_ir())
            handle_command(Command.read_right_front_ir())
            handle_command(Command.read_right_back_ir())
            handle_command(Command.read_reflex_left())
            handle_command(Command.read_reflex_right())
            print("Time took:", (datetime.now()-last_time).total_seconds())
            # Update motor and sensor values
            
        # Autonomous step
        if mode.get_mode() == mode.ControlModeEnums.AUTONOMOUS:
            autonomous_step()
            
    close_UARTs()
    

if __name__ == "__main__":
    main()
