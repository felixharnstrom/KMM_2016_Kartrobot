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
    mode_toggle = False # Changes to true whenever change mode is pressed

    # Init
    init_UARTs()
    gpio.init()
    gpio.poll_shutdown_in_other_thread()
    init_wifi_thread()
    
    
    current_time = datetime.now()     
    last_time = current_time     
    diff_time_trigger = 0.5 #Trigger every 0.4s (To not make the)
    # Loop
    while True:
        # Process messages
        process_actions()
    
        current_time = datetime.now()         
        diff = (current_time - last_time).total_seconds()         
        if (diff >= diff_time_trigger): 
            last_time = current_time
            handle_command(Command.controller_information())
            # Update motor diagnostics values


        # Autonomous step
        if mode.get_mode() == mode.ControlModeEnums.AUTONOMOUS:
            autonomous_step()

        # Check mode switch logic
        if gpio.change_mode_is_pressed():
            if not mode_toggle:
                mode.toggle_mode()
            mode_toggle = True
        else:
            mode_toggle = False
            
    close_UARTs()
    

if __name__ == "__main__":
    main()
