from robot_communication import *
from command import Command
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
    
    # Loop
    while True:
        # Process messages
        process_actions()
        
        # Update motor diagnostics values
        handle_command(Command.controller_information())

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
