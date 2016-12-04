import threading
from enum import Enum

class ControlModeEnums(Enum):
    """
    Contains the enums for the different modes that we can be in.
    """
    AUTONOMOUS = 0
    MANUAL = 1

_mode = ControlModeEnums.MANUAL     #The current mode which we are in
_mode_lock = threading.Lock()       #Lock used for reading/editing our current mode
    
def set_mode(new_mode : ControlModeEnums):
    """
    Sets the current mode to the given mode.
    
    Args:
        :param new_mode (ControlModeEnums): The new mode to set
    """
    global _mode
    _mode_lock.acquire()
    _mode = new_mode
    _mode_lock.release()

def get_mode():
    """
    Returns the current mode.
    
    Returns:
        :return (ControlModeEnums): The mode which we are currently in.
    """
    _mode_lock.acquire()
    mode_enum = _mode
    _mode_lock.release()
    return mode_enum

def toggle_mode():
    """
    Toggles the current mode between AUTONOMOUS and MANUAL.
    """
    global _mode
    _mode_lock.acquire()
    if(_mode == ControlModeEnums.AUTONOMOUS):
        _mode = ControlModeEnums.MANUAL
    else:
        _mode = ControlModeEnums.AUTONOMOUS
    _mode_lock.release()
