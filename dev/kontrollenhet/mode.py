import threading
from enum import Enum

class ControlModeEnums(Enum):
    AUTONOMOUS = 0
    MANUAL = 1

_mode = ControlModeEnums.MANUAL
_mode_lock = threading.Lock()
    
def set_mode(new_mode : ControlModeEnums):
    global _mode
    _mode_lock.acquire()
    _mode = new_mode
    _mode_lock.release()

def get_mode():
    _mode_lock.acquire()
    mode_enum = _mode
    _mode_lock.release()
    return mode_enum
    
 def toggle_mode():
    global _mode
    _mode_lock.acquire()
    if(_mode == ControlModeEnums.AUTONOMOUS):
        _mode = ControlModeEnums.MANUAL
    else:
        _mode = ControlModeEnums.AUTONOMOUS
    _mode_lock.release()
