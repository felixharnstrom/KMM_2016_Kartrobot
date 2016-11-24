import threading
from enum import Enum

class ControlModeEnums(Enum):
    AUTO = 0
    MANUAL = 1

__mode = ControlModeEnums.MANUAL
__mode_lock = threading.Lock()

def set_mode(new_mode : ControlModeEnums):
    __mode_lock.acquire()
    __mode = new_mode
    __mode_lock.release()

def get_mode():
    __mode_lock.acquire()
    mode_enum = __mode
    __mode_lock.release()
    return mode_enum
