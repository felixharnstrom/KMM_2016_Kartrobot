
from enum import Enum

class Instruction(Enum):
    """Available instructions.

    Values should range from 0 up to 255."""
    
    """Echoes args."""
    echo = 0
    
    
def process_request(request_data):
    """Process an incoming instruction and return response to client.

    request_data is bytes with a minimum length of 1.
    The first byte is the instruction performed, the rest are arguments,
    treated differently depending on instruction."""
    instr = request_data[0]
    args = request_data[1:] if len(request_data) > 1 else bytes(0)

    if instr == Instruction.echo.value:
        return args
    # ...
    else: # Others
        return bytes(0)
