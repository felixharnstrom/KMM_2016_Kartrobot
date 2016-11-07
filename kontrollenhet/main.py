
# Hack to be able to import packages from parent directory
# Basically fools main into believing it resides in ..
# Only works in unix.
# TODO: Temporary solution. Look into other solutions.
import sys
from pathlib import Path
if __name__ == '__main__' and __package__ is None:
    top = Path(__file__).resolve().parents[2]
    sys.path.append(str(top))
    import Kartrobot
    __package__ = 'Kartrobot'

# Actual imports
from .bluetooth.server import Server
from process_request import process_request


def main():
    server = Server()
    print("Waiting for client connection.")
    server.advertise_and_connect()
    print("Connected succesfully")

    # Main loop
    while True:

        #print(process_request(bytes(10)))
        #continue
    
        # Process incoming messages
        while server.messages_queued():
            server.send(process_request(server.receive()))

        # TODO: do other stuff
    

if __name__ == "__main__":
    main()
