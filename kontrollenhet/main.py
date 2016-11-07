
# Hack to be able to import packages from parent directory
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

def main():
    server = Server()
    print("Hello world")
    

if __name__ == "__main__":
    main()
