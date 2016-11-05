from bluetooth import *
import time

UUID = "01365216-a33e-11e6-80f5-76304dec7eb7"

def list_devices():
    """Prints out names and adresses of nearby devices"""
    nearby_devices = discover_devices(lookup_names=True)
    for addr, name in nearby_devices:
        print("  %s - %s" % (addr, name))

def connect_rfcomm_server():
    """Returns a tuple: (server socket, client socket)

    Advertises an rfcomm service and waits for a client to connect."""
    server_sock=BluetoothSocket( RFCOMM )
    server_sock.bind(("",PORT_ANY))
    server_sock.listen(1)
    port = server_sock.getsockname()[1]

    advertise_service( server_sock, "SampleServer",
                       service_id = UUID,
                       service_classes = [ UUID, SERIAL_PORT_CLASS ],
                       profiles = [ SERIAL_PORT_PROFILE ]
#                      protocols = [ OBEX_UUID ] 
                     )

    client_sock, client_info = server_sock.accept()
    return server_sock, client_sock
    

def connect_rfcomm_client(addr = None):
    """Returns a bluetooth socket, or None

    Attempts to find an advertised rfcomm server and connect to it.
    Returns None if no such server found."""
    
    # Search for service
    service_matches = find_service( uuid = UUID, address = addr )

    # Terminate if no server found
    if len(service_matches) == 0:
        return None

    # Get service data
    first_match = service_matches[0]
    port = first_match["port"]
    name = first_match["name"]
    host = first_match["host"]

    # Connect
    sock=BluetoothSocket( RFCOMM )
    sock.connect((host, port))
    return sock

def debug_test():
    if (input() == "a"):
        print("server")
        a, b = connect_rfcomm_server()
        a.close()
    else:
        print("client")
        #list_devices()
        rslt = connect_rfcomm_client()
        print(rslt)
        rslt.close()
