from bluetooth import *
import constants

def list_devices():
    """Prints out names and adresses of nearby devices"""
    nearby_devices = discover_devices(lookup_names=True)
    for addr, name in nearby_devices:
        print("  %s - %s" % (addr, name))

def connect_rfcomm_server():
    """Returns a tuple: (server socket, client socket)

    Advertises an rfcomm service and waits for a client to connect."""
    server_sock=BluetoothSocket( RFCOMM )
    server_sock.bind(("", PORT_ANY))
    server_sock.listen(1)
    port = server_sock.getsockname()[1]

    advertise_service(server_sock, "SampleServer",
                      service_id=constants.UUID,
                      service_classes=[constants.UUID, SERIAL_PORT_CLASS],
                      profiles=[SERIAL_PORT_PROFILE])

    client_sock, client_info = server_sock.accept()
    return server_sock, client_sock
    

def connect_rfcomm_client(addr = None):
    """Returns a bluetooth socket, or None

    Attempts to find an advertised rfcomm server and connect to it.
    Returns None if no such server found."""
    
    # Search for service
    service_matches = find_service( uuid = constants.UUID, address = addr )

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


def send_bytes(client_socket, data):
    """Send byte data to connected socket"""
    data_size = len(data).to_bytes(BYTES_FOR_PACKET_SIZE,
                                        byteorder=BYTE_ORDER,
                                        signed=True)
    msg = data_size + data
    client_socket.sendall(msg)

def receive_bytes(client_socket):
    """Returns received bytes

    Waits for client to send bytes data."""
    # Receive with a bigger buffer than necessary to be safe
    first = client_socket.recv(2*MAXIMUM_PACKET_SIZE)

    # Extract data
    data_size = int.from_bytes(first[:BYTES_FOR_PACKET_SIZE],
                               byteorder=BYTE_ORDER,
                               signed=True)
    msg = first[BYTES_FOR_PACKET_SIZE:]

    # While part of message is missing, keep receiving
    while (len(msg) < data_size):
        msg += client_socket.recv(2*MAXIMUM_PACKET_SIZE)
    return msg

