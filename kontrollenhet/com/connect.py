import bluetooth, time

def find(name):
    """Returns a string, or None

    Finds the address of a device with name. Returns None if no such devices can be found. Looks once."""
    nearby_devices = bluetooth.discover_devices(lookup_names=True)
    for nearby_addr, nearby_name in nearby_devices:
        if nearby_name == name:
            return nearby_addr
    return None

def find_timed(name, timeout):
    """Returns a string

    Finds the address within a given timeout (plus the time for the final scan to finish) Returns None if none can be found."""
    time_elapsed = 0.0
    while(time_elapsed < timeout):
        start_time = time.time()
        result = find(name)
        end_time = time.time()
        time_elapsed += end_time - start_time
        if result is not None:
            return result
    return None

def list_devices():
    """Prints out names and adresses of nearby devices"""
    nearby_devices = bluetooth.discover_devices(lookup_names=True)
    for addr, name in nearby_devices:
        print("  %s - %s" % (addr, name))

def connect_rfcomm_server():
    server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

    port = 1
    print("Listening on port", port)
    
    server_sock.bind(("00:15:83:15:A3:10", port))
    server_sock.listen(1)

    print("listened")

    client_sock, addr = server_sock.accept()
    print("Accepted ", addr)

    data = client_sock.recv(1024)
    print("received [%s]" % data)

    client_sock.close()
    server_sock.close()

def connect_rfcomm_client():
    # Figure this one out through hciconfig
    # find doesn't seem to work, need to test with 2 blutooths.
    addr = "00:15:83:15:A3:10"
    
    if addr is None:
        print("Cant find host")
        return
    
    port = 1

    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((addr, port))

    sock.send("hello!!")

    sock.close()

def debug_test():
    if (input() == "a"):
        print("server")
        connect_rfcomm_server()
    else:
        print("client")
        #list_devices()
        connect_rfcomm_client()
