
# Setup

Requires pybluez. Install it using the following. Some of these may be redundant.

```
apt-get install python-bluez libbluetooth-dev && pip3 install pybluez
```

In order for the scripts to work the devices needs to be paired, do this via the OS. The PI`s PIN is 0000.

# Additional setup

Heres a short history of the arcane bullshit I did to get everything up and running. Some (or all) of this might not be necessary.

Find the file /etc/systemd/system/dbus-org.bluez.service and add a -C after the line ending with bluetoothd. This enables a compatability mode.

Then load serial port profile using

```
sudo sdptool add SP
```

Find /etc/bluetooth/main.conf and add the line DisablePlugins = pnat

This is to get rid off the server never accepting requests on certain OS`s, like raspbian.

Restart the bluetooth service using

```
sudo invoke–rc.d bluetooth restart
```

# Usage

You'll primarily use the Client and Server classes. The Client can send messages to the server and give you the response, while the server can send messages and receive messages.

First, the two needs to connect. In order for them to even be able to find each other, the bluetooth modules needs to be paired. This is done in the OS on both machines.

Have the Server advertise it's pressence, and the Client connect. Multiple connection attempts may be required.

Use the following psuedocode for the server-side main loop, after establishing a connection:

```
Receive a message
Process the message
Send response to sender
Repeat
```