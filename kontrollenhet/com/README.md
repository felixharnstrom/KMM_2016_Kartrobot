Requires pybluez. Install it using the following. Some of these may be redundant.

```
apt-get install python-bluez && sudo apt-get install libbluetooth-dev && pip3 install pybluez
```

In order for the scripts to work the devices needs to be paired, do this via the OS. The PI`s PIN is 0000.

-------------------

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