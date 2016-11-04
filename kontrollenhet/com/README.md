Requires pybluez. Install it using the following. Some of these may be redundant.

```
apt-get install python-bluez && sudo apt-get install libbluetooth-dev && pip3 install pybluez
```

Heres a short history of the arcane bullshit I did to get everything up and running. Some of this might not be necessary.

Find the file /etc/systemd/system/dbus-org.bluez.service and add a -C after the line ending with bluetoothd. This enables a compatability mode.

Then load serial port profile using

```
sudo sdptool add SP
```

Find /etc/bluetooth/main.conf and add the line DisablePlugins = pnat

Restart the bluetooth service using

```
sudo invoke–rc.d bluetooth restart
```