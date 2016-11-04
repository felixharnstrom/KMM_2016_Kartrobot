Requires pybluez. Install it using the following. Some of these may be redundant.

```
apt-get install python-bluez && sudo apt-get install libbluetooth-dev && pip3 install pybluez
```

Then...

Heres a short history of the arcane bullshit I did to get everything up and running. Everything may not be required.

Find the file /etc/systemd/system/dbus-org.bluez.service and add a -C after the line ending with bluetoothd.

Find /etc/bluetooth/main.conf and add the line DisablePlugins = pnat

Restart the bluetooth service using

```
sudo invoke–rc.d bluetooth restart
```