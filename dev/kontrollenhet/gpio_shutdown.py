
import RPi.GPIO as GPIO
import sys
import os
import time
import threading

SHUTDOWN_PIN = 2
CHANGE_MODE_PIN = 14

def init():
    # tell the GPIO module that we want to use the
    # chip's pin numbering scheme
    GPIO.setmode(GPIO.BCM)
    # Set pin 2 as in
    GPIO.setup(SHUTDOWN_PIN, GPIO.IN)
    GPIO.setup(CHANGE_MODE_PIN, GPIO.IN)

def change_mode_is_pressed():
    return GPIO.input(CHANGE_MODE_PIN)
    
def poll_shutdown():
    if GPIO.input(SHUTDOWN_PIN):
        # Turn pi off
        os.system("shutdown now")

def poll_shutdown_in_other_thread():
    
    def other_thread():
        init()
        while True:
            time.sleep(0.05)
            poll_shutdown()

    t = threading.Thread(target=other_thread)
    t.start()
    return t



# Test
#poll_shutdown_in_other_thread()
#time.sleep(600)


