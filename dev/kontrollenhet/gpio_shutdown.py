
import RPi.GPIO as GPIO
import sys
import os
import time
import threading

INPUT_PIN = 2

def init():
    # tell the GPIO module that we want to use the
    # chip's pin numbering scheme
    GPIO.setmode(GPIO.BCM)
    # Set pin 2 as in
    GPIO.setup(INPUT_PIN, GPIO.IN)
    
def poll():
    if GPIO.input(INPUT_PIN):
        # Turn pi off
        os.system("shutdown now")

def poll_in_other_thread():
    
    def other_thread():
        init()
        while True:
            time.sleep(0.05)
            poll()

    t = threading.Thread(target=other_thread)
    t.start()
    return t

poll_in_other_thread()
time.sleep(600)


