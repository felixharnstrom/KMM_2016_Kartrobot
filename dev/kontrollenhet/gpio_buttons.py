import RPi.GPIO as GPIO
import sys
import os
import time
import threading
import mode

SHUTDOWN_PIN = 16
CHANGE_MODE_PIN = 2 #CHANGE BACK WHEN FIXED

def launch_poll_threads():
    def init():
        # tell the GPIO module that we want to use the
        # chip's pin numbering scheme
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SHUTDOWN_PIN, GPIO.IN)
        GPIO.setup(CHANGE_MODE_PIN, GPIO.IN)
    
    def poll_shutdown():
        while True:
            time.sleep(0.05)
            if GPIO.input(SHUTDOWN_PIN):
                os.system("shutdown now")
            
    def poll_change_mode():
        while True:
            time.sleep(0.05)
            if GPIO.input(CHANGE_MODE_PIN):
                print("CHANGE_MODE_INPUT_DETECTED")
                mode.toggle_mode()
    init()
    shutdown_t = threading.Thread(target=poll_shutdown)
    shutdown_t.start()
    change_mode_t = threading.Thread(target=poll_change_mode)
    change_mode_t.start()
