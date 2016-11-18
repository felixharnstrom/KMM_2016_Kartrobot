from datetime import datetime
import time

class Pid():

    time_last = None
    input_data = 0
    output_data = 0
    setpoint = 0
    err_sum = 0
    last_err = 0
    kp = 0
    ki = 0
    kd = 0

    def __init__(self):
        self.time_last = datetime.now()

    def compute(self):
        time_now = datetime.now()
        diff = time_now - self.time_last
        time_change = diff.microseconds / 1000
        print (time_change)

        error = self.setpoint - self.input_data             # Proportional
        self.err_sum += (error * time_change)               # Integral
        err_d = (error - self.last_err) / time_change       # Derivative

        self.output_data = (self.kp * error) + (self.ki * self.err_sum) + (self.kd * err_d)

        self.last_err = error
        self.time_last = time_now

    def set_tunings(self, p : float, i: float, d : float):
        self.kp = p
        self.ki = i
        self.kd = d