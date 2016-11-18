from datetime import datetime
import time

class Pid():

    time_last = None
    input_data = 0
    output_data = 0
    setpoint = 0
    err_sum = 0
    last_input = 0
    kp = 0
    ki = 0
    kd = 0
    sample_time = 1000  # 1 sec

    def __init__(self):
        self.time_last = datetime.now()     # Need this to calculate the first time diff in compute()

    def compute(self):
        time_now = datetime.now()
        diff = time_now - self.time_last
        time_change = diff.microseconds / 1000              # Time diff in millis
        if (time_change >= sample_time):


        # Error variables
        error = self.setpoint - self.input_data
        self.err_sum += error
        d_input = (self.input_data - self.last_input)

        self.output_data = (self.kp * error) + (self.ki * self.err_sum) - (self.kd * d_input)

        self.last_input = input_data
        self.time_last = time_now

    def set_tunings(self, p : float, i: float, d : float):
        sample_time_in_sec = sample_time / 1000
        self.kp = p
        self.ki = i * sample_time_in_sec
        self.kd = d / sample_time_in_sec

    def SetSampleTime(new_sample_time : int):
        ratio  = new_sample_time / self.sample_time
        ki *= ratio
        kd /= ratio
        self.sample_time = new_sample_time
}