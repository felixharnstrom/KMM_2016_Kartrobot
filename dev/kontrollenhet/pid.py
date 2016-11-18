from datetime import datetime
import time

class Pid():

    time_last = None
    input_data = 0
    output_data = 0
    setpoint = 0
    i_term = 0              # Integration term
    last_input = 0
    kp = 0
    ki = 0
    kd = 0
    sample_time = 1000      # 1 sec
    automatic_mode = False
    min_out = 0
    max_out = 0

    MANUAL = 0
    AUTOMATIC = 1

    def __init__(self):
        self.time_last = datetime.now()     # Need this to calculate the first time diff in compute()

    def compute(self):
        if (not self.automatic_mode):
            return

        time_now = datetime.now()
        diff = time_now - self.time_last
        time_change = diff.microseconds / 1000      # Time diff in millis
        if (time_change >= self.sample_time):


            # Error variables
            error = self.setpoint - self.input_data
            self.i_term += (self.ki * error)            # Removes output bump when changing tuning parameters on the fly

            # Clamping the integration term to min/max values, to avoid this growing even when output is limited
            if (self.i_term > self.max_out):
                self.i_term = self.max_out
            elif (self.i_term < self.min_out):
                self.i_term = self.min_out
            d_input = (self.input_data - self.last_input)

            self.output_data = (self.kp * error) + self.i_term - (self.kd * d_input)

            # Clamping output to min/max values
            if (self.output_data > self.max_out):
                self.output_data = self.max_out
            elif (self.output_data < self.min_out):
                self.output_data = self.min_out

            self.last_input = input_data
            self.time_last = time_now

    def set_tunings(self, p : float, i: float, d : float):
        sample_time_in_sec = self.sample_time / 1000
        self.kp = p
        self.ki = i * sample_time_in_sec
        self.kd = d / sample_time_in_sec

    def set_sample_time(self, new_sample_time : int):
        ratio  = new_sample_time / self.sample_time
        self.ki *= ratio
        self.kd /= ratio
        self.sample_time = new_sample_time

    def set_output_limits(self, min_out : float, max_out : float):

        self.min_out = min_out
        self.max_out = max_out

        # Clamping output to min/max values
        if (self.output_data > self.max_out):
            self.output_data = self.max_out
        elif (self.output_data < self.min_out):
            self.output_data = self.min_out

        # Clamping the integration term to min/max values, to avoid this growing even when output is limited
        if (self.i_term > self.max_out):
            self.i_term = self.max_out
        elif (self.i_term < self.min_out):
            self.i_term = self.min_out

    def set_mode(self, mode : int):
        new_mode = (mode == self.AUTOMATIC)
        if (new_mode and not self.automatic_mode):
            # From manual to auto
            self.initalize()
        self.automatic_mode = new_mode

    def initalize(self):
        self.last_input = self.input_data
        self.i_term = self.output_data

        # Clamping the integration term to min/max values, to avoid this growing even when output is limited
        if (self.i_term > self.max_out):
            self.i_term = self.max_out
        elif (self.i_term < self.min_out):
            self.i_term = self.min_out