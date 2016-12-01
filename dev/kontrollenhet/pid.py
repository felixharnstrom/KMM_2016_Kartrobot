"""
A generic PID controller.
"""

from datetime import datetime

class Pid():
    """
    PID controller.

    Attributes:
        input_data      (int): Input.
        output_data     (int): Output.
        setpoint        (int): Value we're aiming for.
        kp              (int): Proportional tuning parameter.
        ki              (int): Integrating tuning parameter.
        kd              (int): Derivative tuning parameter.
        sample_time     (int): Sample time in milliseconds.
        automatic_mode  (bool): True = PID on, False = PID off.
        min_out         (int): Will clamp output to at least this value.
        max_out         (int): Will clamp output to at most this value.
    """

    def __init__(self):
        # Public attributes
        self.input_data = 0
        self.output_data = 0
        self.setpoint = 0
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.sample_time = 1000
        self.automatic_mode = False 
        self.min_out = 0
        self.max_out = 0

        # Private attributes
        self._i_term = 0                    # Integration term
        self._last_input = 0                # Last input value
        self._time_last = datetime.now()    # Time since last compute(), need this to calculate the first time diff in compute()
        self._MANUAL = 0
        self._AUTOMATIC = 1

    def compute(self):
        """
        Compute output value from current input and PID terms. Does nothing if not in automatic mode.
        """
        if (not self.automatic_mode):
            return

        # Calculate time diff
        time_now = datetime.now()
        diff = time_now - self._time_last
        time_change = diff.microseconds / 1000      # Time diff in millis
        self.set_sample_time(time_change)

        # Error variables
        error = self.setpoint - self.input_data
        self._i_term += (self.ki * error)            # Removes output bump when changing tuning parameters on the fly

        # Clamping the integration term to min/max values, to avoid this growing even when output is limited
        if (self._i_term > self.max_out):
            self._i_term = self.max_out
        elif (self._i_term < self.min_out):
            self._i_term = self.min_out
        d_input = (self.input_data - self._last_input)

        self.output_data = (self.kp * error) + self._i_term - (self.kd * d_input)

        # Clamping output to min/max values
        if (self.output_data > self.max_out):
            self.output_data = self.max_out
        elif (self.output_data < self.min_out):
            self.output_data = self.min_out

        self._last_input = self.input_data
        self._time_last = time_now

    def set_tunings(self, p : float, i: float, d : float):
        """
        Sets the tuning parameters to be multiplied with the proportional, integrating, and derivative terms.

        Args:
            p (int): Proportional tuning parameter.
            i (int): Integrating tuning parameter.
            d (int): Derivative tuning parameter.
        """
        sample_time_in_sec = self.sample_time / 1000
        self.kp = p
        self.ki = i * sample_time_in_sec
        self.kd = d / sample_time_in_sec

    def set_sample_time(self, new_sample_time : int):
        """
        Sets the sampling time. Used to calculate the integrating and derivative term.

        Args:
            new_sample_time (int): Sample time in milliseconds.
        """
        ratio  = new_sample_time / self.sample_time
        self.ki *= ratio
        self.kd /= ratio
        self.sample_time = new_sample_time

    def set_output_limits(self, min_out : float, max_out : float):
        """
        Sets the minimun and maximum output values. The output as well as the integrating term will be clamped to these values.

        Args:
            min_out (int): Minimum output.
            max_out (int): Maximum output.
        """
        self.min_out = min_out
        self.max_out = max_out

        # Clamping output to min/max values
        if (self.output_data > self.max_out):
            self.output_data = self.max_out
        elif (self.output_data < self.min_out):
            self.output_data = self.min_out

        # Clamping the integration term to min/max values, to avoid this growing even when output is limited
        if (self._i_term > self.max_out):
            self._i_term = self.max_out
        elif (self._i_term < self.min_out):
            self._i_term = self.min_out

    def set_mode(self, mode : int):
        """
        Sets the controller to automatic or manual mode.

        Args:
            mode (int): 0 for manual, 1 for automatic.
        """
        new_mode = (mode == self._AUTOMATIC)
        if (new_mode and not self.automatic_mode):
            # From _MANUAL to auto
            self.initialize()
        self.automatic_mode = new_mode

    def initialize(self):
        """
        Initalization for bumpless transfer.
        """
        self._last_input = self.input_data
        self._i_term = self.output_data

        # Clamping the integration term to min/max values, to avoid this growing even when output is limited
        if (self._i_term > self.max_out):
            self._i_term = self.max_out
        elif (self._i_term < self.min_out):
            self._i_term = self.min_out