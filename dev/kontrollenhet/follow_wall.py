"""
This program endlessly follows the wall on the robot's right side. The USB devices for the control and sensor units are supplied as command-line arguments.

Examples:
        $ python3 follow_wall.py -s 'TTYUSB0' -c 'TTYUSB1'

        $ python3 follow_wall.py -sensor 'TTYUSB0' -control 'TTYUSB1' -v 3

Todo:
    * Automatically detect USB serial devices.
    * Use Daniel's enum for MANUAL, AUTONOM.
    * Consider adjusting to wall to improve final heading after turn().
    * Add the correct code for updating the path_queue in find_next_destination().
    * Add cases for corridors, open rooms, crossings, etc.
    * Use the Direction class instead of hard-coded directions.
    * Complete docstrings.
"""
import time
import math
import sys
import argparse
import numpy as np
from command import Command
from UART import UART
from pid import Pid

VERBOSITY = 1   # Default global verbosity level, can be set by command-line argument.

# TODO: Use Daniel's enum for MANUAL, AUTONOM
class ControllerMode:
    MANUAL = 0
    AUTONOM = 1

# Define directions
class Direction:
    LEFT = 0
    RIGHT = 1

class DriveStatus:
    OBSTACLE_DETECTED = 0
    RIGHT_CORRIDOR_DETECTED = 1
    DONE = 2

class Robot:
    """
    Contains the robot's current state and several functions enabling the robot's high-level behaviour.

    Args:
        mode            (ControllerMode): Whether it should start in manual or autonomous mode.
        sensor_device   (str): Device name of the USB<->Serial converter for the sensor unit.
        control_device  (str): Device name of the USB<->Serial converter for the control unit.

    Attributes:
        driven_distance         (int): Driven distance thus far.
        current_angle           (int): Current angle against right wall.
        control_mode            (ControllerMode): Current controller mode.
        path_trace              (?): ? # TODO: What is this?
        path_queue              (?): ? # TODO: What is this?
        uart_sensorenhet        (str): Device name of the USB<->Serial converter for the sensor unit.
        uart_styrenhet          (str): Device name of the USB<->Serial converter for the control unit.
        pid_controller          (Pid): The PID controller in use.
        BLOCK_SIZE              (int): Block size in millimetres.
        IR_MEDIAN_ITERATIONS    (int): Number of values to get from IR sensors.
        GYRO_MEDIAN_ITERATIONS  (int): Number of values to get from gyro.
        TURN_OVERRIDE_DIST      (int): If the right-hand sensor reaches or exceeds this value it will immediately trigger a right turn. 
        TURN_MIN_DIST           (int): The minimum detected distance that will trigger a right turn.
        RIGHT_TURN_ENTRY_DIST   (int): How far to drive into a corner before performing a 90° turn.
        RIGHT_TURN_EXIT_DIST    (int): How far to drive into a corridor after performing a 90° turn.
        EDGE_SPIKE_FACTOR       (int): How much bigger a spike in IR value must be compared to prior reading until it will trigger an edge detection.
        OBSTACLE_DIST           (int): The maximum distance until an obstacle ahead will trigger a turn.
        SENSOR_SPACING          (int): Distance between side sensors on the robot.
        BASE_SPEED              (int): The speed that seems to work best for the controller with fully charged batteries.
        ACCELERATED_SPEED       (int): Can be used when the controller is disengaged, otherwise too fast for the controller to handle.
    """

    def __init__(self, mode : ControllerMode, sensor_device : str, control_device : str):
        # Public attributes
        self.driven_distance = 0
        self.current_angle = 0
        self.control_mode = mode
        self.path_trace = [] # (Angle, LengthDriven), with this list we can calculate our position
        self.path_queue = [] # (Blocks_To_Drive, Direction)
        self.uart_sensorenhet = UART(sensor_device)
        self.uart_styrenhet = UART(control_device)
        self.BLOCK_SIZE = 400
        self.IR_MEDIAN_ITERATIONS = 3
        self.GYRO_MEDIAN_ITERATIONS = 32
        self.TURN_OVERRIDE_DIST = 300
        self.TURN_MIN_DIST = 100
        self.RIGHT_TURN_ENTRY_DIST = 100
        self.RIGHT_TURN_EXIT_DIST = 300
        self.OBSTACLE_SAFETY_OVERRIDE = 120
        self.EDGE_SPIKE_FACTOR = 2
        self.OBSTACLE_DIST = 170
        self.SENSOR_SPACING = 95
        self.BASE_SPEED = 20
        self.ACCELERATED_SPEED = 40

        # Private attributes
        self._last_dist = 0

        # Initialize PID controller
        self.pid_controller = Pid()
        self.pid_controller.setpoint = 80
        self.pid_controller.output_data = 0
        self.pid_controller.set_tunings(3, 0, -200)
        self.pid_controller.set_sample_time(33)
        self.pid_controller.set_output_limits(-50, 50)
        self.pid_controller.set_mode(1)

    def read_sensor(self, sensor_instr : Command):
        """
        Return the current value of the given sensor.

        Args:
            sensor_instr (Command): A sensor command.

        Returns:
            (int): Distance in millimetres.
        """
        self.uart_sensorenhet.send_command(sensor_instr)
        self.uart_sensorenhet.receive_packet()
        adress, length, typ = self.uart_sensorenhet.decode_metapacket(self.uart_sensorenhet.receive_packet())

        highest = self.uart_sensorenhet.receive_packet()
        lowest = self.uart_sensorenhet.receive_packet()
        return int.from_bytes(highest + lowest, byteorder = "big", signed = True)

    def turn(self, direction : Direction, degrees : int, speed: int):
        """
        Turn a certain amount in the given direction at the given speed.

        Args:
            direction   (Direction): Direction to turn.
            degrees     (int): Degrees to turn.
            speed       (int): Speed as a percentage value between 0-100.
        """
        # TODO: Consider adjusting to wall if we have something on the side
        # Set the current direction to zero
        current_dir = 0

        # Set time to zero to turn untill stopped
        standstill_rate = self._median_sensor(self.GYRO_MEDIAN_ITERATIONS, Command.read_gyro()) / 100
        turn_instr = Command.turn(direction, speed, 0)
        self.uart_styrenhet.send_command(turn_instr)

        while (abs(current_dir) < degrees):
            # Use a reimann sum to add up all the gyro rates
            # Area = TimeDiff * turnRate
            clk = time.time()
            turn_rate = self._median_sensor(self.GYRO_MEDIAN_ITERATIONS, Command.read_gyro()) / 100 - standstill_rate
            current_dir += (time.time() - clk) * turn_rate
            if (VERBOSITY >= 2):
                print("Current dir: " + str(current_dir) + " -- Turn rate: " + str(turn_rate))

        # Turning is completed, stop the motors
        turn_instr = Command.stop_motors()
        self.uart_styrenhet.send_command(turn_instr)

        # Add turned degrees to current_angle
        if direction == Direction.LEFT:
            self.current_angle -= degrees
        else:
            self.current_angle += degrees

    def drive_distance(self, dist : int, speed : int):
        """
        Drives the given distance at the given speed, if possible. Uses LIDAR for determining distance.

        Args:
            dist    (int): Distance in millimetres.
            speed   (int): Speed as a percentage value between 0-100.
        Returns:
            (bool): True if the given distance was driven, false if an obstacle stopped it.
        """
        self.uart_styrenhet.send_command(Command.drive(1, speed, 0))
        lidar_init = self.read_sensor(Command.read_lidar())
        lidar_cur = lidar_init
        if (VERBOSITY >= 2):
            print("LIDAR INIT: ", lidar_init)
        while(lidar_init - lidar_cur < dist):
            lidar_cur = self.read_sensor(Command.read_lidar())
            if lidar_cur < self.OBSTACLE_SAFETY_OVERRIDE:
                self.uart_styrenhet.send_command(Command.stop_motors())
                return False
            if (VERBOSITY >= 2):
                print("LIDAR CURRENT: ", lidar_cur)
        self.uart_styrenhet.send_command(Command.stop_motors())
        return True

    def follow_wall(self, distance : int):
        """
        Follow the wall to the right until the robot encounters a corner or obstacle, or the given distance to drive is reached.

        Args:
            dist (int): Distance in millimetres.

        Returns:
            (DriveStatus): Status when completed.
        """

        # Set time to zero to turn untill stopped
        standstill_rate = self._median_sensor(self.GYRO_MEDIAN_ITERATIONS, Command.read_gyro()) / 100

        # Read start values from sensors
        lidar = self.read_sensor(Command.read_lidar())
        start_lidar = self.read_sensor(Command.read_lidar())

        self._last_dist = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_right_front_ir())

        # Drive until the wanted distance is reached
        while (start_lidar - lidar <= distance):
            # Keep track of current angle
            # Use a reimann sum to add up all the gyro rates
            # Area = TimeDiff * turnRate
            clk = time.time()
            turn_rate = self._median_sensor(self.GYRO_MEDIAN_ITERATIONS, Command.read_gyro()) / 100 - standstill_rate

            # Get sensor values
            ir_right_front = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_right_front_ir())
            ir_right_back = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_right_back_ir())
            ir_left_front = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_left_front_ir())
            ir_left_back = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_left_back_ir())
            lidar = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_lidar())
            if (VERBOSITY >= 3):
                print ("IR_RIGHT_BACK: " + str(ir_right_back) + " IR_RIGHT_FRONT: " + str(ir_right_front) + " IR_LEFT_BACK: " + str(ir_left_back) + " IR_LEFT_FRONT: " + str(ir_left_front) + " LIDAR: " + str(lidar) + " _last_dist: " + str(self._last_dist))

             # Detect corridor to the right
            if ((ir_right_front >= self.TURN_OVERRIDE_DIST) or (ir_right_front > self.TURN_MIN_DIST and ir_right_front > self.EDGE_SPIKE_FACTOR * self._last_dist)):
                self.drive_distance(self.RIGHT_TURN_ENTRY_DIST, self.BASE_SPEED)
                # Save the given length driven
                self.driven_distance += start_lidar - lidar
                self._save_position()
                self._last_dist = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_right_front_ir())
                return DriveStatus.RIGHT_CORRIDOR_DETECTED

            # Obstacle detected, and no turn to the right
            elif(lidar < self.OBSTACLE_DIST):
                # Save the given length driven
                self.uart_styrenhet.send_command(Command.stop_motors())
                self.driven_distance += start_lidar - lidar
                self._save_position()
                return DriveStatus.OBSTACLE_DETECTED

            # We need to get the distance from the center of the robot perpendicular to the wall
            # This assumes that we have a wall on either side (i.e. we are in a corridor)
            # TODO: Different cases for corridors and open rooms.
            dist_right = (ir_right_front + ir_right_back) / 2
            angle_right = math.atan2(ir_right_back - ir_right_front, self.SENSOR_SPACING)
            perpendicular_dist_right = dist_right * math.cos(angle_right)
            if (VERBOSITY >= 3):
                print ("PERPENDICULAR DIST RIGHT:", perpendicular_dist_right)
            self.pid_controller.input_data = perpendicular_dist_right
            self.pid_controller.d_term = angle_right
            self.pid_controller.compute()
            self.pid_controller.output_data += 100
            print (self.pid_controller.output_data)
            self._follow_wall_help(self.pid_controller.output_data / 100, self.BASE_SPEED)
            
            self._last_dist = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_right_front_ir())

        # Save the given length driven
        self.uart_styrenhet.send_command(Command.stop_motors())
        self.driven_distance += distance
        self._save_position()
        return DriveStatus.DONE
    
    # TODO: Add the correct code for updating the path_queue when the code is ready
    def find_next_destination(self):
        """
        Find the next destionation and update path_queue.

        Returns:
            (list of ?): Path to drive. # TODO: What is this?
        """
        return [] # Return random path to drive

    def scan(self):
        """
        Scan the room at this position and return the recorded data.

        Returns:
            (list of ?): Recorded data. # TODO: What is this?
        """
        recorded_data = [] # (Angle, distance)
        # Turn LIDAR to 0 degrees
        servo_instr = Command.servo(0)
        self.uart_styrenhet.send_command(servo_instr)

        # Sleep 1 second to wait for LIDAR to be in position
        time.sleep(1)

        for i in range(180):
            # Point LIDAR in the right angle
            servo_instr = Command.servo(i)
            self.uart_styrenhet.send_command(servo_instr)

            # Read data from the LIDAR sensor
            lidar_distance = self.read_sensor(Command.read_lidar())
            recorded_data += [(i,lidar_distance)]
            time.sleep(0.005)

        return recorded_data

    def autonom_mode(self):
        """
        Drive autonomously through the room.
        """
        # Update the path_queue for positions to drive
        find_next_destination()


    def stand_perpendicular(self, side: str):
        """
        Turn to stand perpendicular to the wall at the given side.

        Args:
            side (str): 'left' or 'right'
        """
        if side == "left":
            ir_front = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_left_front_ir())
            ir_back = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_left_back_ir())
            angle = math.atan2(ir_back - ir_front, self.SENSOR_SPACING)
            self.turn(math.degrees(angle) > 0, abs(int(math.degrees(angle))), speed = self.BASE_SPEED+10)

        elif side == "right":
            ir_front = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_right_front_ir())
            ir_back = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_right_back_ir())
            angle = math.atan2(ir_back - ir_front, self.SENSOR_SPACING)
            self.turn(math.degrees(angle) < 0 , abs(int(math.degrees(angle))), speed = self.BASE_SPEED+10)

    def _median_sensor(self, it : int, sensor_instr : Command):
        """
        Return the median value of the specified number of readings from the given sensor.

        Args:
            it              (int): Number of sensor values.
            sensor_instr    (Command): A sensor command.

        Returns:
            (int): Median value.
        """
        values = []
        for i in range(it):
            values.append(self.read_sensor(sensor_instr))
        if len(values) == 1:
            return values[0]
        else:
            return np.median(values)

    def _save_position(self):
        """
        Save the collected data to the trace list.
        """
        self.path_trace += [(self.current_angle, self.driven_distance)]

    def _follow_wall_help(self, ratio : int, base_speed : int):
        """
        Calculate motor speeds and send drive instruction.

        Args:
            ratio       (int): Will drive forward if exactly 1, turn left if > 1, and turn right if < 1.
            base_speed  (int): Base speed that ratio will be applied to.
        """
        left_speed = max(min(base_speed / ratio, 100), 0)
        right_speed = max(min(base_speed * ratio, 100), 0)
        drive_instr = Command.side_speeds(1, round(left_speed), 1, round(right_speed))
        self.uart_styrenhet.send_command(drive_instr)

    def _is_moving(self, threshold = 30, wait_time = 0.2):
        """
        Checks for sensor changes above threshold within wait_time.
        Args:
            threshold   (int): How much the sensor values are allowed to change (mm)
            wait_time   (int): How long to wait between sensor reads
        Returns:
            (bool): If the robot has moved more than threshold.
        """
        ir_back = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_back_ir())
        lidar = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_lidar())
        ir_right_front = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_right_front_ir())
        ir_right_back = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_right_back_ir())
        ir_left_front = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_left_front_ir())
        ir_left_back = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_left_back_ir())

        time.sleep(wait_time)

        ir_back_diff = abs(ir_back - self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_back_ir()))
        lidar_diff = abs(lidar - self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_lidar()))
        ir_right_front_diff = abs(ir_right_front - self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_right_front_ir()))
        ir_right_back_diff = abs(ir_right_back - self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_right_back_ir()))
        ir_left_back_diff = abs(ir_left_back - self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_left_back_ir()))
        ir_left_front_diff = abs(ir_left_front - self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_left_front_ir()))

        if (lidar_diff < threshold and ir_right_back_diff < threshold and
            ir_right_front_diff < threshold and ir_back_diff < threshold and
            ir_left_back_diff < threshold and ir_left_front_diff < threshold):
            return False
        else:
            if VERBOSITY >= 2:
                print("MOVING: threshold=", threshold, "mm, wait_time", wait_time)
            return True


def sensor_test(robot):
    """
    Continuously get all sensor values and print these.
    """
    while 1:
        ir_right_front = robot._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_right_front_ir())
        ir_right_back = robot._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_right_back_ir())
        ir_left_front = robot._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_left_front_ir())
        ir_left_back = robot._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_left_back_ir())
        ir_back = robot._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_back_ir())
        lidar = robot._median_sensor(1, Command.read_lidar())
        print ("IR_RIGHT_BACK: " + str(ir_right_back) + " IR_RIGHT_FRONT : " + str(ir_right_front) + " LIDAR: " + str(lidar), "IR_LEFT_BACK", ir_left_back, "IR_LEFT_FRONT", ir_left_front, "IR_BACK", ir_back)
    
def main(argv):
    global VERBOSITY
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--sensor", required = True, metavar = "SENSOR_DEVICE", help = "Device name of the USB<->serial converter for the sensor unit.")
    parser.add_argument("-c", "--control", required = True, metavar = "CONTROL_DEVICE", help = "Device name of the USB<->serial converter for the control unit.")
    parser.add_argument("-v", "--verbosity", type = int, default = 1, choices = range(0, 4), help = "Verbosity level. 0 is lowest and 3 highest. Default 1.")
    args = parser.parse_args()
    VERBOSITY = args.verbosity
    robot = Robot(ControllerMode.MANUAL, args.sensor, args.control)

    # Turn LIDAR to 90 degrees
    servo_instr = Command.servo(90)
    robot.uart_styrenhet.send_command(servo_instr)

    # Wait fot LIDAR to be in position
    time.sleep(1)
    
    # Try driving in infinite loop around the maze
    while 1:
            # Drive forward without crashing in wall
            status = robot.follow_wall(9999999)
            if VERBOSITY >= 1:
                print(robot.path_trace)
            if (status == DriveStatus.OBSTACLE_DETECTED):
                if (VERBOSITY >= 1):
                    print ("---------- OBSTACLE DETECTED!")
                    print ("---------- TURNING LEFT 90 degrees")
                turn_instr = Command.stop_motors()
                robot.uart_styrenhet.send_command(turn_instr)
                while robot._is_moving(): pass
                ir_right_front = robot._median_sensor(robot.IR_MEDIAN_ITERATIONS, Command.read_right_front_ir())
                ir_left_front = robot._median_sensor(robot.IR_MEDIAN_ITERATIONS, Command.read_left_front_ir())
                if (ir_right_front > ir_left_front):
                    robot.stand_perpendicular('left')
                    robot.turn(Direction.RIGHT, 80, speed = robot.ACCELERATED_SPEED)
                    while robot._is_moving(): pass
                    robot.stand_perpendicular('left')
                else:
                    robot.stand_perpendicular('right')
                    robot.turn(Direction.LEFT, 80, speed = robot.ACCELERATED_SPEED)
                    while robot._is_moving(): pass
                    robot.stand_perpendicular('right')
                robot.pid_controller.kp = 0
                robot.pid_controller.ki = 0
                robot.pid_controller.kd = 0
                robot.pid_controller.set_tunings(3, 0, -200)
            elif (status == DriveStatus.RIGHT_CORRIDOR_DETECTED):
                if (VERBOSITY >= 1):
                    print ("---------- DETECTED CORRIDOR TO RIGHT!")
                    print ("---------- TURNING RIGHT 90 degrees")
                turn_instr = Command.stop_motors()
                robot.uart_styrenhet.send_command(turn_instr)
                while robot._is_moving(): pass
                #robot.stand_perpendicular('left')
                robot.drive_distance(robot.RIGHT_TURN_ENTRY_DIST, robot.BASE_SPEED)
                robot.turn(Direction.RIGHT, 80, speed = robot.ACCELERATED_SPEED)
                while robot._is_moving(threshold = 20): pass

                # TODO: Detection works, but seems to commonly result in the robot standing staring at a wall, and obstacle detection.
                if robot._median_sensor(1, Command.read_lidar()) < robot.BLOCK_SIZE:
                    if VERBOSITY >= 1:
                        print("Not a corridor, moving back")
                    robot.turn(Direction.LEFT, 80, speed = robot.ACCELERATED_SPEED)
                    robot.stand_perpendicular('right')
                    while robot._is_moving(threshold = 20): pass
                else:
                    # TODO: Find a way to do stand_perpendicular when there is no wall to the left. Or lower RIGHT_TURN_EXIT_DIST again.
                    # As it is now, a right turn into a single square corridor does not work well.
                    robot.drive_distance(robot.RIGHT_TURN_EXIT_DIST, robot.BASE_SPEED)
                    while robot._is_moving(): pass
                    robot.stand_perpendicular('right')
                    robot.stand_perpendicular('left')
                robot.pid_controller.kp = 0
                robot.pid_controller.ki = 0
                robot.pid_controller.kd = 0
                robot.pid_controller.set_tunings(3, 0, -200)

if __name__ == "__main__":
    main(sys.argv[1:])
