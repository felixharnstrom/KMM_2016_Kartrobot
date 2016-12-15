"""
This program endlessly follows the wall on the robot's right side. The USB devices for the control and sensor units are supplied as command-line arguments.

Examples:
        $ python3 follow_wall.py -s 'ttyUSB0' -c 'ttyUSB1'

        $ python3 follow_wall.py -sensor 'ttyUSB0' -control 'ttyUSB1' -v 3

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
import logging
from map import *
import numpy as np
from command import Command
from UART import UART
from pid import Pid
from robot_communication import handle_command, init_UARTs


import map
import grid_map
from geometry import Position
import time



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
    CORRIDOR_DETECTED_RIGHT = 1
    CORRIDOR_DETECTED_LEFT = 2
    DONE = 3

class Goal:
    MAP_OUTER_WALLS = 0
    FIND_ISLAND = 1
    DRIVE_TO_ISLAND = 2
    MAP_ISLAND = 3
    RETURN_HOME = 4
    NONE = 5

def unknown_in_view(location: Position, angle: int, g: GridMap, move_forward: int):
    """
    Checks if there are unknowns in the row or column in front of the location.

    Args:
        :param location (Position): The robots current location
        :param angle (int): In which direction the robot is looking (0 == Right, 90 == Up, ...)7
        :param move_forward (int): How many cells in front of the robots current location to check. (0 == current row/column)

    Returns:
        :return (bool): If there is a unknow cell in the row/column specified.
    """

    angle = angle % 360
    if angle == 0 or angle == 180:
        print("THIS IS A COLUMN")
        # Scan one row forwards
        if angle == 0:
            scancolumn = location.x+move_forward
        elif angle == 180:
            scancolumn = location.x-move_forward
        # print("ScanColumn", scancolumn)

        if g.get(scancolumn, location.y) == CellType.WALL:
            raise ValueError("Robot will stand on wall")

        # Loop through current y-axis (since we are moving on the x-axis)
        # From current location to either unknow or a blocking wall.
        for y in range(location.y, g.bottom_right().y):
            print("LOOKING at x:", scancolumn, "y:", y)
            if g.get(scancolumn, y) == CellType.UNKNOWN:
                return True
                pass
            if g.get(scancolumn, y) == CellType.WALL:
                break
        # print(location.y, g.top_left().y)

        # Loop through current y-axis (since we are moving on the x-axis)
        # From current location to either unknow or a blocking wall.
        for y in range(location.y, g.top_left().y-1, -1):
            print("LOOKING at x:", scancolumn, "y:", y)
            if g.get(scancolumn, y) == CellType.UNKNOWN:
                return True
            if g.get(scancolumn, y) == CellType.WALL:
                break
            # print (g.get(location.x, y))
        return False
    elif angle == 90 or angle == 270:
        print("THIS IS A ROW")
        # Moving on y-axis.
        # Assuming up is 90 degrees, down is 270 degrees.
        # Scan one row forwards
        if angle == 90:
            scanrow = location.y-move_forward
        elif angle == 270:
            scanrow = location.y+move_forward

        # print("ScanRow", scanrow)

        if g.get(location.x, scanrow) == CellType.WALL:
            raise Exception("Robot will stand on wall")

        # Loop through current y-axis (since we are moving on the x-axis)
        # From current location to either unknow or a blocking wall.
        for x in range(location.x, g.bottom_right().x):
            # print("LOOKING at x:", x, "y:", scanrow)
            if g.get(x, scanrow) == CellType.UNKNOWN:
                return True
                pass
            if g.get(x, scanrow) == CellType.WALL:
                # print("met wall")
                break
        # print(location.x, g.top_left().x)
        # print((location.x, g.top_left().x-1, -1))
        # Loop through current y-axis (since we are moving on the x-arobotxis)
        # From current location to either unknow or a blocking wall.
        for x in range(location.x, g.top_left().x-1, -1):
            # print("LOOKING at x:", x, "y:", scanrow)
            if g.get(x, scanrow) == CellType.UNKNOWN:
                return True
            if g.get(x, scanrow) == CellType.WALL:
                break
        return False

    
class Robot:
    """
    Contains the robot's current state and several functions enabling the robot's high-level behaviour.

    Args:
        mode            (ControllerMode): Whether it should start in manual or autonomous mode.
        sensor_device   (str): Device name of the USB<->Serial converter for the sensor unit.
        control_device  (str): Device name of the USB<->Serial converter for the control unit.

    Attributes:
        goal                        (Goal): What we're currently trying to do
        return_home                 (bool): True if the robot should return home
        grid_map                    (GridMap): Mapping of room.
        start_cell_at_island        (Position): Start cell at island.

        driven_distance             (int): Driven distance thus far.
        current_angle               (int): Current angle against right wall.
        control_mode                (ControllerMode): Current controller mode.
        path_trace                  (?): ? # TODO: What is this?
        path_queue                  (?): ? # TODO: What is this?
        uart_sensorenhet            (str): Device name of the USB<->Serial converter for the sensor unit.
        uart_styrenhet              (str): Device name of the USB<->Serial converter for the control unit.
        pid_controller              (Pid): The PID controller in use.
        has_been_to_other_cell      (bool): Robot has left the starting position at the island. 
        START_X                     (int): Starting x-value, in mm
        BLOCK_SIZE                  (int): Block size in millimetres.
        IR_MEDIAN_ITERATIONS        (int): Number of values to get from IR sensors.
        GYRO_MEDIAN_ITERATIONS      (int): Number of values to get from gyro.
        TURN_OVERRIDE_DIST          (int): If the right-hand sensor reaches or exceeds this value it will immediately trigger a right turn. 
        TURN_MIN_DIST               (int): The minimum detected distance that will trigger a right turn.
        CORRIDOR_TURN_ENTRY_DIST       (int): How far to drive into a corner before performing a 90° turn.
        CORRIDOR_TURN_EXIT_DIST        (int): How far to drive into a corridor after performing a 90° turn.
        OBSTACLE_SAFETY_OVERRIDE    (int): The override distance that will trigger an immediate stop in drive_distance().
        EDGE_SPIKE_FACTOR           (int): How much bigger a spike in IR value must be compared to prior reading until it will trigger an edge detection.
        OBSTACLE_DIST               (int): The maximum distance until an obstacle ahead will trigger a turn.
        SENSOR_SPACING              (int): Distance between side sensors on the robot.
        BASE_SPEED                  (int): The speed that seems to work best for the controller with fully charged batteries.
        ACCELERATED_SPEED           (int): Can be used when the controller is disengaged, otherwise too fast for the controller to handle.
        WHEEL_RADIUS                (int): Radius for a wheel on the robot in cm.
        DRIVE_TO_ISLAND_THRESHOLD   (int): The robot won't drive to the island if it is further than this distance
    """

    def __init__(self, mode: ControllerMode, logger):
        # Public attributes
        self.grid_map = GridMap()
        self.goal = Goal.MAP_OUTER_WALLS
        self.start_cell_at_island = Position(0,0)
        self.driven_distance = 0
        self.current_angle = 0
        self.control_mode = mode
        self.path_trace = [] # (Angle, LengthDriven), with this list we can calculate our position
        self.path_queue = [] # (Blocks_To_Drive, Direction)
        self.START_X = 200
        self.BLOCK_SIZE = 400
        self.IR_MEDIAN_ITERATIONS = 3
        self.GYRO_MEDIAN_ITERATIONS = 32
        self.TURN_OVERRIDE_DIST = 220
        self.TURN_MIN_DIST = 100
        self.CORRIDOR_TURN_ENTRY_DIST = 130
        self.CORRIDOR_TURN_EXIT_DIST = 350
        self.OBSTACLE_SAFETY_OVERRIDE = 30
        self.EDGE_SPIKE_FACTOR = 2
        self.OBSTACLE_DIST = 80
        self.SENSOR_SPACING = 95
        self.BASE_SPEED = 40
        self.ACCELERATED_SPEED = 40
        self.WHEEL_RADIUS = 32.5
        self.DRIVE_TO_ISLAND_THRESHOLD = 800
        self.logger = logger
        self.has_been_to_other_cell = False
        # Private attributes
        self._last_dist = 0
        self._grid_map = GridMap()

        # Initialize PID controller
        self.pid_controller = Pid()
        self.pid_controller.setpoint = 60
        self.pid_controller.output_data = 0
        self.pid_controller.set_tunings(3, 0, -200)
        self.pid_controller.set_sample_time(33)
        self.pid_controller.set_output_limits(-50, 50)
        self.pid_controller.set_mode(1)
        init_UARTs()

        # Read start values for X and Y
        self._y_start = self._median_sensor(10, Command.read_right_back_ir())+100
        self._x_start = self.START_X;
        
    def read_ir_side(self, side: Direction):
        """
        Returs the median of IR_MEDIAN_ITERATIONS reads of the specified ir sensor pair.

        Args:
            :side    (Direction): Which side to read.
        Returns:
            (int, int): Back and front sensor readings.
        """

        if side == Direction.RIGHT:
            ir_side_front = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_right_front_ir())
            ir_side_back = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_right_back_ir())
        else:
            ir_side_front = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_left_front_ir())
            ir_side_back = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_left_back_ir())
        return ir_side_back, ir_side_front

    def turn(self, direction : Direction, degrees : int, speed: int, save_new_angle=False):
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

        time.sleep(0.5)
        # Set time to zero to turn untill stopped
        standstill_rate = self._median_sensor(self.GYRO_MEDIAN_ITERATIONS, Command.read_gyro()) / 100
        turn_instr = Command.turn(direction, speed, 0)
        handle_command(turn_instr)

        while (abs(current_dir) < degrees):
            # Use a reimann sum to add up all the gyro rates
            # Area = TimeDiff * turnRate
            clk = time.time()
            turn_rate = self._median_sensor(self.GYRO_MEDIAN_ITERATIONS, Command.read_gyro()) / 100 - standstill_rate
            current_dir += (time.time() - clk) * turn_rate
            #logger.debug("Current dir: " + str(current_dir) + " -- Turn rate: " + str(turn_rate))

        # Turning is completed, stop the motors
        turn_instr = Command.stop_motors()
        handle_command(turn_instr)

        # Add turned degrees to current_angle if save_new_angle is true
        if save_new_angle:
            if direction == Direction.LEFT:
                self.current_angle -= degrees + 5
            else:
                self.current_angle += degrees + 5

    def get_position(self):
        """
        Calculates your x and y coordinates from path_trace.

        Returns:
            (Position): Contains the x and y position.
        """
        x = self._x_start
        y = self._y_start
        last_distance = 0

        for path in self.get_driven_dist():
            angle = path[0]
            if angle != 0:
                angle_sign = abs(angle) / angle
                abs_angle = abs(angle) % 360
            else:
                angle_sign = 1
                abs_angle = 0

            distance = path[1] - last_distance 

            if abs_angle == 0:
                y += distance
            elif abs_angle == 90:
                x += distance * angle_sign
            elif abs_angle == 180:
                y -= distance
            elif abs_angle == 270:
                x -= distance * angle_sign

            last_distance = path[1]

        return Position(x,y)

    
    def get_angle(self):
        """
        Return the angle in the intervall 0 <= angle <= 360.
        Returns:
            (int): The current angle in the given intervall above.
        """
        if (self.current_angle >= 0):
            return self.current_angle % 360
        elif (self.current_angle < 0):
            return 360 - abs(self.current_angle) % 360

    
    def update_map(self):
        """
        Update the gridmap with new scan data.
        """
        lines = movement_to_lines(self.get_driven_dist(), Position(self._x_start, self._y_start))
        cells = movement_lines_to_cells(lines, 1)
        # Scanning walls
        # unknown_in_view(location: Position, angle: int, g: GridMap, move_forward: int):
        #print(self.path_trace[0:max(len(self.path_trace)-1, 20)])
        self.logger.debug("Current cell" +str(cells[-1]))
        self.logger.debug("Current path" +str(self.get_driven_dist()))
        self.logger.debug("Current path: Uncalibrated" +str(self.path_trace))
        if (self.goal == Goal.MAP_OUTER_WALLS or self.goal == Goal.MAP_ISLAND) and len(cells) > 0:
            self.grid_map = GridMap()
            self._add_garage(cells[0])
            make_open(cells, self.grid_map)
            self.grid_map.debug_print()
            self.logger.info("---------")
            add_walls(cells, self.grid_map)
            self.logger.info("STOP")
            self.logger.info("start: " + str(cells[0]))
            self.logger.info("end: " + str(cells[-1]))
            if cells[-1] == cells[0] and len(cells) > 5:
                # We've returned to the garage
                self.goal = Goal.FIND_ISLAND
                self.logger.warning("In garage - looking for island")

        # Print grid
        old = self.grid_map.get(cells[-1].x, cells[-1].y)
        self.grid_map.set(cells[-1].x, cells[-1].y, CellType.LOCATION)
        self.grid_map.debug_print(print_origin = True)
        self.grid_map.set(cells[-1].x, cells[-1].y, old)
        self.logger.info("ANGLE: " + str(self.get_angle()))
        self.logger.info("POS: " + str(self.get_position()))
        
        if self.goal == Goal.FIND_ISLAND and cells:
            # We should no longer need to check if we need to scan, as long as we
            #   do this on every turn. left_island_exists does this implicitly.
            is_island, distance = island_exists(cells[-1], self.get_angle() - 90, self.grid_map)
            if is_island:
                self.logger.info("ISLAND!")
            else:
                self.logger.info("No island :(")
            if is_island and distance < self.DRIVE_TO_ISLAND_THRESHOLD:
                self.logger.info("DRIVE TO ISLAND!")
                self.drive_to_island()
            
        if cells and cells[-1] == cells[0] and self.goal == Goal.RETURN_HOME:
            # We're done!
            self.logger.info("================")
            self.logger.info(" D O N E")
            self.logger.info("================")
            self.goal = Goal.NONE


    def drive_distance(self, dist: int, speed: int, save_new_distance=False):
        """
        Drives the given distance at the given speed, if possible. Uses LIDAR for determining distance.

        Args:
            dist    (int): Distance in millimetres.
            speed   (int): Speed as a percentage value between 0-100.
        Returns:
            (bool): True if the given distance was driven, false if an obstacle stopped it.
        """
        reflex_right_start = handle_command(Command.read_reflex_right())
        reflex_right = reflex_right_start
        handle_command(Command.drive(1, speed, 0))
        # logger.debug("LIDAR INIT: ", lidar_init)
        while(reflex_right - reflex_right_start < dist):
            reflex_right = handle_command(Command.read_reflex_right())
            lidar_cur = handle_command(Command.read_front_ir())
            if lidar_cur < self.OBSTACLE_SAFETY_OVERRIDE:
                handle_command(Command.stop_motors())
                if save_new_distance:
                    self._save_position(reflex_right - reflex_right_start)
                return False
            # logger.debug("LIDAR CURRENT: ", lidar_cur)
        handle_command(Command.stop_motors())
        if save_new_distance:
            self._save_position(reflex_right - reflex_right_start)
        return True

    def follow_wall(self, distance: int, side="right"):
        """
        Follow the wall to the right until the robot encounters a corner or obstacle, or the given distance to drive is reached.

        Args:
            dist (int): Distance in millimetres.

        Returns:
            (DriveStatus): Status when completed.
        """

        if side == "right":
            # We're scanning for island on the left - turn it left
            handle_command(Command.servo(0))
        elif side == "left":
            # We're scanning for island on the right - turn it right
            handle_command(Command.servo(180))
        else:
            # Unreachable, unless caller is a moron
            # the argument should really be an enum.
            assert (False)

        self.logger.info("Side: " + side)

        # Sleep after servo turn
        #time.sleep(0.75)
        
        # Read start values from sensors
        reflex_right_start = handle_command(Command.read_reflex_right())
        reflex_right = handle_command(Command.read_reflex_right())
        reflex_right_start_2 = handle_command(Command.read_reflex_right())
        self._last_dist = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_right_front_ir())

        cells = 0
        # Drive until the wanted distance is reached
        while (reflex_right - reflex_right_start <= distance):
            if ((reflex_right - reflex_right_start) / 400) > cells:
                self._save_position(handle_command(Command.read_reflex_right()) - reflex_right_start_2)
                reflex_right_start += (handle_command(Command.read_reflex_right()) - reflex_right_start_2)
                reflex_right_start_2 = handle_command(Command.read_reflex_right())
                
                before = self.goal
                self.update_map()
                reflex_right_start += (handle_command(Command.read_reflex_right()) - reflex_right_start_2)
                if self.goal == Goal.FIND_ISLAND and before != self.goal:
                    return DriveStatus.DONE
                cells += 1
            # Get sensor values
            reflex_right = handle_command(Command.read_reflex_right())
            ir_front = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_front_ir())

            if side == "right":
                ir_side_back, ir_side_front = self.read_ir_side(Direction.RIGHT)
            else:
                ir_side_back, ir_side_front = self.read_ir_side(Direction.LEFT)

            # Detect corridor to the right
            if (ir_side_front >= self.TURN_OVERRIDE_DIST):
                self.logger.debug("IR - front: " + str(ir_side_front) + " IR - back: " + str(ir_side_back) + " list_dist: " + str(self._last_dist))
                # Save the given length driven
                self._save_position(reflex_right - reflex_right_start)
                self._last_dist = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_right_front_ir())
                if side == "right":
                    return DriveStatus.CORRIDOR_DETECTED_RIGHT
                else:
                    return DriveStatus.CORRIDOR_DETECTED_LEFT

            # Obstacle detected, and no turn to the right
            if(ir_front < self.OBSTACLE_DIST):
                # Save the given length driven
                handle_command(Command.stop_motors())
                self._save_position(reflex_right - reflex_right_start)
                return DriveStatus.OBSTACLE_DETECTED

            # We need to get the distance from the center of the robot perpendicular to the wall
            # This assumes that we have a wall on either side (i.e. we are in a corridor)
            # TODO: Different cases for corridors and open rooms.
            dist_side = (ir_side_front + ir_side_back) / 2
            angle_side = math.atan2(ir_side_back - ir_side_front, self.SENSOR_SPACING)
            perpendicular_dist_side = dist_side * math.cos(angle_side)
            # logger.debug("PERPENDICULAR DIST RIGHT:", perpendicular_dist_right)
            self.pid_controller.input_data = perpendicular_dist_side
            self.pid_controller.d_term = angle_side
            self.pid_controller.compute()
            self.pid_controller.output_data += 100
            self._follow_wall_help(self.pid_controller.output_data / 100, self.BASE_SPEED * min(max(ir_front,100, distance - (reflex_right - reflex_right_start)), 200) / 200, side)
            if side == "right":
                self._last_dist = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_right_front_ir())
            else:
                self._last_dist = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_left_front_ir())

        # Save the given length driven
        handle_command(Command.stop_motors())
        self._save_position(reflex_right - reflex_right_start)
        return DriveStatus.DONE

    def scan(self):
        """
        Scan the room at this position and return the recorded data.

        Returns:
            (list): Recorded data in a list of distances, where the index is the servo degree
        """
        recorded_data = []  # (Angle, distance)
        # Turn LIDAR to 0 degrees
        servo_instr = Command.servo(0)
        handle_command(servo_instr)

        # Sleep 1 second to wait for LIDAR to be in position
        time.sleep(1)

        for i in range(180):
            # Point LIDAR in the right angle
            servo_instr = Command.servo(i)
            handle_command(servo_instr)

            # Read data from the LIDAR sensor
            lidar_distance = handle_command(Command.read_lidar())
            recorded_data.append([i, lidar_distance])
            time.sleep(0.005)

        # Point LIDAR forward
        handle_command(Command.servo(180))

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
            ir_back, ir_front = self.read_ir_side(Direction.LEFT)
            angle = math.atan2(ir_back - ir_front, self.SENSOR_SPACING)
            self.turn(math.degrees(angle) > 0, abs(int(math.degrees(angle))), speed=self.BASE_SPEED+10)

        elif side == "right":
            ir_back, ir_front = self.read_ir_side(Direction.RIGHT)
            angle = math.atan2(ir_back - ir_front, self.SENSOR_SPACING)
            self.turn(math.degrees(angle) < 0, abs(int(math.degrees(angle))), speed=self.BASE_SPEED+10)

    def leave_island(self):
        """
        Turn left, drive to the wall and turn right 
        """
        self.turn(Direction.LEFT, 85, speed = self.ACCELERATED_SPEED, save_new_angle = True)
        self.drive_distance(99999, self.BASE_SPEED, save_new_distance = True)
        self.turn(Direction.RIGHT, 85, speed = self.ACCELERATED_SPEED, save_new_angle = True)
        self.goal = Goal.RETURN_HOME
            
    def drive_to_island(self):
        """
		Turn towards the island, drive to it and then turn left so that
		the robot will follow the right side of the wall.
        """
        handle_command(Command.stop_motors())
        time.sleep(0.5)
        self.turn(Direction.LEFT, 85, speed = self.ACCELERATED_SPEED, save_new_angle = True)
        self.drive_distance(99999, self.BASE_SPEED, save_new_distance = True)
        self.turn(Direction.LEFT, 85, speed = self.ACCELERATED_SPEED, save_new_angle = True)
        self.start_cell_at_island = approximate_to_cell(self.get_position())
        self.goal = Goal.MAP_ISLAND


    def _add_garage(self, where:Position):
        """
        Add a garage in the desired cell.
        """
        # Update grid_map to include garage
        self.grid_map.set(where.x - 1, where.y, CellType.WALL)
        self.grid_map.set(where.x, where.y + 1, CellType.WALL)
        self.grid_map.set(where.x, where.y - 1, CellType.WALL)
        self.grid_map.set(where.x, where.y, CellType.OPEN)

            

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
            values.append(handle_command(sensor_instr))
        if len(values) == 1:
            return values[0]
        else:
            return np.median(values)

    def _save_position(self, unsaved_distance   :   int):
        """
        Save the collected data to the trace list.

        Args:
            unsaved_distance   (int): Reflex data that have been driven but not added to self.driven_distance
        """
        print ("SAVING", unsaved_distance)
        self.driven_distance += unsaved_distance * 0.9195402298850575
        self.path_trace += [(self.current_angle, self.driven_distance)]

    def get_driven_dist(self):
        distances = self.path_trace
        last_angle = None
        new_list = []
        for angle, dist in distances:
            # print (angle, dist)
            if last_angle == angle:
                # print("merging")
                new_list[-1][1] = dist
            else:
                new_list.append([angle, dist])
            last_angle = angle

#        print("Merged distances", new_list)

        dist_list = [[0, 0]]

        for i in range(len(new_list)):
            angle, dist = new_list[i]
            dist_list.append([angle, dist_list[-1][1] + round(dist/400) * 400])
            # print (new_list)
            for j in range(i+1, len(new_list)):
                new_list[j][1] = new_list[j][1] - dist
            # print (new_list)

        # print (dist_list)
        dist_list = dist_list[1:]
        return dist_list




    def _follow_wall_help(self, ratio : int, base_speed : int, side="right"):
        """
        Calculate motor speeds and send drive instruction.

        Args:
            ratio       (int): Will drive forward if exactly 1, turn left if > 1, and turn right if < 1.
            base_speed  (int): Base speed that ratio will be applied to.
        """
        left_speed = max(min(base_speed / ratio, 100), 0)
        right_speed = max(min(base_speed * ratio, 100), 0)
        if side == "right":
            drive_instr = Command.side_speeds(1, round(left_speed), 1, round(right_speed))
        else:
            drive_instr = Command.side_speeds(1, round(right_speed), 1, round(left_speed))
        handle_command(drive_instr)

    def _is_moving(self, threshold = 30, wait_time = 0.2):
        """
        Checks for sensor changes above threshold within wait_time.
        Args:
            threshold   (int): How much the sensor values are allowed to change (mm)
            wait_time   (int): How long to wait between sensor reads
        Returns:
            (bool): If the robot has moved more than threshold.
        """
        ir_back = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_front_ir())
        lidar = self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_lidar())
        ir_right_back, ir_right_front = self.read_ir_side(Direction.RIGHT)
        ir_left_back, ir_left_front = self.read_ir_side(Direction.LEFT)

        time.sleep(wait_time)
        # TODO: Currently skipping check
        return False
        ir_back_diff = abs(ir_back - self._median_sensor(self.IR_MEDIAN_ITERATIONS, Command.read_front_ir()))
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
            # logger.debug("MOVING: threshold=", threshold, "mm, wait_time", wait_time)
            return True

def sensor_test(robot):
    """
    Continuously get all sensor values and print these.
    """
    while 1:
        ir_right_back, ir_right_front = robot.read_ir_side(Direction.RIGHT)
        ir_left_back, ir_left_front = robot.read_ir_side(Direction.LEFT)
        ir_back = robot._median_sensor(robot.IR_MEDIAN_ITERATIONS, Command.read_front_ir())
        lidar = robot._median_sensor(1, Command.read_lidar())
        robot.logger.debug("IR_RIGHT_BACK: " + str(ir_right_back) + " IR_RIGHT_FRONT : " + str(ir_right_front) + " LIDAR: " + str(lidar), "IR_LEFT_BACK", ir_left_back, "IR_LEFT_FRONT", ir_left_front, "IR_BACK", ir_back)

def victory_dance():
    while True:
        handle_command(Command.servo(0))
        time.sleep(0.75)
        handle_command(Command.servo(180))
        time.sleep(0.75)

def main(argv):
    # create logger
    logger = logging.getLogger()
    ch = logging.StreamHandler()

    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbosity", type = int, default = 1, choices = range(0, 3), help = "Verbosity level. 0 is lowest and 2 highest. Default 1.")
    args = parser.parse_args()

    verbosity = args.verbosity
    if (verbosity == 0):
        logger.disabled = True
        ch.disabled = True
    elif (verbosity == 1):
        logger.setLevel(logging.INFO)
        ch.setLevel(logging.INFO)
    elif (verbosity == 2):
        logger.setLevel(logging.DEBUG)
        ch.setLevel(logging.DEBUG)

    # create formatter
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')

    # add formatter to ch
    ch.setFormatter(formatter)

    # add ch to logger
    logger.addHandler(ch)

    robot = Robot(ControllerMode.MANUAL, logger)

    # Turn LIDAR to 90 degrees
    servo_instr = Command.servo(180)
    handle_command(servo_instr)

    # Wait fot LIDAR to be in position
    time.sleep(1)
#    sensor_test(robot)

    g = grid_map.GridMap()

    print (robot.get_position())
    #print("lidar", robot.scan())
    g.debug_print()
#    time.sleep(10)
    # Try driving in infinite loop around the maze
    while 1:
            robot.pid_controller.kp = 0
            robot.pid_controller.ki = 0
            robot.pid_controller.kd = 0
            robot.pid_controller.set_tunings(3, 0, -200)
            # Drive forward without crashing in wall
            if robot.goal == Goal.NONE:
                victory_dance()
                break
            if robot.goal == Goal.FIND_ISLAND:
                status = robot.follow_wall(99999999, side = "right")
            elif robot.goal == Goal.MAP_ISLAND:
                robot_position = robot.get_position()
                if (robot.start_cell_at_island == approximate_to_cell(robot_position) and robot.has_been_to_other_cell):
                    robot.leave_island()
                    robot.logger.info("RETURNING HOME!")
                    status = robot.follow_wall(999999, side = "left")
                elif (robot.start_cell_at_island != approximate_to_cell(robot_position)):
                    robot.has_been_to_other_cell = True
                    status = robot.follow_wall(999999, side = "right")
                else:
                    status = robot.follow_wall(999999, side = "right")
            elif robot.goal == Goal.RETURN_HOME:
                status = robot.follow_wall(9999999, side = "left")
            else:
                status = robot.follow_wall(9999999, side = "right")
            handle_command(Command.stop_motors())    
            logger.debug(robot.get_position())
            logger.debug(robot.get_driven_dist())
            logger.info("Explore island: " + str(robot.goal == Goal.MAP_ISLAND))
            
            robot.update_map()

            if (status == DriveStatus.OBSTACLE_DETECTED):
                logger.info("---------- OBSTACLE DETECTED! \n---------- TURNING LEFT 90 degrees")
                turn_instr = Command.stop_motors()
                handle_command(turn_instr)
                while robot._is_moving(): pass
                ir_right_front = robot._median_sensor(robot.IR_MEDIAN_ITERATIONS, Command.read_right_front_ir())
                ir_left_front = robot._median_sensor(robot.IR_MEDIAN_ITERATIONS, Command.read_left_front_ir())

                if ir_right_front > robot.TURN_OVERRIDE_DIST and ir_left_front > robot.TURN_OVERRIDE_DIST:
                    #robot.stand_perpendicular('left')
                    robot.turn(Direction.RIGHT, 85, speed = robot.ACCELERATED_SPEED, save_new_angle = True)
                    while robot._is_moving(): pass
                    #robot.stand_perpendicular('left')
                elif ir_right_front < robot.TURN_OVERRIDE_DIST and ir_left_front < robot.TURN_OVERRIDE_DIST:
                    #robot.stand_perpendicular('right')
                    robot.turn(Direction.LEFT, 85, speed = robot.ACCELERATED_SPEED, save_new_angle = True)
                    while robot._is_moving(): pass
                    #robot.stand_perpendicular('right')
                else:
                    if (ir_right_front > ir_left_front):
                        #robot.stand_perpendicular('left')
                        robot.turn(Direction.RIGHT, 85, speed = robot.ACCELERATED_SPEED, save_new_angle = True)
                        while robot._is_moving(): pass
                        #robot.stand_perpendicular('left')
                        
                    else:
                        #robot.stand_perpendicular('right')
                        robot.turn(Direction.LEFT, 85, speed = robot.ACCELERATED_SPEED, save_new_angle = True)
                        while robot._is_moving(): pass
                        #robot.stand_perpendicular('right')
                robot.pid_controller.kp = 0
                robot.pid_controller.ki = 0
                robot.pid_controller.kd = 0
                robot.pid_controller.set_tunings(3, 0, -200)
                #Test the map functionality
            elif (status == DriveStatus.CORRIDOR_DETECTED_RIGHT):
                logger.info("---------- DETECTED CORRIDOR TO RIGHT! \n---------- TURNING RIGHT 90 degrees")
                turn_instr = Command.stop_motors()
                handle_command(turn_instr)
                while robot._is_moving(): pass
                #robot.stand_perpendicular('left')
                robot.drive_distance(robot.CORRIDOR_TURN_ENTRY_DIST, robot.BASE_SPEED, save_new_distance = True)
                robot.turn(Direction.RIGHT, 85, speed = robot.ACCELERATED_SPEED, save_new_angle = True)
                while robot._is_moving(threshold = 30): pass

                # TODO: Detection works, but seems to commonly result in the robot standing staring at a wall, and obstacle detection.
                """if robot._median_sensor(robot.IR_MEDIAN_ITERATIONS, Command.read_front_ir()) < 200:
                    logger.info("Not a corridor, moving back")
                    robot.turn(Direction.LEFT, 85, speed = robot.ACCELERATED_SPEED, save_new_angle = True)
                    robot.stand_perpendicular('right')
                    while robot._is_moving(threshold = 30): pass
                else:"""
                    # TODO: Find a way to do stand_perpendicular when there is no wall to the left. Or lower CORRIDOR_TURN_EXIT_DIST again.
                    # As it is now, a right turn into a single square corridor does not work well.
                robot.drive_distance(robot.CORRIDOR_TURN_EXIT_DIST, robot.BASE_SPEED, save_new_distance = True)
                while robot._is_moving(): pass
                    #robot.stand_perpendicular('right')
                    #robot.stand_perpendicular('left')
                robot.pid_controller.kp = 0
                robot.pid_controller.ki = 0
                robot.pid_controller.kd = 0
                robot.pid_controller.set_tunings(3, 0, -200)
            elif (status == DriveStatus.CORRIDOR_DETECTED_LEFT):
                logger.info("---------- DETECTED CORRIDOR TO LEFT! \n---------- TURNING LEFT 90 degrees")
                turn_instr = Command.stop_motors()
                handle_command(turn_instr)
                while robot._is_moving(): pass
                robot.drive_distance(robot.CORRIDOR_TURN_ENTRY_DIST, robot.BASE_SPEED, save_new_distance = True)
                robot.turn(Direction.LEFT, 85, speed = robot.ACCELERATED_SPEED, save_new_angle = True)
                while robot._is_moving(threshold = 30): pass

                # TODO: Detection works, but seems to commonly result in the robot standing staring at a wall, and obstacle detection.
                if robot._median_sensor(robot.IR_MEDIAN_ITERATIONS, Command.read_front_ir()) < 150:
                    logger.info("Not a corridor, moving back")
                    robot.turn(Direction.RIGHT, 85, speed = robot.ACCELERATED_SPEED, save_new_angle = True)
                    robot.stand_perpendicular('left')
                    while robot._is_moving(threshold = 30): pass
                else:
                    # TODO: Find a way to do stand_perpendicular when there is no wall to the left. Or lower CORRIDOR_TURN_EXIT_DIST again.
                    # As it is now, a right turn into a single square corridor does not work well.
                    robot.drive_distance(robot.CORRIDOR_TURN_EXIT_DIST, robot.BASE_SPEED, save_new_distance = True)
                    while robot._is_moving(): pass
                    #robot.stand_perpendicular('left')
                    #robot.stand_perpendicular('right')
                robot.pid_controller.kp = 0
                robot.pid_controller.ki = 0
                robot.pid_controller.kd = 0
                robot.pid_controller.set_tunings(3, 0, -200)
            robot._grid_map.debug_print()

if __name__ == "__main__":
    main(sys.argv[1:])
