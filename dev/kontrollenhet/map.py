
from UART import UART
from modules import *
from sensorenhet_functions import *
from geometry import *
import json
import math, time
import logging
import numpy as np
import matplotlib.pyplot as plt
from grid_map import GridMap, CellType

logging.getLogger(__name__).setLevel(logging.INFO)


"""The real-world size of a grid cell, in millimeters. Also the length of a line."""
GRID_SIZE = 400

"""The number of 'line segments per line."""
POINTS = 4

"""The number of votes per line section required for something to be considered a full line."""
MIN_MESURE = 1

"""The number of line sections that needs to be voted in for their line to be voted in."""
POINTS_LINE = 3

"""The thickness of a line segment."""
ACCURACY = 130

"""The length of a line segment."""
LINE_SEG_LENGTH = GRID_SIZE // POINTS



def coordinates_to_lines(coordinates):
    """
    Approximate coordinates to line segments.

    Args:
        :param coordinates (List of length 2 tuple of floats): Measurement endpoints formatted as (x, y).

    Returns:
        :return (list of Line's): Lines describing walls, each being horizontal or vertical with lengths of GRID_SIZE millimeters.
    """
    # Gets size coordinate area in squares of GRID_SIZE
    top_left = top_left_grid_index(coordinates)
    bottom_right = bottom_right_grid_index(coordinates)
    # Get votes
    horizontal_votes = get_votes_for_horizontal_line_segments(coordinates, top_left, bottom_right)
    vertical_votes = get_votes_for_vertical_line_segments(coordinates, top_left, bottom_right)

    #grid_map.expand_to_fit(bottom_right.x-top_left.x, bottom_right.y-top_left.y)

    # Our return value
    lines = []

    # Loop over y-indices for the grid
    for y_index in range(top_left.y, bottom_right.y + 1):
        # Stores square position, start in y and end in y.
        y = GRID_SIZE * y_index
        y_next = y + GRID_SIZE
        
        # Loop over x-indices for the grid
        for x_index in range(top_left.x, bottom_right.x + 1):
            # Stores square position, start in x and end in x.
            x = GRID_SIZE * x_index
            x_next = x + GRID_SIZE

            # correct that positions values can be negative, but lists don't have neg values
            pos = Position(x_index - top_left.x, y_index - top_left.y)

            # The number of line segments with enough votes to be considered part of a full line
            horizontal_line_segs = 0
            vertical_line_segs = 0

            # Count how many line segments have "enough" votes
            for i in horizontal_votes[pos.y][pos.x]:
                if i >= MIN_MESURE:
                    horizontal_line_segs += 1
            for j in vertical_votes[pos.y][pos.x]:
                if j >= MIN_MESURE:
                    vertical_line_segs += 1

            # Check if enough line segments making up a line have enough votes
            # If they do, we have detected a full line
            if horizontal_line_segs >= POINTS_LINE:
                start = Position(x, y)
                end = Position(x_next, y)
                line = Line(start, end)
                if line not in lines:
                    lines.append(line)

            if vertical_line_segs >= POINTS_LINE:
                start = Position(x, y)
                end = Position(x, y_next)
                line = Line(start, end)
                if line not in lines:
                    lines.append(line)

    return lines

# TODO: Coordinates are only used for min/max index
# TODO: Non-DRY
def get_grid_map(coordinates, lines, robot_pos:Position, grid_map:GridMap):
    """
    Gives every grid a CellType, as OPEN or WALL. OPEN if the grid is open from robot point of wiev adn WALL if the grid is behind a wall from robot point of view.

    Args:
        :param coordinates (List of length 2 tuple of float): Measurement endpoints formatted as (x, y).
        :param (list of Line): Lines describing walls, each being horizontal or vertical with lengths of GRID_SIZE millimeters.
        :param robot_pos (Position): The position of the robot as the measurements was taken.
        :param grid_map (GridMap): The GridMap to modify.
    """
    # Gets size coordinate area in squares of GRID_SIZE
    top_left = top_left_grid_index(coordinates)
    bottom_right = bottom_right_grid_index(coordinates)

    # Loop over y-indices for the grid
    for y_index in range(top_left.y, bottom_right.y + 1):
        # Stores square position, start in y and end in y as multiplied with GRID_SIZE.
        y = GRID_SIZE * y_index
        y_next = y + GRID_SIZE

        # Loop over x-indices for the grid
        for x_index in range(top_left.x, bottom_right.x + 1):
            # Stores square position, start in x and end in x as multiplied with GRID_SIZE.
            x = GRID_SIZE * x_index
            x_next = x + GRID_SIZE

            grid_changed = False

            #Position in grid coordinates
            grid_pos = Position(x_index, y_index)
            # Creates a line with current grid, so that it can check if this line actually exists
            start = Position(x, y)
            end_horizontal = Position(x_next, y)
            end_vertical = Position(x, y_next)
            line_horizontal = Line(start, end_horizontal)
            line_vertical = Line(start, end_vertical)

            #Check if there exists a horizontal line from current grid, if it exists create a WALL grid
            if line_horizontal in lines:
                change_grid_type(robot_pos, grid_pos, bottom_right, top_left, line_horizontal, grid_map)
                grid_changed = True

            # Check if there exists a vertical line from current grid, if it exists create a WALL grid
            if line_vertical in lines:
                change_grid_type(robot_pos, grid_pos, bottom_right, top_left, line_vertical, grid_map)
                grid_changed = True

            # if there has not been created a WALL grid from current grid, then make it OPEN
            if not grid_changed:
                # creating a line that never will exists, so that just this grid position can be mae open
                no_line = Line(Position(0, 0), Position(0, 0))
                change_grid_type(robot_pos, grid_pos, bottom_right, top_left, no_line, grid_map)



def plot_lines(lines):
    """
    Plot axis-aligned lines, and return the plot.

    Args:
        :param lines (list of Line): Axis axis-aligned lines to plot.

    Returns:
        :return (matplotlib): The plot.
    """
    for line in lines:
        start = line.start
        end = line.end
        # Horizontal lines
        if start.x == end.x:
            plot_line = np.linspace(start.y, end.y, POINTS)
            plt.plot([start.x] * POINTS, plot_line)
        # Vertical lines
        elif start.y == end.y:
            plot_line = np.linspace(start.x, end.x, POINTS)
            plt.plot(plot_line, [start.y] * POINTS)
    return plt


def top_left_grid_index(coordinates):
    """
    Return the top_left coordinates of an AABB enclosing all coordinates and origin, scaled by 1/GRID_SIZE.
    
    Args:
        :param coordinates (List of length 2 tuple of float): Measurement endpoints formatted as (x, y).

    Returns:
        :return (Position): The top left position of said AABB.
    """
    top_left = Position(0, 0)
    for x, y in coordinates:
        top_left.x = min(top_left.x, int(x//GRID_SIZE))
        top_left.y = min(top_left.y, int(y//GRID_SIZE))
    return top_left

def bottom_right_grid_index(coordinates):
    """
    Return the bottom_left coordinates of an AABB enclosing all coordinates and origin, scaled by 1/GRID_SIZE.
    
    Args:
        :param coordinates (List of length 2 tuple of float): Measurement endpoints formatted as (x, y).

    Returns:
        :return (Position): The bottom left position of said AABB, scaled by 1/GRID_SIZE.
    """
    bottom_right = Position(0, 0)
    for x, y in coordinates:
        bottom_right.x = max(bottom_right.x, int(x//GRID_SIZE + 1))
        bottom_right.y = max(bottom_right.y, int(y//GRID_SIZE + 1))
    return bottom_right



def get_votes_for_axis_aligned_line_segments(coordinates, top_left, bottom_right, vertical:bool):
    """
    For each possible line segment approximate each coordinate to the closest segment, and return the
    number of coordinates approximated to fall in each line segment (the number of "votes"). A line segment 
    is considered a line with a length of ACCURACY, while a full line has a length of GRID_SIZE.

    Vertical determines if we attempt to approximate coordinates to vertical lines (True) or horizontal
    ones (False).

    Args:
        :param coordinates: A list of tuples of floats, containing real-world (x, y) coordinates.
        :param vertical: What sort of lines to approximate to. True if vertical, False if horizontal.

    Returns:
        :return (list of int): a 3-dimensional list.
        The first two indices indicates the starting point of a line, divided by GRID_SIZE. The third an offset
        off n*ACCURACY, where n is the index (0 =< n < POINTS), on the x-axis or y-axis depending on if the
        line is vertical or horizontal. The lines end-point will thus be offset by an 
        additional (n+1)*ACCURACY. On that index we'll find the number of coordinates approximated to
        fall on that line.

        To be more specific, vertical being True or False determines if the last index causes us to move
        along the x-axis (vertical=False) or the y-axis (vertical=True).

        examples:
        assuming not vertical, [0][0][0] contains votes for the line (0,0) -> (ACCURACY, 0)
        assuming vertical,     [0][0][0] contains votes for the line (0,0) -> (0, ACCURACY)
        assuming not vertical, [0][0][2] contains votes for the line (2*ACCURACY, 0) -> (3*ACCURACY, 0)
        assuming not vertical, [1][0][0] contains votes for the line (1*GRID_SIZE, 0) -> (1*GRID_SIZE + 1*ACCURACY, 0)
    """
    # The size of the AABB enclosing all coordinates
    size = bottom_right.difference(top_left)
    size.add(Size(1, 1))
    # Array instead of list because list doesn't work as intended
    # 3-dimensional
    votes = np.array([[[0]*POINTS]*size.w]*size.h)

    for x, y in coordinates:
        # Approximate position to grid
        # Approximate the pair of coordinates to the grid
        pos_x = int(x//GRID_SIZE - top_left.x)
        pos_y = int(y//GRID_SIZE - top_left.y)
        pos = Position(pos_x, pos_y)
        
        # 
        pos_dist = Position(x%GRID_SIZE, y%GRID_SIZE)
        relevant_dist = pos_dist.x if vertical else pos_dist.y

        #
        factor = int(GRID_SIZE/POINTS_LINE)
        pos_loc = Position(pos_dist.x//factor, pos_dist.y//factor)
        relevant_loc = pos_loc.y if vertical else pos_loc.x
        
        # Check if measurement is within +- ACCURACY from line
        if relevant_dist < ACCURACY:
            votes[pos.y][pos.x][relevant_loc] += 1
        elif relevant_dist > (GRID_SIZE - ACCURACY):
            if vertical:
                votes[pos.y][pos.x+1][relevant_loc] += 1
            else:
                votes[pos.y+1][pos.x][relevant_loc] += 1
    return votes



def get_votes_for_horizontal_line_segments(coordinates, top_left, bottom_right):
    """
    Performs get_votes_for_axis_aligned_line_segments(...), returning the votes for
    all horizontal line segments.

    See get_votes_for_axis_aligned_line_segments(...) for more information regarding return value.
    """
    return get_votes_for_axis_aligned_line_segments(coordinates, top_left, bottom_right, False)

def get_votes_for_vertical_line_segments(coordinates, top_left, bottom_right):
    """
    Performs get_votes_for_axis_aligned_line_segments(...), returning the votes for
    all vertical line segments.

    See get_votes_for_axis_aligned_line_segments(...) for more information regarding return value.
    """
    return get_votes_for_axis_aligned_line_segments(coordinates, top_left, bottom_right, True)



def change_grid_type(robot_pos:Position, grid_pos:Position, bottom_right:Position, top_left:Position, line:Line, grid_map:GridMap):
    """
    Updates a grid cell to CellType.WALL if it falls on the given wall line, CellType.OPEN if it's within the scanned area, or leaves it unchanged otherwise.

    Args:
        :param robot_pos (Position): The position of the robot as the line was scanned.
        :param grid_pos (Position): The grid cell to check.
        :param bottom_right (Position): The bottom-right corner of the AABB enclosing the scanned area.
        :param top_left (Position): The top-left corner of the AABB enclosing the scanned area.
        :param grid_map (GridMap): The GridMap to modify.
    """
    next_grid_pos = Position(grid_pos.x + 1, grid_pos.y + 1)
    prev_grid_pos = Position(grid_pos.x - 1, grid_pos.y - 1)

    #create a line based on current grid position
    next_start = Position(grid_pos.x * GRID_SIZE, grid_pos.y * GRID_SIZE)
    next_vertical_end = Position(grid_pos.x * GRID_SIZE, next_grid_pos.y * GRID_SIZE)
    next_horizontal_end = Position(next_grid_pos.x * GRID_SIZE, grid_pos.y * GRID_SIZE)
    next_vertical = Line(next_start, next_vertical_end)
    next_horizontal = Line(next_start, next_horizontal_end)

    #Check if there is a line at current grid position. It also checks on e´what side of teh line teh robot is at, so it knows with grid should be a WALL
    if line == next_vertical:
        if grid_pos.x > robot_pos.x:
            grid_map.set(grid_pos.x, grid_pos.y, CellType.WALL)
        elif grid_pos.x < robot_pos.x:
            grid_map.set(prev_grid_pos.x, grid_pos.y, CellType.WALL)
    if line == next_horizontal:
        if grid_pos.y > robot_pos.y:
            grid_map.set(grid_pos.x, grid_pos.y, CellType.WALL)
        elif grid_pos.y < robot_pos.y:
            grid_map.set(grid_pos.x, prev_grid_pos.y, CellType.WALL)

    # if current grid is not a WALL and it is withing scanned area it will be made to CellType OPEN
    if not (grid_map.get(grid_pos.x, grid_pos.y) == CellType.WALL) and \
                    grid_pos.x < bottom_right.x and grid_pos.y < (bottom_right.y - 1) and \
                    grid_pos.x >= top_left.x and grid_pos.y > top_left.y:
        grid_map.set(grid_pos.x, grid_pos.y, CellType.OPEN)


def convert_to_coordinates(measurements, robot_pos:Position, angle):
    """
    Convert angle and distances to coordinates via polar projection.
    
    Args:
        :param measurements (list of length-2-tuple of float: Tuples containing (angle, distance to wall), where angle is degrees.
        :param robot_pos (Position): The robot position; origin point of the measurements.
        :angle (float): The angle of the robot, in degrees.
    
    Returns:
        :return (List of length 2 tuple of floats): Measurement endpoints formatted as (x, y).
    """
    coordinates = []
    for degree, dist in measurements:
        x = (math.sin(math.radians(degree + angle)) * dist) + robot_pos.x
        y = (math.cos(math.radians(degree + angle)) * dist) + robot_pos.y
        coordinates.append((x, y))
    return coordinates



def read_debug_data(file_name):
    """
    Reads measurement data from json.

    Args:
        :param file_name (str): file name of file to read from.

    Returns:
        :return (List of length 2 tuple of floats): Measurement endpoints formatted as (x, y).
    """
    with open(file_name) as data_file:
        data = json.load(data_file)
    return data



def measure_lidar(motor_uart:UART, sensor_uart:UART):
    """
    Turn the laser 180 degrees, taking measurements. Return said measurements.

    Args:
        :param motor_uart (UART): The motor unit UART interface.
        :param sensor_uart (UART): The sensor unit UART interface.

    Returns:
        :return: (list of length-2-tuple of float: Tuples containing (angle, distance to wall), where angle is degrees.
    """
    driveInstruction = Servo(0)

    degree_plot = []
    distance_plot = []

    motor_uart.send_function(driveInstruction)

    time.sleep(1.5)
    degree = 0

    measurements = []

    for degree in range(0, 180):
        sensor_uart.send_function(ReadLidar())

        if sensor_uart.decode_metapacket(sensor_uart.receive_packet())[2] != 0:
            raise RuntimeError("Incorrect acknowledge packet.")
        if sensor_uart.decode_metapacket(sensor_uart.receive_packet())[2] != 8:
            raise RuntimeError("Not a lidar value.")

        highest = sensor_uart.receive_packet()
        lowest = sensor_uart.receive_packet()
        dist = ord(lowest) + ord(highest) * (2 ** 8)

        x = math.sin(math.radians(degree)) * dist
        y = math.cos(math.radians(degree)) * dist

        measurements.append([degree, dist])

        motor_uart.send_function(Servo(int(degree)))
        time.sleep(0.005)
    return measurements



def debug_plot(coordinates, lines):
    """
    A test plotting, that shows all mesuring points and walls on the same plot. This is for debugging purpesus only.
    This returns a plot with a dot for all mesured data and also draws out all lines/Walls
    """
    plot_lines(lines)
    top_left = top_left_grid_index(coordinates)
    bottom_right = bottom_right_grid_index(coordinates)
    x_plot = []
    y_plot = []
    for x, y in coordinates:
        x_plot.append(x)
        y_plot.append(y)
    plt.plot(x_plot, y_plot, '.')
    plt.plot([(top_left.x - 1) * GRID_SIZE, (bottom_right.x + 1) * GRID_SIZE],
             [(top_left.y) * GRID_SIZE, (bottom_right.y) * GRID_SIZE], '.')
    plt.show()
