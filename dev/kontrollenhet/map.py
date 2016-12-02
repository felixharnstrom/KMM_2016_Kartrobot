
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

"""The length of a line segment."""
ACCURACY = GRID_SIZE//POINTS


def coordinates_to_lines(coordinates, robot_pos:Position, grid_map:GridMap):
    """
    Approximate coordinates to line segments.
    :param coordinates: A list of tuples of floats, containing real-world (x, y) coordinates.
    :param robot_pos: The position of the robot
    :param grid_map: The GridMap to append walls to.
    :return: A list of Line's, each being horizontal or vertical with lengths of GRID_SIZE millimeters.
    """
    # Gets size coordinate area in squares of GRID_SIZE
    top_left = top_left_grid_index(coordinates)
    bottom_right = bottom_right_grid_index(coordinates)

    # Get votes
    horizontal_votes = get_votes_for_horizontal_line_segments(coordinates)
    vertical_votes = get_votes_for_vertical_line_segments(coordinates)

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

            grid_changed = False

            # Check if enough line segments making up a line have enough votes
            # If they do, we have detected a full line
            if horizontal_line_segs >= POINTS_LINE:
                start = Position(x, y)
                end = Position(x_next, y)
                line = Line(start, end)
                if line not in lines:
                    lines.append(line)
                    grid_changed = change_grid_type(robot_pos.x, robot_pos.y, x, y, line, grid_map)              
            if vertical_line_segs >= POINTS_LINE:
                start = Position(x, y)
                end = Position(x, y_next)
                line = Line(start, end)
                if line not in lines:
                    lines.append(line)
                    grid_changed = change_grid_type(robot_pos.x, robot_pos.y, x, y, line, grid_map)

        if grid_changed:
            change_grid_type(robot_pos.x, robot_pos.y, x, y, (), grid_map)

    # Debug plot
    #debug_plot(lines, coordinates, top_left.x, bottom_right.x, top_left.y, bottom_right.y)
    return lines

    


def plot_lines(lines):
    """
    Plot axis-aligned lines
    :param lines: A list of axis-aligned lines
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


def check_available_grid(x_min, x_max, y_min, y_max, grid_map):
    """
    Checks all available arches between nodes the robot can go
    :param map: A list with tupleres containing all walls
    :return: A list of all posible grid -> grid [(fron_x1, from_y1, to_x1, to_y1), (from_x2, from_y2, ...), ...]
    """
    possible_squares = []
    for y in range(y_min, y_max - 1):
        y_next = y + 1
        for x in range(x_min, x_max - 1):
            x_next = x + 1
            if grid_map.get(x, y) == CellType.OPEN:
                if grid_map.get(x + 1, y) == CellType.OPEN:
                    possible_squares.append((x, y, x_next, y))
                if grid_map.get(x, y + 1) == CellType.OPEN:
                    possible_squares.append((x, y, x, y_next))
    return possible_squares




def top_left_grid_index(coordinates):
    """Return the top_left coordinates of an AABB enclosing all coordinates and origin,
    scaled by 1/GRID_SIZE."""
    top_left = Position(0, 0)
    for x, y in coordinates:
        top_left.x = min(top_left.x, int(x//GRID_SIZE))
        top_left.y = min(top_left.y, int(y//GRID_SIZE))
    return top_left

def bottom_right_grid_index(coordinates):
    """Return the bottom_right coordinates of an AABB enclosing all coordinates and origin,
    scaled by 1/GRID_SIZE."""
    bottom_right = Position(0, 0)
    for x, y in coordinates:
        bottom_right.x = max(bottom_right.x, int(x//GRID_SIZE + 1))
        bottom_right.y = max(bottom_right.y, int(y//GRID_SIZE + 1))
    return bottom_right


def get_votes_for_axis_aligned_line_segments(coordinates, vertical:bool):
    """
    For each possible line segment approximate each coordinate to the closest segment, and return the
    number of coordinates approximated to fall in each line segment (the number of "votes"). A line segment 
    is considered a line with a length of ACCURACY, while a full line has a length of GRID_SIZE.

    Vertical determines if we attempt to approximate coordinates to vertical lines (True) or horizontal
    ones (False).

    :param coordinates: A list of tuples of floats, containing real-world (x, y) coordinates.
    :param vertical: What sort of lines to approximate to. True if vertical, False if horizontal.

    :return: a 3-dimensional list.
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
    top_left = top_left_grid_index(coordinates)
    bottom_right = bottom_right_grid_index(coordinates)
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


def get_votes_for_horizontal_line_segments(coordinates):
    """
    Performs get_votes_for_axis_aligned_line_segments(...), returning the votes for
    all horizontal line segments.

    See get_votes_for_axis_aligned_line_segments(...) for more information regarding return value.
    """
    return get_votes_for_axis_aligned_line_segments(coordinates, False)



def get_votes_for_vertical_line_segments(coordinates):
    """
    Performs get_votes_for_axis_aligned_line_segments(...), returning the votes for
    all vertical line segments.

    See get_votes_for_axis_aligned_line_segments(...) for more information regarding return value.
    """
    return get_votes_for_axis_aligned_line_segments(coordinates, True)



def convert_to_coordinates(measurements, x_position, y_position, angle):
    """
    Convert angle and distances to coordinates via polar projection
    :param measurements: A list of tuples containing (angle, distance), where angle is degrees
    :param x_position: The x-position of the robot
    :param y_position: The y-position of the robot
    :angle: The facing angle of the robot, in degrees
    :return: A list of coordinates - tuples containing (x, y)
    """
    coordinates = []
    for degree, dist in measurements:
        x = (math.sin(math.radians(degree + angle)) * dist) + x_position
        y = (math.cos(math.radians(degree + angle)) * dist) + y_position
        coordinates.append((x, y))
    return coordinates



def read_debug_data(file_name):
    """
    Reads old measurements that have been saved to a json file
    """
    with open(file_name) as data_file:
        data = json.load(data_file)
    return data



def measure_lidar():
    """
    Turns servo 180 degrees while scanning and converts the scanned points to cordinates.
    :return: Measurements in [[degree1, dist1],[degree2, dist2], ...]
    """
    uart = UART("ttyUSB0")
    sensorunit = UART("ttyUSB1")
    driveInstruction = Servo(0)

    degree_plot = []
    distance_plot = []

    uart.send_function(driveInstruction)

    time.sleep(1.5)
    degree = 0

    measurements = []

    for degree in range(0, 180):
        sensorunit.send_function(ReadLidar())

        if sensorunit.decode_metapacket(sensorunit.receive_packet())[2] != 0:
            raise Exception("Incorrect acknowledge packet")
        if sensorunit.decode_metapacket(sensorunit.receive_packet())[2] != 8:
            raise Exception("Not a lidar value")

        highest = sensorunit.receive_packet()
        lowest = sensorunit.receive_packet()
        dist = ord(lowest) + ord(highest) * (2 ** 8)

        x = math.sin(math.radians(degree)) * dist
        y = math.cos(math.radians(degree)) * dist

        measurements.append([degree, dist])

        uart.send_function(Servo(int(degree)))
        time.sleep(0.005)
    return measurements



def debug_plot(map, coordinates, x_min, x_max, y_min, y_max):
    """
    A test plotting, that shows all mesuring points and walls on the same plot
    """
    plot_lines(map)
    x_plot = []
    y_plot = []
    for x, y in coordinates:
        x_plot.append(x)
        y_plot.append(y)
    plt.plot(x_plot, y_plot, '.')
    plt.plot([(x_min - 1) * GRID_SIZE, (x_max + 1) * GRID_SIZE], [(y_min) * GRID_SIZE, (y_max) * GRID_SIZE], '.')
    plt.show()


def change_grid_type(robot_x, robot_y, x, y, line, grid_map):
    """Changes the grid type to wall if it is a wall"""
    x_next = x + 1
    x_prev = x - 1
    y_next = y + 1
    y_prev = y - 1

    next_start = Position(x*GRID_SIZE, y*GRID_SIZE)
    next_vertical_end = Position(x_next*GRID_SIZE, y*GRID_SIZE)
    next_horizontal_end = Position(x*GRID_SIZE, y_next*GRID_SIZE)
    next_vertical = Line(next_start, next_vertical_end)
    next_horizontal = Line(next_start, next_horizontal_end)
    
    if line == next_vertical:
        if y > robot_y:
            grid_map.set(x, y_next, CellType.WALL)
        elif y < robot_y:
            grid_map.set(x, y_prev, CellType.WALL)

    if line == next_horizontal:
        if x > robot_x:
            grid_map.set(x, y, CellType.WALL)
            if not (grid_map.get(x, y) == CellType.WALL):
                grid_map.set(x, y, CellType.OPEN)
        elif x < robot_x:
            grid_map.set(x_prev, y, CellType.WALL)

    if not (grid_map.get(x, y) == CellType.WALL):
        grid_map.set(x, y, CellType.OPEN)
    return True

def get_grid_map(robot_x, robot_y, lines):
    """Returns a GridMap containing the room as we currently know it."""
    min_x, max_x, min_y, max_y = get_size(lines)

    rslt = GridMap()

    rslt.set_robot_pos(robot_x, robot_y)

    for y in range (min_y, max_y):
        for x in range (min_x, max_x):
            x_next = x + 1
            x_prev = x - 1
            y_next = y + 1
            y_prev = y - 1

            next_start = Position(x*GRID_SIZE, y*GRID_SIZE)
            next_vertical_end = Position(x_next*GRID_SIZE, y*GRID_SIZE)
            next_horizontal_end = Position(x*GRID_SIZE, y_next*GRID_SIZE)
            next_vertical = Line(next_start, next_vertical_end)
            next_horizontal = Line(next_start, next_horizontal_end)

            if next_vertical in lines:
                if y > robot_y:
                    print(x_next, y, "Wall")
                    rslt.set(x, y_next, CellType.WALL)
                elif y < robot_y:
                    print(x, y_prev, "Wall")
                    rslt.set(x, y_prev, CellType.WALL)

            if next_horizontal in lines:
                if x > robot_x:
                    print(x, y, "Wall")
                    rslt.set(x, y, CellType.WALL)
                    if not (rslt.get(x, y) == CellType.WALL):
                        print(x, y, "Open")
                        rslt.set(x, y, CellType.OPEN)
                elif x < robot_x:
                    print(x_prev, y, "Wall")
                    rslt.set(x_prev, y, CellType.WALL)
            ## TODO: check why it is not setting the cell type to anything
            if not (rslt.get(x, y) == CellType.WALL):
                print(x, y, "Open")
                rslt.set(x, y, CellType.OPEN)
    return rslt

