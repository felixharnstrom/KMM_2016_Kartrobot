
from UART import UART
from modules import *
from sensorenhet_functions import *
import json
import math, time
import logging
import numpy as np
import matplotlib.pyplot as plt
from grid_map import GridMap, CellType



logging.getLogger(__name__).setLevel(logging.INFO)


grid = []

# Constants for how accurate this system should be
GRID_SIZE = 400     # What si the Grid size
POINTS = 4          # Every line of GRID_SIZE, in how many parts should it be devided
MIN_MESURE = 1      # How many messurments should be in every like section
POINTS_LINE = 3     # How many Linesections should have messures to be a line
ACCURACY = 100      # How far from line is it ok for mesurments to be


def map_room(robot_x, robot_y, angle, grid_map):
    """
    This definition plots the room
    :param robot: A list with robot position and angle [x, y, angle]
    :param map: Pevious mapping of the room
    :return: Returns new_lines, that is a list of tupels with line information [(x11, y11, x21, y21), (x12, y12...), ...]
    """
    #coordinates = get_coordinates(measure_lidar(), x_position, y_position, angle)

    #coordinates = get_coordinates(read_debug_data('demo_data/perfect_square_center_raw_data.json'), robot_x, robot_y, angle)
    coordinates = read_debug_data('demo_data/triple_sided_wall_with_imperfections.json')

    # Gets size coordinate area in squares of GRID_SIZE
    x_min, x_max, y_min, y_max = get_size(coordinates)

    # Gets lists contaning all mesurements that  could be on horizontal lines and vertical lines
    horizontal_lines = get_horizontal_lines(coordinates, x_min, x_max, y_min, y_max)
    vertical_lines = get_vertical_lines(coordinates, x_min, x_max, y_min, y_max)

    new_lines = []

    # Loops over the entire area based on squares of GRID_SIZE
    # + 1 needed so that it always can call y_next, even in last grid
    for n in range(y_min, y_max + 1):
        # Stores square position, start in y and end in y.
        y = GRID_SIZE * n
        y_next = GRID_SIZE * (n + 1)
        y_pos = n - y_min        # corrects that position values can be negative, but lists don't have neg values

        # + 1 needed so that it always can call x_next, even in last grid
        for m in range(x_min, x_max + 1):
            # Stores square position, start in x and end in x.
            x = GRID_SIZE * m
            x_next = GRID_SIZE * (m + 1)
            x_pos = m - x_min     # corrects that position values can be negative, but lists don't have neg values

            # Temp values for ammount of  Line sections with mesures
            tmp_horizontal = 0
            tmp_vertical = 0

            # Counts how many Lines sections have >= than MIN_MESURE
            for i in horizontal_lines[y_pos][x_pos]:
                if i >= MIN_MESURE:
                    tmp_horizontal += 1
            for j in vertical_lines[y_pos][x_pos]:
                if j >= MIN_MESURE:
                    tmp_vertical += 1

            grid_changed = False

            # Checks if ammount of Lines sections are >= than POINTS_LINE
            if tmp_horizontal >= POINTS_LINE:
                line = (x, y, x_next, y)            # create a tupel with line start and end point
                if not (grid_map.get_relative(x, y) == CellType.WALL):
                    new_lines.append(line)
                    grid_changed = change_grid_type(robot_x, robot_y, x, y, line, grid_map)

            if tmp_vertical >= POINTS_LINE:
                line = (x, y, x, y_next)            # create a tupel with line start and end point
                if not (grid_map.get_relative(x, y) == CellType.WALL):
                    new_lines.append(line)
                    grid_changed = change_grid_type(robot_x, robot_y, x, y, line, grid_map)
        if grid_changed:
            change_grid_type(robot_x, robot_y, x, y, (), grid_map)

        print("CELLTYPE", grid_map.get_relative(x, y), x, y)
        print("CELLTYPE S", grid_map.get_relative(x + 1, y), x + 1, y)
        print("CELLTYPE T", grid_map.get_relative(x, y + 1), x, y + 1)
    return new_lines

    


def plot_room(lines):
    """
    Plots the room and return the plot
    :param map: Is a list of tupels that has the information about walls [(x11, y11, x21, y21), (x12, y12, ...), ...]
    :return: A list of coordinates
    """
    for coord in lines:
        if coord[1] == coord[3]:
            line = np.linspace(coord[0], coord[2], POINTS)
            plt.plot(line, [coord[1]] * POINTS)
        if coord[0] == coord[2]:
            line = np.linspace(coord[1], coord[3], POINTS)
            plt.plot([coord[0]] * POINTS, line)
    return plt



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
            if grid_map.get_relative(x, y) == CellType.OPEN:
                if grid_map.get_relative(x + 1, y) == CellType.OPEN:
                    possible_squares.append((x, y, x_next, y))
                if grid_map.get_relative(x, y + 1) == CellType.OPEN:
                    possible_squares.append((x, y, x, y_next))

    return possible_squares



def get_size(coordinates):
    """
    Extracts number of Sqaures with the size of GRID_SIZE
    :param coordinates: Takes in a list of coordinates
    :return: [min_x, max_x, min_y, max_y] in number of areas with GRID_SIZE
    """
    min_x, max_x, min_y, max_y = 0, 0, 0, 0

    for x, y in coordinates:
        min_x, max_x = check_size(x, min_x, max_x)
        min_y, max_y = check_size(y, min_y, max_y)

    min_x, max_x = get_grid(min_x, max_x)
    min_y, max_y = get_grid(min_y, max_y)

    return min_x, max_x, min_y, max_y



def check_size(val, min, max):
    """
    Checks if val is smaller than min value or larger than max value
    :param val: Value to chek if it is smaller than min or larger than max
    :param min: A min value to compare with and also the out value
    :param max: A max value to compare with and also the out value
    :return: Minimum value and maximum value
    """
    if val < min:
        min = val
    elif val > max:
        max = val
    return min, max


def get_grid(min, max):
    """
    Changes the 400x400 grid to a 1x1 grid in 1D
    :param min: Minimum value in 1D
    :param max: Maximum value in 1D
    :return: returns how many grids in 1D there is, as a mimum value toa maximum value
    """
    min_grid = min//GRID_SIZE
    max_grid = max//GRID_SIZE + 1

    return int(min_grid), int(max_grid)


def get_horizontal_lines(coordinates, x_min, x_max, y_min, y_max):
    """
    Gets a list of all mesurments on horizontal lines
    :param coordinates: A list with all coordinates
    :param size: [min_x, max_x, min_y, max_y]
    :return: Returns a list [[[ammount of mesures in nr of POINTS], [..]], [row 2 ..], [[..],[..]]]
    """
    x_size = x_max - x_min + 1
    y_size = y_max - y_min + 1

    horizontal_lines = np.array([[[0]*POINTS]*x_size]*y_size)

    # Loops over al coordinates
    for x, y in coordinates:
        pos_x = int(x//GRID_SIZE) - x_min
        pos_x_loc = int((x%GRID_SIZE)//int(GRID_SIZE/POINTS_LINE))
        pos_y = int(y//GRID_SIZE) - y_min
        pos_y_dist = y%GRID_SIZE

        # Checks if a mesurement is within the selected area from line, area is +- ACCURACY from line
        if pos_y_dist < ACCURACY:
            horizontal_lines[pos_y][pos_x][pos_x_loc] += 1
        elif pos_y_dist > (GRID_SIZE - ACCURACY):
            horizontal_lines[pos_y+1][pos_x][pos_x_loc] += 1

    return horizontal_lines



def get_vertical_lines(coordinates, x_min, x_max, y_min, y_max):
    """
    Gets a list of all mesurments on vertical lines
    :param coordinates: A list with all coordinates
    :param size: [min_x, max_x, min_y, max_y]
    :return: Returns a list [[[ammount of mesures in nr of POINTS], [..]], [colum 2 ..], [[..],[..]]]
    """
    x_size = x_max - x_min + 1
    y_size = y_max - y_min + 1

    vertical_lines = np.array([[[0]*POINTS]*x_size]*y_size)

    # Loops over al coordinates
    for x, y in coordinates:
        pos_x = int(x//GRID_SIZE) - x_min
        pos_y_loc = int((y%GRID_SIZE)//int(GRID_SIZE/POINTS))
        pos_y = int(y//GRID_SIZE) - y_min
        pos_x_dist = x%GRID_SIZE

        # Checks if a mesurement is within the selected area from line, area is +- ACCURACY from line
        if pos_x_dist < ACCURACY:
            vertical_lines[pos_y][pos_x][pos_y_loc] += 1
        elif pos_x_dist > (GRID_SIZE - ACCURACY):
            vertical_lines[pos_y][pos_x+1][pos_y_loc] += 1

    return vertical_lines



def get_coordinates(mesurments, x_position, y_position, angle):
    """
    Gets coordinates out of mesures
    :param mesurments: A list of mesures, where a mesurement is [degree, distancs]
    :param robot: Robot position and angle, [x, y, angle]
    :return: A list of coordinates
    """
    coordinates = []
    for degree, dist in mesurments:
        x = (math.sin(math.radians(degree + angle)) * dist) + x_position
        y = (math.cos(math.radians(degree + angle)) * dist) + y_position
        coordinates.append([x, y])

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

        logging.info("distance", dist)
        logging.info("move to", degree)

        # Read controller info
        # uart.send_function(ControllerInformation())
        # for i in range(6):
        #    lsb = uart.receive_packet()
        # print (ord(lsb))

        x = math.sin(math.radians(degree)) * dist
        y = math.cos(math.radians(degree)) * dist

        measurements.append([degree, dist])

        uart.send_function(Servo(int(degree)))
        time.sleep(0.005)
    return measurements



def test_plot(map, coordinates, x_min, x_max, y_min, y_max):
    """
    A test plotting, that shows all mesuring points and walls on the same plot
    """
    plt = plot_room(map)
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

    if (x*GRID_SIZE, y*GRID_SIZE, x_next*GRID_SIZE, y*GRID_SIZE) == line:
        if y > robot_y:
            grid_map.set(x, y_next, CellType.WALL)
        elif y < robot_y:
            grid_map.set(x, y_prev, CellType.WALL)

    if (x*GRID_SIZE, y*GRID_SIZE, x*GRID_SIZE, y_next*GRID_SIZE) == line:
        if x > robot_x:
            grid_map.set(x, y, CellType.WALL)
            if not (grid_map.get_relative(x, y) == CellType.WALL):
                grid_map.set(x, y, CellType.OPEN)
        elif x < robot_x:
            grid_map.set(x_prev, y, CellType.WALL)

    if not (grid_map.get_relative(x, y) == CellType.WALL):
        grid_map.set(x, y, CellType.OPEN)
    return True
