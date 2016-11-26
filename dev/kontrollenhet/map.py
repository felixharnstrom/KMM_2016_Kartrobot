
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
ANGEL_DIFF = -10    # How much the staring point of LIDAR scanner is off


def map_room(robot, map):
    """
    This definition plots the room
    :param robot: A list with robot position and angle [x, y, angle]
    :param map: Pevious mapping of the room
    :return: Returns new_lines, that is a list of tupels with line information [(x11, y11, x21, y21), (x12, y12...), ...]
    """
    coordinates = get_coordinates(measure_lidar(), robot)

    #coordinates = get_coordinates(read_debug_data('demo_data/perfect_square_center_raw_data.json'), robot)

    # Gets size coordinate area in squares of GRID_SIZE
    # size = [min_x, max_x, min_y, max_y]
    size = get_size(coordinates)

    # Gets lists contaning all mesurements that  could be on horizontal lines and vertical lines
    horizontal_lines = get_horizontal_lines(coordinates, size)
    vertical_lines = get_vertical_lines(coordinates, size)

    new_lines = []

    # Loops over the entire area based on squares of GRID_SIZE
    for n in range(size[2], size[3]):
        # Stores square position, start in y and end in y.
        y = GRID_SIZE * n
        y_next = GRID_SIZE * (n + 1)
        y_pos = n - size[2]     # corrects that position values can be negative, but lists don't have neg values

        for m in range(size[0], size[1]):
            # Stores square position, start in y and end in y.
            x = GRID_SIZE * m
            x_next = GRID_SIZE * (m + 1)
            x_pos = m - size[0]     # corrects that position values can be negative, but lists don't have neg values

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

            # Checks if ammount of Lines sections are >= than POINTS_LINE
            if tmp_horizontal >= POINTS_LINE:
                line = (x, y, x_next, y)            # create a tupel with line start and end point
                if line not in map:
                    map.append(line)
                    new_lines.append(line)
            if tmp_vertical >= POINTS_LINE:
                line = (x, y, x, y_next)            # create a tupel with line start and end point
                if line not in map:
                    map.append(line)
                    new_lines.append(line)

    # If you don't want it to plot all lines and all mesuring points then uncomment this
    test_plot(map, coordinates, size)

    return new_lines

    


def plot_room(map):
    """
    Plots the room and return the plot
    :param map: Is a list of tupels that has the information about walls [(x11, y11, x21, y21), (x12, y12, ...), ...]
    :return: A list of coordinates
    """
    for coord in map:
        if coord[1] == coord[3]:
            line = np.linspace(coord[0], coord[2], POINTS)
            plt.plot(line, [coord[1]] * POINTS)
        if coord[0] == coord[2]:
            line = np.linspace(coord[1], coord[3], POINTS)
            plt.plot([coord[0]] * POINTS, line)
    return plt



def check_available_grid(map):
    """
    Checks all available arches between nodes the robot can go
    :param map: A list with tupleres containing all walls
    :return: A list of all posible grid -> grid [(fron_x1, from_y1, to_x1, to_y1), (from_x2, from_y2, ...), ...]
    """
    size = get_size(map)
    possible_squares = []
    corr = 0        # size correction, should it go outside known area or not, 0 = not, 1 = yes

    for y in range(size[2], size[3] - 1 + corr):
        y_next = y + 1

        for x in range(size[0], size[1] - 1 + corr):
            x_next = x + 1

            if ((x + 1) * GRID_SIZE, y * GRID_SIZE, (x + 1) * GRID_SIZE, y_next * GRID_SIZE) not in map:
                possible_squares.append((x, y, x_next, y))
            if (x * GRID_SIZE, (y + 1) * GRID_SIZE, x_next * GRID_SIZE, (y + 1) * GRID_SIZE) not in map:
                possible_squares.append((x, y, x, y_next))

    return possible_squares



def get_size(coordinates):
    """
    Extracts number of Sqaures with the size of GRID_SIZE
    :param coordinates: Takes in a list of coordinates
    :return: [min_x, max_x, min_y, max_y] in number of areas with GRID_SIZE
    """
    min_x = 0
    max_x = 0
    min_y = 0
    max_y = 0
    if len(coordinates[0]) == 2:
        for x, y in coordinates:
            min_x, max_x =check_size(x, min_x, max_x)
            min_y, max_y =check_size(y, min_y, max_y)
    elif len(coordinates[0]) == 4:
        for x1, y1, x2, y2 in coordinates:
            min_x, max_x = check_size(x1, min_x, max_x)
            min_x, max_x =check_size(x2, min_x, max_x)
            min_y, max_y =check_size(y1, min_y, max_y)
            min_y, max_y =check_size(y2, min_y, max_y)
    print (min_x,min_y,max_x,max_y)

    return [int((min_x//GRID_SIZE)-1), int((max_x//GRID_SIZE)+2), int((min_y//GRID_SIZE)-1), int((max_y//GRID_SIZE)+2)]



def check_size(val, min, max):
    """
    Checks if val is smaller than min value or larger than max value
    :param val: Value to chek if it is smaller than min or larger than max
    :param min: A min value to compare with and also the out value
    :param max: A max value to compare with and also the out value
    """
    if val < min:
        min = val
    elif val > max:
        max = val
    return min, max


def get_horizontal_lines(coordinates, size):
    """
    Gets a list of all mesurments on horizontal lines
    :param coordinates: A list with all coordinates
    :param size: [min_x, max_x, min_y, max_y]
    :return: Returns a list [[[ammount of mesures in nr of POINTS], [..]], [row 2 ..], [[..],[..]]]
    """
    x_size = size[1] - size[0]
    y_size = size[3] - size[2]

    horizontal = np.array([[[0]*POINTS]*x_size]*y_size)

    # Loops over al coordinates
    for x, y in coordinates:
        pos_x = int(x//GRID_SIZE) - size[0]
        pos_x_loc = int((x%GRID_SIZE)//int(GRID_SIZE/POINTS_LINE))
        pos_y = int(y//GRID_SIZE) - size[2]
        pos_y_dist = y%GRID_SIZE

        # Checks if a mesurement is within the selected area from line, area is +- ACCURACY from line
        if pos_y_dist < ACCURACY:
            horizontal[pos_y][pos_x][pos_x_loc] += 1
        elif pos_y_dist > (GRID_SIZE - ACCURACY):
            horizontal[pos_y+1][pos_x][pos_x_loc] += 1

    return horizontal



def get_vertical_lines(coordinates, size):
    """
    Gets a list of all mesurments on vertical lines
    :param coordinates: A list with all coordinates
    :param size: [min_x, max_x, min_y, max_y]
    :return: Returns a list [[[ammount of mesures in nr of POINTS], [..]], [colum 2 ..], [[..],[..]]]
    """
    x_size = size[1] - size[0]
    y_size = size[3] - size[2]

    vertical = np.array([[[0]*POINTS]*x_size]*y_size)

    # Loops over al coordinates
    for x, y in coordinates:
        pos_x = int(x//GRID_SIZE) - size[0]
        pos_y_loc = int((y%GRID_SIZE)//int(GRID_SIZE/POINTS))
        pos_y = int(y//GRID_SIZE) - size[2]
        pos_x_dist = x%GRID_SIZE

        # Checks if a mesurement is within the selected area from line, area is +- ACCURACY from line
        if pos_x_dist < ACCURACY:
            vertical[pos_y][pos_x][pos_y_loc] += 1
        elif pos_x_dist > (GRID_SIZE - ACCURACY):
            vertical[pos_y][pos_x+1][pos_y_loc] += 1

    return vertical



def get_coordinates(mesurments, robot):
    """
    Gets coordinates out of mesures
    :param mesurments: A list of mesures, where a mesurement is [degree, distancs]
    :param robot: Robot position and angle, [x, y, angle]
    :return: A list of coordinates
    """
    coordinates = []
    for degree, dist in mesurments:
        x = (math.sin(math.radians(degree + robot[2] + ANGEL_DIFF)) * dist) + robot[0]
        y = (math.cos(math.radians(degree + robot[2] + ANGEL_DIFF)) * dist) + robot[1]
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

    time.sleep(2)
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

    return measurements











def test_plot(map, coordinates, size):
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
    plt.plot([(size[0] - 1) * GRID_SIZE, (size[1]) * GRID_SIZE], [(size[2] - 1) * GRID_SIZE, (size[3]) * GRID_SIZE], '.')
    plt.show()

def get_grid_map(robot_x, robot_y, lines):
    """Returns a GridMap containing the room as we currently know it."""
    # I exptect get_size to output to be the rectangle encapsulating
    #   all lines, but it seems to be wrong.
    min_x, max_x, min_y, max_y = [e*GRID_SIZE for e in get_size(lines)]
    # This could probably take a gridmap to modify instead
    rslt = GridMap()
    print("MIN", min_x, min_y)
    # Translate each line endpoint to a cell
    # As each is GRID_SIZE long, this wont miss inbetween cells
    for line in lines:
        # Copy
        linecpy = [coord for coord in line]
        # Have coordinates originate from origin
        linecpy[0] -= min_x
        linecpy[1] -= min_y
        linecpy[2] -= min_x
        linecpy[3] -= min_y
        print(linecpy)
        # Adjust coordinates to align with center of a cell
        for i, coord in enumerate(linecpy):
            linecpy[i] = coord + GRID_SIZE//2
            print(linecpy[i])
        # Calculate grid indices
        gx1, gy1, gx2, gy2 = [math.floor(coord/GRID_SIZE) for coord in linecpy]
        print(gx1, gy1)
        print(gx2, gy2)
        print("")
        rslt.set(gx1, gy1, CellType.WALL)
        rslt.set(gx2, gy2, CellType.WALL)
    # TODO: Update robot position in result (fairly trivial, same algorithm)
    # TODO: Change discovered open cells from CellType.UNKNOWN to CellType.OPEN
    return rslt

# Test
#get_grid_map(0,0,[(0, -400, 400, -400), (400, -400, 400, 0), (400, 0, 400, 400), (0, 400, 400, 400)]).debug_print()
