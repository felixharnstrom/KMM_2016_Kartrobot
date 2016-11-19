from UART import UART
from modules import *
from sensorenhet_functions import *
import json
import math, time
import logging
import matplotlib.pyplot as plt
import numpy as np


logging.getLogger(__name__).setLevel(logging.INFO)


grid = []

# Constants for howe accurate this system should be
GRID_SIZE = 400     # What si the Grid size
POINTS = 4          # Every line of GRID_SIZE, in how many parts should it be devided
MIN_MESURE = 1      # How many messurments should be in every like section
POINTS_LINE = 3     # How many Linesections should have messures to be a line
ACCURACY = 100      # How far from line is it ok for mesurments to be


# TODO consider robot position and angle when getting coordinates
def plot_room(robot, plt):
    """
    This definition plots the room
    :param robot: A list with robot position and angle [x, y, angle]
    :param plt: Pevious ploting of room
    :return: Returns plt, that is the plot of room
    """

    #coordinates = get_coordinates(measure_lidar(), robot)

    coordinates = get_coordinates(read_debug_data('demo_data/perfect_square_center_raw_data.json'), robot)
    #coordinates = read_debug_data('demo_data/perfect_square_center.json')
    #coordinates = read_debug_data('demo_data/triple_sided_wall_with_imperfections.json')

    # Gets size coordinate area in squares of GRID_SIZE
    # size = [min_x, max_x, min_y, max_y]
    size = get_size(coordinates)

    # Gets lists contaning all mesurements that  could be on horizontal lines and vertical lines
    horizontal_lines = get_horizontal_lines(coordinates, size)
    vertical_lines = get_vertical_lines(coordinates, size)

    # Loops over the entire area based on squares of GRID_SIZE
    for n in range(size[2], size[3]):
        # Stores square position, start in y and end in y.
        y = GRID_SIZE * n
        y_next = GRID_SIZE * (n + 1)
        y_line = np.linspace(y, y_next, POINTS)
        y_pos = n - size[2]     # corrects that position values can be negative, but lists don't have neg values

        for m in range(size[0], size[1]):
            # Stores square position, start in y and end in y.
            x = GRID_SIZE * m
            x_next = GRID_SIZE * (m + 1)
            x_line = np.linspace(x, x_next, POINTS)
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
                plt.plot(x_line, [y] * POINTS)      # Plot a horizontal line
            if tmp_vertical >= POINTS_LINE:
                plt.plot([x] * POINTS, y_line)      # Plot a vertical line

    # Plots all mesures points
    degree_plot = []
    distance_plot = []
    for degree, dist in coordinates:
        degree_plot.append(degree)
        distance_plot.append(dist)
    # If you don't want all mesurment point comment out the next line
    plt.plot(degree_plot, distance_plot, '.')

    # Create a area on graph, based on area of messures
    plt.plot([(size[0]-1)*GRID_SIZE, (size[1])*GRID_SIZE], [(size[2]-1)*GRID_SIZE, (size[3])*GRID_SIZE], '.')

    return plt



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

    for x, y in coordinates:
        if x < min_x:
            min_x = x
        elif x > max_x:
            max_x = x
        elif y < min_y:
            min_y = y
        elif y > max_y:
            max_y = y

    return [int(min_x//GRID_SIZE), int((max_x//GRID_SIZE)+2), int(min_y//GRID_SIZE), int((max_y//GRID_SIZE)+2)]



def get_horizontal_lines(coordinates, size):
    """
    Gets a list of all mesurments on horizontal lines
    :param coordinates: A list with all coordinates
    :param size: [min_x, max_x, min_y, max_y]
    :return: Returns a list [[[ammount of mesures in nr of POINTS], [..]], [row 2 ..], [[..],[..]]]
    """
    horizontal = np.array([[[0]*POINTS]*(size[1]-size[0])]*(size[3]-size[2]))

    # Loops over al coordinates
    for x, y in coordinates:
        pos_x = int(x//GRID_SIZE)-size[0]
        pos_x_loc = int((x%GRID_SIZE)//int(GRID_SIZE/POINTS_LINE))
        pos_y = int(y//GRID_SIZE)-size[2]
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
    vertical = np.array([[[0]*POINTS]*(size[1]-size[0])]*(size[3]-size[2]))

    # Loops over al coordinates
    for x, y in coordinates:
        pos_x = int(x//GRID_SIZE)-size[0]
        pos_y_loc = int((y%GRID_SIZE)//int(GRID_SIZE/POINTS))
        pos_y = int(y//GRID_SIZE)-size[2]
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
        x = (math.sin(math.radians(degree + robot[2])) * dist) + robot[0]
        y = (math.cos(math.radians(degree + robot[2])) * dist) + robot[1]
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
    :return: Measurements in [[x1,y2],[x2,y2], ...]
    """
    uart = UART("ttyUSB1")

    sensorunit = UART("ttyUSB0")
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

