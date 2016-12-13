
from geometry import *
from robot_communication import *
import json
import math, time
import numpy as np
import matplotlib.pyplot as plt
from grid_map import GridMap, CellType

"""The real-world size of a grid cell, in millimeters. Also the length of a line."""
CELL_SIZE = 400

"""The number of line segments per line."""
DIVISIONS_PER_LINE = 4

"""The number of votes per line section required for something to be considered a full line."""
MIN_MESURE = 3

"""The number of line segments that needs to be voted in for their line to be voted in."""
SEGMENTS_REQUIRED = 3

"""The thickness of a line segment in millimeters."""
ACCURACY = 200

"""The length of a line segment in millimeters."""
LINE_SEG_LENGTH = CELL_SIZE // DIVISIONS_PER_LINE

"""The sleep time when moving the servo to 0."""
LIDAR_INITIAL_SLEEP = 1.5


"""The sleep time when moving the servo 1 degree."""
LIDAR_SLEEP = 0.015

def raytrace(line):
    # Stolen from
    x0 = line.start.x
    y0 = line.start.y
    x1 = line.end.x
    y1 = line.end.y
    dx = abs(x1 - x0)
    dy = abs(y1- y0)
    x = x0
    y = y0
    n = 1 + dx + dy
    x_inc = 1 if x1 > x0 else -1
    y_inc = 1 if y1 > y0 else -1
    error = dx - dy
    dx *= 2
    dy *= 2

    result = []
    
    while n > 0:
        result.append(Position(x, y))
        if error > 0:
            x += x_inc
            error -= dy
        else:
            y += y_inc
            error += dx
        n -= 1
    return result

def bresenham(line):
    """
    Source: http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm

    Bresenham's Line Algorithm
    Return a list of cell indices connecting the end-points of a given line, as few as possible, approximated to lay on the line as much as possible.
    
    Args:
        :param line (Line): a line to find a path through.
    Returns:
        :return (list of Position): The cell indices the line passes through.
    """
    # Extract points
    start = line.start
    end = line.end
    # Setup initial conditions
    x1 = start.x
    y1 = start.y
    x2 = end.x
    y2 = end.y
    dx = x2 - x1
    dy = y2 - y1
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
 
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
 
    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1
 
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        point = Position(y, x) if is_steep else Position(x, y)
        points.append(point)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
 
    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points


def coordinates_to_lines(coordinates):
    """
    Approximate coordinates to line segments.

    Args:
        :param coordinates (List of length 2 tuple of floats): Measurement endpoints formatted as (x, y), in millimeters.

    Returns:
        :return (list of Line's): Lines with end-points in millimeters describing walls, each being horizontal or vertical with lengths of CELL_SIZE millimeters.
    """
    # Gets size coordinate area in squares of CELL_SIZE
    top_left = top_left_grid_index(coordinates)
    bottom_right = bottom_right_grid_index(coordinates)
    # Get votes
    horizontal_votes = get_votes_for_horizontal_line_segments(coordinates, top_left, bottom_right)
    vertical_votes = get_votes_for_vertical_line_segments(coordinates, top_left, bottom_right)

    # Our return value
    lines = []

    # Loop over y-indices for the grid
    for y_index in range(top_left.y, bottom_right.y + 1):
        # Stores square position, start in y and end in y.
        y = CELL_SIZE * y_index
        y_next = y + CELL_SIZE
        
        # Loop over x-indices for the grid
        for x_index in range(top_left.x, bottom_right.x + 1):
            # Stores square position, start in x and end in x.
            x = CELL_SIZE * x_index
            x_next = x + CELL_SIZE

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
            if horizontal_line_segs >= SEGMENTS_REQUIRED:
                start = Position(x, y)
                end = Position(x_next, y)
                line = Line(start, end)
                if line not in lines:
                    lines.append(line)
            if vertical_line_segs >= SEGMENTS_REQUIRED:
                start = Position(x, y)
                end = Position(x, y_next)
                line = Line(start, end)
                if line not in lines:
                    lines.append(line)
    return lines

def open_missed_corners(grid_map:GridMap):
    """
    Change unknown tiles surrounded by known tiles to OPEN.
    Repeat this until no unknown tiles are surrounded.

    Args:
        :param grid_map (GridMap): The GridMap to modify.
    """
    # TODO: Worst case is O(W*W*H*H)
    # Pretty dang slow.

    if grid_map.width() < 3 or grid_map.height() < 3:
        # No tiles are surrounded
        return

    tl = grid_map.top_left()
    br = grid_map.bottom_right()
    for x in range(tl.x + 1, br.x - 1):
        for y in range(tl.y + 1, br.y - 1):
            neighbours = ((x+1, y), (x-1, y), (x, y+1), (x, y-1))
            unknown_n = sum(grid_map.get(nx, ny) == CellType.UNKNOWN for nx, ny in neighbours)
            if unknown_n == 0 and grid_map.get(x, y) == CellType.UNKNOWN:
                grid_map.set(x, y, CellType.OPEN)
                # Recursive call
                open_missed_corners(grid_map)
                return
                

# TODO: Non-DRY
def update_grid_map(lines, robot_pos:Position, grid_map:GridMap):
    """
    Gives every grid a CellType, as OPEN or WALL. OPEN if the grid is open from robot point of wiev adn WALL if the grid is behind a wall from robot point of view.

    Args:
        :param (list of Line): Lines describing walls, each being horizontal or vertical with lengths of CELL_SIZE millimeters.
        :param robot_pos (Position): The position of the robot as the measurements was taken, in millimeters.
        :param grid_map (GridMap): The GridMap to modify.
    """
    # Gets size coordinate area in squares of CELL_SIZE
    line_positions = [line.start for line in lines] + [line.end for line in lines]
    line_coordinates = [(pos.x, pos.y) for pos in line_positions]
    top_left = top_left_grid_index(line_coordinates)
    bottom_right = bottom_right_grid_index(line_coordinates)

    # Robot pos in grid
    robot_grid_pos = Position(math.floor(robot_pos.x/CELL_SIZE), math.floor(robot_pos.y/CELL_SIZE))
    
    # Loop over y-indices for the grid
    for y_index in range(top_left.y, bottom_right.y + 1):
        # Stores square position, start in y and end in y as multiplied with CELL_SIZE.
        y = CELL_SIZE * y_index
        y_next = y + CELL_SIZE

        # Loop over x-indices for the grid
        for x_index in range(top_left.x, bottom_right.x + 1):
            # Stores square position, start in x and end in x as multiplied with CELL_SIZE.
            x = CELL_SIZE * x_index
            x_next = x + CELL_SIZE

            changed_pos_hor = None
            changed_pos_ver = None

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
                changed_pos_hor = change_grid_type(robot_pos, grid_pos, line_horizontal, grid_map)

            # Check if there exists a vertical line from current grid, if it exists create a WALL grid
            if line_vertical in lines:
                changed_pos_ver = change_grid_type(robot_pos, grid_pos, line_vertical, grid_map)

            # Change all cells between the new walls and the robot to OPEN
            # For each new wall
            for changed_pos in (changed_pos_hor, changed_pos_ver):
                # If we have a new wall
                if changed_pos is not None:
                    cells_between = bresenham(Line(robot_grid_pos, changed_pos))
                    #print(robot_grid_pos, "->", changed_pos, ":", len(cells_between))
                    for cell in cells_between:
                        # If the cell isn't known, set it to OPEN
                        if grid_map.get(cell.x, cell.y) == CellType.UNKNOWN:
                            grid_map.set(cell.x, cell.y, CellType.OPEN)

    # Bresenham can miss a few points on large maps.
    # This takes care of those.
    open_missed_corners(grid_map)


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
            plot_line = np.linspace(start.y, end.y, DIVISIONS_PER_LINE)
            plt.plot([start.x] * DIVISIONS_PER_LINE, plot_line)
        # Vertical lines
        elif start.y == end.y:
            plot_line = np.linspace(start.x, end.x, DIVISIONS_PER_LINE)
            plt.plot(plot_line, [start.y] * DIVISIONS_PER_LINE)
    return plt


def top_left_grid_index(coordinates):
    """
    Return the top_left coordinates of an AABB enclosing all coordinates and origin, scaled by 1/CELL_SIZE.
    
    Args:
        :param coordinates (List of length 2 tuple of float): Measurement endpoints formatted as (x, y), in millemeters.

    Returns:
        :return (Position): The top left position of said AABB, scaled by 1/CELL_SIZE (i.e an offset of 1 is an offset of CELL_SIZE millimeters).
    """
    top_left = Position(0, 0)
    for x, y in coordinates:
        top_left.x = min(top_left.x, int(x//CELL_SIZE))
        top_left.y = min(top_left.y, int(y//CELL_SIZE))
    return top_left

def bottom_right_grid_index(coordinates):
    """
    Return the bottom_left coordinates of an AABB enclosing all coordinates and origin, scaled by 1/CELL_SIZE.
    
    Args:
        :param coordinates (List of length 2 tuple of float): Measurement endpoints formatted as (x, y), in millimeters.

    Returns:
        :return (Position): The bottom right position of said AABB, scaled by 1/CELL_SIZE (i.e an offset of 1 is an offset of CELL_SIZE millimeters).
    """
    bottom_right = Position(0, 0)
    for x, y in coordinates:
        bottom_right.x = max(bottom_right.x, int(x//CELL_SIZE + 1))
        bottom_right.y = max(bottom_right.y, int(y//CELL_SIZE + 1))
    return bottom_right



def get_votes_for_axis_aligned_line_segments(coordinates, top_left, bottom_right, vertical:bool):
    """
    For each possible line segment approximate each coordinate to the closest segment, and return the
    number of coordinates approximated to fall in each line segment (the number of "votes"). A line segment 
    is considered a line with a length of LINE_SEG_LENGTH, while a full line has a length of CELL_SIZE.

    Vertical determines if we attempt to approximate coordinates to vertical lines (True) or horizontal
    ones (False).

    Args:
        :param coordinates: A list of tuples of floats, containing real-world (x, y) coordinates in millimeters.
        :param vertical: What sort of lines to approximate to. True if vertical, False if horizontal.

    Returns:
        :return (numpy.array of int): a 3-dimensional list.
        The first two indices indicates the starting point of a line, divided by CELL_SIZE. The third an offset
        off n*ACCURACY, where n is the index (0 =< n < DIVISIONS_PER_LINE), on the x-axis or y-axis depending on if the
        line is vertical or horizontal. The lines end-point will thus be offset by an 
        additional (n+1)*ACCURACY. On that index we'll find the number of coordinates approximated to
        fall on that line.

        To be more specific, vertical being True or False determines if the last index causes us to move
        along the x-axis (vertical=False) or the y-axis (vertical=True).

        examples:
        assuming not vertical, [0][0][0] contains votes for the line (0,0) -> (ACCURACY, 0)
        assuming vertical,     [0][0][0] contains votes for the line (0,0) -> (0, ACCURACY)
        assuming not vertical, [0][0][2] contains votes for the line (2*ACCURACY, 0) -> (3*ACCURACY, 0)
        assuming not vertical, [1][0][0] contains votes for the line (1*CELL_SIZE, 0) -> (1*CELL_SIZE + 1*ACCURACY, 0)
    """
    # The size of the AABB enclosing all coordinates
    size = bottom_right.difference(top_left)
    size.add(Size(1, 1))
    # Array instead of list because list doesn't work as intended
    # 3-dimensional
    votes = np.array([[[0]*DIVISIONS_PER_LINE]*size.w]*size.h)

    for x, y in coordinates:
        # Approximate position to grid
        # Approximate the pair of coordinates to the grid
        pos_x = int(x//CELL_SIZE - top_left.x)
        pos_y = int(y//CELL_SIZE - top_left.y)
        pos = Position(pos_x, pos_y)
        
        # 
        pos_dist = Position(x%CELL_SIZE, y%CELL_SIZE)
        relevant_dist = pos_dist.x if vertical else pos_dist.y

        #
        factor = int(CELL_SIZE/SEGMENTS_REQUIRED)
        pos_loc = Position(pos_dist.x//factor, pos_dist.y//factor)
        relevant_loc = pos_loc.y if vertical else pos_loc.x
        
        # Check if measurement is within +- ACCURACY from line
        if relevant_dist < ACCURACY:
            votes[pos.y][pos.x][relevant_loc] += 1
        elif relevant_dist > (CELL_SIZE - ACCURACY):
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



def change_grid_type(robot_pos:Position, grid_pos:Position, line:Line, grid_map:GridMap):
    """
    Updates a grid cell to CellType.WALL if it falls on the given wall line, or leaves it unchanged otherwise.

    Args:
        :param robot_pos (Position): The position of the robot as the line was scanned in millimeters.
        :param grid_pos (Position): The grid cell to check.
        :param grid_map (GridMap): The GridMap to modify.

    Returns:
        :return (Position): The grid index that was changed. Can be None.
    """
    next_grid_pos = Position(grid_pos.x + 1, grid_pos.y + 1)
    prev_grid_pos = Position(grid_pos.x - 1, grid_pos.y - 1)

    #create a line based on current grid position
    next_start = Position(grid_pos.x * CELL_SIZE, grid_pos.y * CELL_SIZE)
    next_vertical_end = Position(grid_pos.x * CELL_SIZE, next_grid_pos.y * CELL_SIZE)
    next_horizontal_end = Position(next_grid_pos.x * CELL_SIZE, grid_pos.y * CELL_SIZE)
    next_vertical = Line(next_start, next_vertical_end)
    next_horizontal = Line(next_start, next_horizontal_end)

    #Check if there is a line at current grid position.
    if line == next_vertical:
        if grid_pos.x > robot_pos.x / CELL_SIZE:
            grid_map.set(grid_pos.x, grid_pos.y, CellType.WALL)
            return Position(grid_pos.x, grid_pos.y)
        elif grid_pos.x < robot_pos.x / CELL_SIZE:
            grid_map.set(prev_grid_pos.x, grid_pos.y, CellType.WALL)
            return Position(prev_grid_pos.x, grid_pos.y)
    elif line == next_horizontal:
        if grid_pos.y > robot_pos.y / CELL_SIZE:
            grid_map.set(grid_pos.x, grid_pos.y, CellType.WALL)
            return Position(grid_pos.x, grid_pos.y)
        elif grid_pos.y < robot_pos.y / CELL_SIZE:
            grid_map.set(grid_pos.x, prev_grid_pos.y, CellType.WALL)
            return Position(grid_pos.x, prev_grid_pos.y)
    return None


def convert_to_coordinates(measurements, robot_pos:Position, angle):
    """
    Convert angle and distances to coordinates via polar projection.
    
    Args:
        :param measurements (list of length-2-tuple of float: Tuples containing (angle, distance to wall), where angle is degrees.
        :param robot_pos (Position): The robot position in millimeters; origin point of the measurements.
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



def measure_lidar():
    """
    Turn the laser 180 degrees, taking measurements. Return said measurements.

    Returns:
        :return: (list of length-2-tuple of float): Tuples containing (angle, distance to wall), where angle is degrees.
    """
    degree_plot = []
    distance_plot = []

    handle_command(Command.servo(0))

    time.sleep(LIDAR_INITIAL_SLEEP)
    degree = 0

    measurements = []

    for degree in range(0, 180):
        dist = handle_command(Command.read_lidar())
        measurements.append([degree, dist])
        handle_command(Command.servo(int(degree)))
        time.sleep(LIDAR_SLEEP)
    handle_command(Command.servo(90))
    return measurements


def scan_and_update_grid(robot_pos:Position, robot_angle:float, grid_map:GridMap):
    """
    Scan the room and update grid_map with walls an open spaces found.

    Args:
        :param robot_pos (Position): The position of the robot, in millimeters.
        :param robot_angle (float): The facing angle of the robot, in degrees..
        :grid_map (GridMap): The GridMap to insert the results into.
        :passes (int): The number of times we rotate the LIDAR to measure.
    """
    measurements = measure_lidar()
    coordinates = convert_to_coordinates(measurements, robot_pos, robot_angle)
    lines = coordinates_to_lines(coordinates)
    update_grid_map(lines, robot_pos, grid_map)

    

def debug_plot(coordinates, lines):
    """
    A test plotting, that shows all meuring points and walls on the same plot. This is for debugging purpesus only.
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
    plt.plot([(top_left.x - 1) * CELL_SIZE, (bottom_right.x + 1) * CELL_SIZE],
             [(top_left.y) * CELL_SIZE, (bottom_right.y) * CELL_SIZE], '.')
    plt.show()


def approximate_to_cell(pos, resolution=1):
    """
    Convert a real-world position (in mm) to a cell position (in CELL_SIZE/resolution mm).

    Args:
        :param pos (Position): The position to convert, in mm.
        :param resolution (float, >= 1): The amount to scale the result up with. 

    Returns:
        :return (Position): The cell indices the pos represents, in CELL_SIZE/resolution mm.

    """
    dx = pos.x 
    dy = pos.y 
    return Position(math.floor(resolution*dx / CELL_SIZE), math.floor(resolution*dy / CELL_SIZE))

def mark_all_cells_direction(cells, direction):
    #print(direction)
    for cell in cells:
        cell.direction = direction


def movement_lines_to_cells(lines, resolution=1):
    """
    Return the cells that a list of real-world lines passes through.

    Args:
        :param lines (list of Line): Lines indicating path of movement, in mm.
        :param resolution (float, >= 1): The amount to scale the result up with. Primarily for debugging.

    Returns:
        :return (list of Position): A list of cell indices the lines passes through, in CELL_SIZE/resolution mm.
    """
    grid_points = []
    print ("LINES",len(lines))
    for line in lines:
        start = approximate_to_cell(line.start, resolution)
        end = approximate_to_cell(line.end, resolution)
        between = bresenham(Line(start, end))
        print (line.start, line.end)
        if line.end.x > line.start.x:
            mark_all_cells_direction(between, Direction.LEFT)
        if line.end.x < line.start.x:
            mark_all_cells_direction(between, Direction.RIGHT)
        if line.end.y > line.start.y:
            mark_all_cells_direction(between, Direction.UP)
        if line.end.y < line.start.y:
            mark_all_cells_direction(between, Direction.DOWN)

        grid_points += between
    return grid_points


def movement_to_lines(movements: list, start):
    """
    Convert a list of movements (angles and distances) to lines indicating path of movement

    Args:
        :param movements (list): List of (current_angle_in_degrees, distance_since_start_in_mm) of the robots movements.

    Returns:
        :return (list of Line): The path of movement represented by lines.
    """
    # Start first line at (0,0). (The second pair will be popped before use)
    points_x = [start.x, start.x]
    points_y = [start.y, start.y]

    lines = []

    # Convert to points
    last_distance = 0
    for angle, distance in movements:
        # Calculate distance moved since last savepoint
        current_distance = distance - last_distance

        # Add line ending (x2,y2), from last position (x1,y1)
        points_x.append(points_x[-1] + current_distance * math.sin(math.radians(angle)))
        points_y.append(points_y[-1] + current_distance * math.cos(math.radians(angle)))

        # Add current line to plot
        last_distance = distance

    # Convert to lines
    last = None
    for i in range(len(points_x)):
        current = Position(points_x[i], points_y[i])
        if last is not None:
            lines.append(Line(last, current))
        last = current
    return lines


def make_open(cells, grid_map):
    """
    Set all cell indices given to OPEN.

    Args:
        :param cells (list of Position): The cell indices to set.
        :param grid_map (GridMap): The GridMap to modify.
    """
    for cell in cells:
        grid_map.set(cell.x, cell.y, CellType.OPEN)

        
def set_to_wall_if_unknown(x, y, grid_map):
    """
    Set a cell to WALL if it is UNKNOWN.

    Args:
       :param x (int): The x-index to set.
       :param y (int): The y-index to set.
       :param grid_map (GridMap): The GridMap to modify.
    """
    if grid_map.get(x, y) == CellType.UNKNOWN:
        grid_map.set(x, y, CellType.WALL)

        
def add_walls(open_cells, grid_map):
    """
    Pad open cells with walls to the right.

    Args:
        :param open_cells (list of Position): The grid indices containing open cells to pad.
        :param grid_map (GridMap): The GridMap to modify.
    """
    last_dir = None
    for i in range(len(open_cells)-1):
        start = open_cells[i]
        
        print (open_cells[i].direction)
        if open_cells[i].direction == Direction.RIGHT:
            set_to_wall_if_unknown(start.x, start.y+1, grid_map)
            #set_to_wall_if_unknown(start.x, start.y-1, grid_map)
        if open_cells[i].direction == Direction.LEFT:
            set_to_wall_if_unknown(start.x, start.y-1, grid_map)
            #set_to_wall_if_unknown(start.x, start.y+1, grid_map)
        if open_cells[i].direction == Direction.DOWN:
            set_to_wall_if_unknown(start.x-1, start.y, grid_map)
            #set_to_wall_if_unknown(start.x+1, start.y, grid_map)
        if open_cells[i].direction == Direction.UP:
            set_to_wall_if_unknown(start.x+1, start.y, grid_map)
            #set_to_wall_if_unknown(start.x-1, start.y, grid_map)
        # Now available in animation!
        #grid_map.debug_print()
        #time.sleep(0.1)

def get_cells_passed(grid_pos:Position, angle:float):
    grid_map.get(grid_pos.x, grid_pos.y) # Expand to fit grid_pos
    # A line can be at most the length of the diagonal of the GridMap
    dx = grid_map.width()
    dy = grid_map.height()
    diagonal = math.sqrt(dx*dx + dy*dy)
    #print("GCELLS ANGLE:", angle)
    line_end = Position(grid_pos.x + diagonal*math.cos(math.radians(angle)),
                        grid_pos.y + diagonal*math.sin(math.radians(angle)))
    #print("END:", line_end)
    line = Line(grid_pos, line_end)
    # Find the cells line passes through
    return raytrace(line)

def first_cell_of_type(grid_pos:Position, angle:float, ctype:CellType, grid_map:GridMap):
    """
    Return the index of the first cell in grid_map matching ctype on a line from grid_pos
    at angle degrees, or None if no such cell exists.

    Args:
        :param grid_pos (Position): The position in the grid to originate from.
        :param angle (float): The angle to check at, in degrees.
        :param ctype (CellType): The cell type to look for.
        :param grid_map (GridMap): The Gridmap to read in.

    Returns:
        :return (Position): The cell index the celltype was first found on, or None if it doesn't exist.
    """
    cells = get_cells_passed(grid_pos, angle)
    # Look for cell of type
    for cell in cells:
        #print(cell)
        if not grid_map.is_within_bounds(cell.x, cell.y):
            return None
        elif grid_map.get(cell.x, cell.y) == ctype:
            return cell
    return None

def passes_through_unknown_before_wall(grid_pos:Position, angle:float, grid_map:GridMap):
    """
    Return True if iterating over all cells covering a line in a certain angle
    encounters an UNKNOWN cell before a WALL.

    Args:
        :param position (Position): The position to start at.
        :param angle (float): The angle to iterate in, in degrees.
        :param grid_map (GridMap): The grid_map to read from.

    Returns:
        :return (bool): True if UNKNOWN was encountered first.
    """
    cells = get_cells_passed(grid_pos, angle)
    # Look for cell of type
    for cell in cells:
        if not grid_map.is_within_bounds(cell.x, cell.y):
            return True
        if grid_map.get(cell.x, cell.y) == CellType.WALL:
            return False
        if grid_map.get(cell.x, cell.y) == CellType.UNKNOWN:
            return True
    return True
    
def measurements_with_island(start_pos:Position, robot_pos:Position, facing_angle:float,
                             measurements:list, grid_map:GridMap):
    """
    Return the measurements detecting an island: a previously undetected wall that seems to be
    at least one tile from a previously detected wall behind it.

    Args:
        :param start_pos (Position): The position in the maze the robot started at.
        :param robot_pos (Position): The current position of the robot, in millimeters.
        :param facing_angle (float): The current facing of the robot, in degrees.
        :param measurements (list): A list of tuples containing (angle, dist), where angle is the angle of the servo in degrees (0 to +180), and dist is the distance to a wall, in millimeters.
        :param grid_map (GridMap): A GridMap of previously detected walls.

    Returns:
        :return (list): A subset of measurements - those who indicates a previously undetected walls at least one tile from a previously detected wall behind it.
    """
    SMALL_FLOAT = 0.01 # For float comparison, just to be safe
    FAST = True # Ignore less expensive checks
    grid_pos = approximate_to_cell(robot_pos)
    with_island = [] # Result
    for angle, dist in measurements:
#        input()

        # Check if the measurement coincides with the wall behind it at that angle
        actual_angle = facing_angle + 90 - angle
        wall_there = first_cell_of_type(grid_pos, actual_angle, CellType.WALL, grid_map)
        #grid_map.debug_print(print_origin=True)
        #print(wall_there)
        #time.sleep(0.1)
        dif = Position(dist*math.cos(math.radians(actual_angle)),
                       dist*math.sin(math.radians(actual_angle)))
        scanned_pos = Position(robot_pos.x + dif.x,
                               robot_pos.y + dif.y)
        scanned_grid_pos = approximate_to_cell(scanned_pos)
        """print(angle, actual_angle, dist, dif)
        print(grid_pos, wall_there)
        print(scanned_pos, scanned_grid_pos)
        if wall_there is not None:
            print(wall_there)
            print(scanned_grid_pos.dist_to_squared(wall_there))"""
        if (wall_there is not None and
            wall_there != grid_pos and
            scanned_grid_pos.dist_to_squared(wall_there) > 2 + SMALL_FLOAT):
            # More expensive checks
            # Not even sure if this catches anything. I think it should.
            if FAST or passes_through_unknown_before_wall(robot_pos, actual_angle, grid_map):
                #print("ISLAND")
                with_island.append((angle, dist))
    return with_island

def find_island(minimum_measurements:int,
                start_pos:Position, robot_pos:Position, facing_angle:float,
                measurements:list, grid_map:GridMap):
    """
    Return a tuple (angle, dist), which leads to an island, or None, if measurements doesn't indicate that an island exist. An island is considered a wall not in grid_map, which is at least one full tile away from the wall behind it.

    Not tested.

    Args:
        :param minimum_measurements (int): The minimum number of measurements that considers something an island in order for an island to be considered detected.
        :param start_pos (Position): The position in the maze the robot started at.
        :param robot_pos (Position): The current position of the robot, in millimeters.
        :param facing_angle (float): The current facing of the robot, in degrees.
        :param measurements (list): A list of tuples containing (angle, dist), where angle is the angle of the servo in degrees (-90 to +90), and dist is the distance to a wall, in millimeters.
        :param grid_map (GridMap): A GridMap of previously detected walls.

    Returns:
        :return (tuple): A tuple containing (angle, dist), where angle is the angle the robot must turn to face the island, and dist the distance it has to drive to reach it. Or None, if no island is detected.
    """
    with_island = measurements_with_island(start_pos, robot_pos, facing_angle, measurements, grid_map)
    # Did enough measurements think it was an island?
    if len(with_island) >= minimum_measurements:
        # Returns median value - should be about the center of the part of the island we see
        with_island = sorted(with_island, key=lambda e: e[0])
        middle = math.floor(len(with_island)/2)
        return with_island[middle]
    return None
        
    
