
from geometry import *
from robot_communication import *
import json
import math, time
from grid_map import GridMap, CellType

"""The real-world size of a grid cell, in millimeters. Also the length of a line."""
CELL_SIZE = 400

"""The sleep time when moving the servo to 0."""
LIDAR_INITIAL_SLEEP = 1.5

"""The sleep time when moving the servo 1 degree."""
LIDAR_SLEEP = 0.015


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



def polar_projection(dist:float, angle:float):
    """
    Convert angle and distance to position.
    Args:
        :param dist (float): The distance.
        :param angle (float): The angle.
    Returns:
        :return (Position): The position.
    """
    return Position(dist*math.cos(math.radians(angle)),
                    dist*math.sin(math.radians(angle)))

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

    handle_command(Command.servo(180))
    return measurements


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
    # print ("LINES",len(lines))
    for line in lines:
        start = approximate_to_cell(line.start, resolution)
        end = approximate_to_cell(line.end, resolution)
        between = bresenham(Line(start, end))
        # print (line.start, line.end)
        if line.end.x < line.start.x:
            mark_all_cells_direction(between, Direction.LEFT)
        if line.end.x > line.start.x:
            mark_all_cells_direction(between, Direction.RIGHT)
        if line.end.y < line.start.y:
            mark_all_cells_direction(between, Direction.UP)
        if line.end.y > line.start.y:
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
        change = polar_projection(current_distance, angle)
        points_x.append(points_x[-1] + change.x)
        points_y.append(points_y[-1] + change.y)

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


def make_open(cells, grid_map:GridMap):
    """
    Set all cell indices given to OPEN.

    Args:
        :param cells (list of Position): The cell indices to set.
        :param grid_map (GridMap): The GridMap to modify.
    """
    for cell in cells:
        grid_map.set(cell.x, cell.y, CellType.OPEN)

        
def set_to_wall_if_unknown(x, y, grid_map:GridMap):
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
        
        # print (open_cells[i].direction)
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
        if open_cells[i].direction == Direction.NONE and i > 0:
            # Fixes cul-de-sacs
            prev_dir = open_cells[i-1].direction
            if prev_dir == Direction.RIGHT:
                set_to_wall_if_unknown(start.x+1, start.y, grid_map)
            if prev_dir == Direction.LEFT:
                set_to_wall_if_unknown(start.x-1, start.y, grid_map)
            if prev_dir == Direction.DOWN:
                set_to_wall_if_unknown(start.x, start.y+1, grid_map)
            if prev_dir == Direction.UP:
                set_to_wall_if_unknown(start.x, start.y-1, grid_map)
        # Now available in animation!
        #grid_map.debug_print()
        #time.sleep(0.1)
        
                
            

def median_sensor(it : int, sensor_instr : Command):
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


def island_exists(grid_pos:Position, measuring_angle:float, grid_map:GridMap):
    """
    Check if an island exists .
    There's definetly an island if it returns not None. It could still be one there
      if we get None - we just can't determine right now.

    Args:
        :robot_pos (Position): The current position of the robot, in 40x40cm.
        :measuring_angle (float): The angle we're measuring in.
        :grid_map (GridMap): A grid_map containing all outer wall tiles.

    Returns:
        :return (bool, float): The bool is True iff we are sure 
    """

    # Check what multiple of 90 degrees we're facing
    dir_i = math.floor((measuring_angle % 360) / 90)

    # Convert that to a normalized direction vector
    direction = None
    if dir_i == 0:
        direction = Position(1, 0)
    elif dir_i == 1:
        direction = Position(0, 1)
    elif dir_i == 2:
        direction = Position(-1, 0)
    else:
        direction = Position(0, -1)

    cur_pos = grid_pos
    distance = 0
    has_found_unknown = False

    # Search for a wall in the measuring direction
    # distance will be the distance to that wall, in mm
    while (grid_map.is_within_bounds(cur_pos.x, cur_pos.y) and
           grid_map.get(cur_pos.x, cur_pos.y) != CellType.WALL):

        if grid_map.get(cur_pos.x, cur_pos.y) == CellType.UNKNOWN:
            has_found_unknown = True
        cur_pos.x += direction.x
        cur_pos.y += direction.y
        distance += CELL_SIZE

    # Check if we exited because we found a wall
    has_found_wall = grid_map.get(cur_pos.x, cur_pos.y) == CellType.WALL

    # Measure lidar
    MEASUREMENT_ITERATIONS = 10
    lidar_dist = median_sensor(MEASUREMENT_ITERATIONS, Command.read_lidar())

    if not has_found_wall:
        # There is no wall in that direction =>
        #   we can't determine if there's an island there
        return False, lidar_dist
    elif not has_found_unknown:
        # We didn't pass any UNKNOWN's =>
        #   There's no room for an island there.
        return False, lidar_dist
    # If the wall is at least a tile away from what we measured,
    #   we should either have discovered it when reading outer walls,
    #   or it is an island.
    # Thus, unless the grid_map is wrong, we've definetly found our island.
    elif distance < lidar_dist + CELL_SIZE:
        return False, lidar_dist
    else:
        return True, lidar_dist
    
