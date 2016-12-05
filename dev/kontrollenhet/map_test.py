from map import *
from geometry import *
import matplotlib.pyplot as plt

# robot position
robot_pos = Position(0,0)

# robot angle
angle = 0

lines = []
possible_paths = []

grid_map = GridMap()

grid = []

#coordinates = convert_to_coordinates(measure_lidar(), robot_pos, angle)

#coordinates = convert_to_coordinates(read_debug_data('demo_data/perfect_square_center_raw_data.json'), robot_pos, angle)
coordinates = read_debug_data('demo_data/triple_sided_wall_with_imperfections.json')

top_left = top_left_grid_index(coordinates)
bottom_right = bottom_right_grid_index(coordinates)

lines = coordinates_to_lines(coordinates, robot_pos, grid_map)



for line in lines:
    print("line(", line.start.x, line.start.y, line.end.x, line.end.y, ")")

for y in range(top_left.y, bottom_right.y):
    for x in range(top_left.x, bottom_right.x + 1):
        print("Grid(", grid_map.get(x, y), x, y, ")")

grid = check_available_grid(grid_map, coordinates)

print(grid)

debug_plot(coordinates, lines)