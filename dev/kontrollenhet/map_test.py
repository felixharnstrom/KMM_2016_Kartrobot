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
#coordinates = read_debug_data('demo_data/triple_sided_wall_with_imperfections.json')
#coordinates = [(0, 0)]
coordinates = [(1300, 1300), (-600, -600)]

top_left = top_left_grid_index(coordinates)
bottom_right = bottom_right_grid_index(coordinates)

#lines = coordinates_to_lines(coordinates)

lines = [Line(Position(0, -400), Position(400, -400)), Line(Position(400, -400), Position(400, 0)), Line(Position(400, 0), Position(800, 0)), Line(Position(800, 0), Position(800, 400)), Line(Position(800, 400), Position(800, 800)), Line(Position(800, 800), Position(1200, 800)), Line(Position(800, 1200), Position(1200, 1200)), Line(Position(400, 1200), Position(800, 1200)), Line(Position(0, 1200), Position(400, 1200)), Line(Position(-400, -400), Position(0, -400)), Line(Position(-400, -400), Position(-400, 0)), Line(Position(-400, 0), Position(-400, 400)), Line(Position(-400, 400), Position(-400, 800)), Line(Position(-400, 800), Position(-400, 1200))]

for line in lines:
    print("line(", line.start.x, line.start.y, line.end.x, line.end.y, ")")

get_grid_map(coordinates, lines, robot_pos, grid_map)

grid_map.debug_print()

"""
for y in range(top_left.y, bottom_right.y):
    for x in range(top_left.x, bottom_right.x + 1):
        print("Grid(", grid_map.get(x, y), x, y, ")")

grid = check_available_grid(grid_map, coordinates)

print(grid)
"""

#debug_plot(coordinates, lines)
