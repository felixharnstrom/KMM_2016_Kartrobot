from map import *
from geometry import *
import matplotlib.pyplot as plt
# robot position
robot_pos = [Position(201,236), Position(201,1027), Position(-1049,1027), Position(-1049,1001), Position(-692,1001), Position(-641,261), Position(-590,261), Position(-590,1026)]
# robot angle
#angle = [0, 0, -90, -180, -270, -180, -270, -360]
#angle = [0, -90, -180, -180, -180, -270, -270, -270]
angle = [0, 0-90, -90-90, -180-90, -270+90-90, -180-90, -270-90, -360+90]

data = read_debug_data('demo_data/scan.json')


lines = []
possible_paths = []
grid_map = GridMap()
grid = []
nr = 0
all_lines = []
all_coordinates = []
new_angle = 0

for mesure in data:
    if nr==1: #or nr==1 or nr==3 or nr==4 or nr==5 or nr==6: #2 and 7 have really bad data, too much noise
        coordinates = convert_to_coordinates(mesure, robot_pos[nr], angle[nr])
        lines = coordinates_to_lines(coordinates, angle[nr])
        for line in lines:
            all_lines.append(line)
        for pos in coordinates:
            all_coordinates.append(pos)

        #debug_plot(coordinates,lines)
    nr += 1


# n=1
#
# coordinates = convert_to_coordinates(data[n], robot_pos[n], (angle[n]))
# lines = coordinates_to_lines(coordinates)
# debug_plot(coordinates,lines)

debug_plot(all_coordinates, all_lines)


#coordinates = convert_to_coordinates(measure_lidar(), robot_pos, angle)

#coordinates = convert_to_coordinates(read_debug_data('demo_data/perfect_square_center_raw_data.json'), robot_pos, angle)
#coordinates = read_debug_data('demo_data/triple_sided_wall_with_imperfections.json')
#coordinates = [(0, 0)]
#coordinates = [(1300, 1300), (-600, -600)]

#top_left = top_left_grid_index(coordinates)
#bottom_right = bottom_right_grid_index(coordinates)

#lines = coordinates_to_lines(coordinates)

#lines = [Line(Position(0, -400), Position(400, -400)), Line(Position(400, -400), Position(400, 0)), Line(Position(400, 0), Position(800, 0)), Line(Position(800, 0), Position(800, 400)), Line(Position(800, 400), Position(800, 800)), Line(Position(800, 800), Position(1200, 800)), Line(Position(800, 1200), Position(1200, 1200)), Line(Position(400, 1200), Position(800, 1200)), Line(Position(0, 1200), Position(400, 1200)), Line(Position(-400, -400), Position(0, -400)), Line(Position(-400, -400), Position(-400, 0)), Line(Position(-400, 0), Position(-400, 400)), Line(Position(-400, 400), Position(-400, 800)), Line(Position(-400, 800), Position(-400, 1200))]

#for line in lines:
#    print("line(", line.start.x, line.start.y, line.end.x, line.end.y, ")")


#debug_plot(coordinates, lines)
#test = []


"""
for y in range(top_left.y, bottom_right.y):
    for x in range(top_left.x, bottom_right.x + 1):
        print("Grid(", grid_map.get(x, y), x, y, ")")

grid = check_available_grid(grid_map, coordinates)

print(grid)
"""

#debug_plot(coordinates, lines)
