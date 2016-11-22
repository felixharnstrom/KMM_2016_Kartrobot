from map import *
import matplotlib.pyplot as plt
import numpy as np

# robot position
x = 0
y = 0

# robot angle
angle = 0

plt.plot([x],[y], '.')

map = []

size = map_room([x,y,angle], map)

for coord in map:
    if coord[1] == coord[3]:
        line = np.linspace(coord[0], coord[2], POINTS)
        plt.plot(line, [coord[1]] * POINTS)
    if coord[0] == coord[2]:
        line = np.linspace(coord[1], coord[3], POINTS)
        plt.plot([coord[0]] * POINTS, line)
plt.plot([(size[0]-1)*GRID_SIZE, (size[1])*GRID_SIZE], [(size[2]-1)*GRID_SIZE, (size[3])*GRID_SIZE], '.')
plt.show()