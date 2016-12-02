from map import *
import matplotlib.pyplot as plt

# robot position
x = 0
y = 0

# robot angle
angle = 0

lines = []

grid_map = GridMap()

map_room(x,y,angle, grid_map)
print("lines",lines)

print(lines)
print(check_available_grid(lines))

#get_grid_map(x, y, lines)

plt = plot_room(lines)

plt.show()