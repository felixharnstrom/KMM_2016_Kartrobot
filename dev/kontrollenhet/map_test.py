from map import *
import matplotlib.pyplot as plt

# robot position
x = 0
y = 0

# robot angle
angle = 0

lines = []

map_room(x,y,angle, lines)
print("lines",lines)

print(lines)
print(check_available_grid(lines))

get_grid_map(x, y, lines)

plt = plot_room(lines)

plt.show()