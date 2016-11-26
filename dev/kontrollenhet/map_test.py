from map import *
import matplotlib.pyplot as plt

# robot position
x = 0
y = 0

# robot angle
angle = 0

map = []

map_room(x,y,angle, map)
print ("map",map)

print(map)
print(check_available_grid(map))

plt = plot_room(map)

plt.show()