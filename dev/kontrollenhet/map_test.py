from map import *

# robot position
x = 0
y = 0

# robot angle
angle = 0

plt.plot([x],[y], '.')

plot_room([x,y,angle], plt)

plt.show()