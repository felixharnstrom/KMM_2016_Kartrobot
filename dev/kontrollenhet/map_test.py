from map import *

# robot position
x = 0
y = 0

# robot angle
angle = 0

plt.plot([x],[y], '.')

plot_room([x,y,angle], plt, 'file')

#plot_room([x, y, angle], plt, 'file', 'demo_data/triple_sided_wall_with_imperfections.json')

plt.show()