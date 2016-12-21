from grid_map import *
import random
import json
from map import *

grid_map = GridMap()
robot_pos = Position(600,200)
robot_approx = approximate_to_cell(robot_pos)
print(robot_approx)



for i in range(0,17):
    for j in range(0,17):
        rand = random.randint(0,1)
        if rand == 0:
            grid_map.set(i,j,CellType.OPEN)
        else:
            grid_map.set(i,j,CellType.OPEN)

grid_map.debug_print()


print(grid_map.gui_drawable(robot_approx.x, robot_approx.y))



#s = json.dumps(grid_map.gui_drawable(0,0))
#print(json.loads(s))
