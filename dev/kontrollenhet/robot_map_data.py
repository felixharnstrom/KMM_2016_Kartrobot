import threading
import copy

map_lock = threading.Lock()
grid_map_output = [[0,0],[]]
robot_distance = 0

def get_grid_map():
    global map_lock
    map_lock.acquire()
    gmap = copy.deepcopy(grid_map_output)
    map_lock.release()
    return gmap

def set_grid_map(gmap):
    global map_lock
    global grid_map_output
    map_lock.acquire()
    grid_map_output = gmap
    map_lock.release()

