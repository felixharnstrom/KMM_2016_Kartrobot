import threading
import copy

""" Lock for reading setting grid_map. """
map_lock = threading.Lock()

""" The global grid_map. """
grid_map_output = [[0,0],[]]

""" The driven distance for the robot. """
robot_distance = 0

def get_grid_map():
	""" 
	Reads the grid_map with a lock functionality and returns the grid_map. 
	
	Returns:
		:return (2D list): A copy of the global grid_map.
	"""
    global map_lock
    map_lock.acquire()
    gmap = copy.deepcopy(grid_map_output)
    map_lock.release()
    return gmap

def set_grid_map(gmap):
	""" 
	Sets the grid_map with a lock functionality.
	
	Args:
		:param gmap		(2D list): The new grid_map to update the global to.
	"""
    global map_lock
    global grid_map_output
    map_lock.acquire()
    grid_map_output = gmap
    map_lock.release()

