
from enum import Enum

class CellType(Enum):
    UNKNOWN = 0 # We don't know what's in the cell
    OPEN = 1 # There's open space in the cell
    WALL = 2 # There's a wall in the cell

class Position:
    """Position vector."""
    def __init__(self, x:int, y:int):
        self.x = x
        self.y = y

class GridMap:
    """A map representation with, indexed the following way:
    (0,0) (1,0) ...
    (0,1) (1,1)
    ...         ...
    No need to worry about out-of-bounds. Accessing indices
    expands the grid to fit them.

    Members:
    grid:list the internal representation of the map
    robot_pos:Position the position of the robot
    origin:Index of origin. Used internally.
    width:int the current width of the map
    height:int the current height of the map

    It's recommended to access members from methods.
    Reading width/height is fine, but probably unnecessary
    as the grid expands automatically.
    """

    def __init__(self):
        """Constructor."""
        # The starting area as defined in the course spec
        #self.grid = [[CellType.WALL, CellType.WALL, CellType.WALL],
        #             [CellType.WALL, CellType.OPEN, CellType.WALL],
        #             [CellType.UNKNOWN, CellType.OPEN, CellType.UNKNOWN]]
        #self.robot_pos = Position(1,1)
        #self.width = 3
        #self.height = 3
        # An empty grid:
        self.grid = []
        self.origin = Position(0,0)
        self.robot_pos = Position(0,0)
        self.width = 0
        self.height = 0

    def expand_down(self):
        """Add a new row at the bottom."""
        self.height += 1
        self.grid.append([CellType.UNKNOWN for i in range(self.width)])

    def expand_up(self):
        """Add a new row at the top (y=0)."""
        self.height += 1
        self.grid.insert(0, [CellType.UNKNOWN for i in range(self.width)])
        self.origin.y += 1

    def expand_right(self):
        """Add a new column to the right."""
        for row in self.grid:
            row.append(CellType.UNKNOWN)
        self.width += 1

    def expand_left(self):
        """Add a new column to the left."""
        for row in self.grid:
            row.insert(0, CellType.UNKNOWN)
        self.origin.x += 1
        self.width += 1

    def expand_to_fit(self, x:int, y:int):
        """Expands size to fit coordinates.
        Returns coordinates in new dimensions."""
        while x >= self.width:
            self.expand_right()
        while y >= self.height:
            self.expand_down()
        xdif = 0
        ydif = 0
        if x < 0:
            xdif = abs(x)
            for i in range(abs(x)):
                self.expand_left()
        if y < 0:
            ydif = abs(y)
            for i in range(abs(y)):
                self.expand_up()
        return (xdif, ydif)

    def pos_to_index(self, x:int, y:int):
        """Convert coordinates to index.
        Expands bounds as necessary.
        Used internally."""
        xa = x + self.origin.x
        ya = y + self.origin.y
        print("::", xa, ya)
        xdif, ydif = self.expand_to_fit(xa, ya)
        return (xa + xdif, ya + ydif)

    def get_robot_pos(self):
        """Get robot position."""
        return self.robot_pos

    def set_robot_pos(self, x:int, y:int):
        """Set robot pos. Accepts values outside of bounds; expands map."""
        newx, newy = self.pos_to_index(x, y)
        return self.grid[newy][newx]

    def move_robot(self, xdif:int, ydif:int):
        """Move robot. Acceps values outside of bounds; expands map."""
        p = self.robot_pos
        self.set_robot_pos(p.x + xdif, p.y + ydif)

    def get(self, x:int, y:int):
        """Get a cell value. Accepts values outside of bounds; expands map."""
        newx, newy = self.pos_to_index(x, y)
        return self.grid[newy][newx]

    def set(self, x:int, y:int, ctype:CellType):
        """Set a cell value. Accepts values outside of bounds; expands map."""
        newx, newy = self.pos_to_index(x, y)
        self.grid[newy][newx] = ctype

    def get_relative(self, x:int, y:int):
        """Get relative to robot. Accepts values outside of bounds; expands map."""
        return self.get(self.robot_pos.x + x, self.robot_pos.y + y)

    def set_relative(self, x:int, y:int, ctype:CellType):
        """Set relative to robot. Accepts values outside of bounds; expands map."""
        return self.set(self.robot_pos.x + x, self.robot_pos.y + y, ctype)

    def gui_drawable(self):
        """Return a version as the GUI would like to see it."""
        def row_to_drawable(row):
            return [(1 if cell == CellType.WALL else 0) for cell in row]
        return [row_to_drawable(row) for row in self.grid]

    def debug_print(self):
        """Prints a readable version of the map."""
        for y, row in enumerate(self.grid):
            for x, cell in enumerate(row):
                if (self.robot_pos.x + self.origin.x == x and
                    self.robot_pos.y + self.origin.y == y):
                    print("R", end="")
                elif (self.origin.x == x and
                      self.origin.y == y):
                    print("O", end="")
                elif cell == CellType.UNKNOWN:
                    print("?", end="")
                elif cell == CellType.OPEN:
                    print(" ", end="")
                elif cell == CellType.WALL:
                    print("X", end="")
            print("")


"""
# Test
m = Map()
#m.expand_up()
#m.expand_down()
#m.expand_left()
#m.expand_right()
m.set_relative(-3, 0, CellType.WALL)
m.debug_print()
print(m.gui_drawable())
print(m.get_relative(0,0))
"""

