
from enum import Enum
from geometry import Position

class CellType(Enum):
    UNKNOWN = 0 # We don't know what's in the cell
    OPEN = 1 # There's open space in the cell
    WALL = 2 # There's a wall in the cell

class GridMap:
    """A map representation with, indexed the following way:
    (0,0) (1,0) ...
    (0,1) (1,1)
    ...         ...
    No need to worry about out-of-bounds. Accessing indices
    expands the grid to fit them.

    Members:
    grid:list the internal representation of the map
    origin:Position the index of origin. Used internally.
    width:int the current width of the map
    height:int the current height of the map

    It's recommended to access members from methods.
    Reading width/height is fine, but probably unnecessary
    as the grid expands automatically.
    """

    def __init__(self):
        """Constructor."""
        # The starting area as defined in the course spec
        # Robot pos should be (1, 1)
        #self.grid = [[CellType.WALL, CellType.WALL, CellType.WALL],
        #             [CellType.WALL, CellType.OPEN, CellType.WALL],
        #             [CellType.UNKNOWN, CellType.OPEN, CellType.UNKNOWN]]
        #self.width = 3
        #self.height = 3
        # An empty grid:
        self.grid = []
        self.origin = Position(0,0)
        self.width = 0
        self.height = 0

        self.bottom_left = Position(0,0)
        self.top_right = Position(0,0)

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
        xdif, ydif = self.expand_to_fit(xa, ya)
        return (xa + xdif, ya + ydif)

    def get(self, x:int, y:int):
        """Get a cell value. Accepts values outside of bounds; expands map."""
        newx, newy = self.pos_to_index(x, y)
        return self.grid[newy][newx]

    def set(self, x:int, y:int, ctype:CellType):
        """Set a cell value. Accepts values outside of bounds; expands map."""
        newx, newy = self.pos_to_index(x, y)
        self.grid[newy][newx] = ctype

    def gui_drawable(self):
        """Return a version as the GUI would like to see it."""
        def row_to_drawable(row):
            return [(1 if cell == CellType.WALL else 0) for cell in row]
        return [row_to_drawable(row) for row in self.grid]

    def debug_print(self):
        """Prints a readable version of the map."""
        for y, row in enumerate(self.grid):
            for x, cell in enumerate(row):
                if (self.origin.x == x and
                      self.origin.y == y):
                    print("O", end="")
                elif cell == CellType.UNKNOWN:
                    print("?", end="")
                elif cell == CellType.OPEN:
                    print(" ", end="")
                elif cell == CellType.WALL:
                    print("X", end="")
            print("")
    
    def is_complete(self, start_pos:Position):
        """Return True if all cells flooded from start_pos are known, 
        i.e it's impossible to find an unknown cell by moving from there."""

        # Check for special case grids
        start_cell_type = self.get(start_pos.x, start_pos.y)
        if start_cell_type == CellType.WALL:
            raise Exception("The start pos is not allowed to be inside a wall when checking completeness.")
        if start_cell_type == CellType.UNKNOWN:
            # Something's fucky; Shouldn't happen unless members are messed with.
            # Still, the map is certainly not complete.
            return False 

        # DFS
        visited = set() 
        queue = [(start_pos.x, start_pos.y)]
        while queue: # If not empty
            cell = queue.pop()
            if cell not in visited:
                x, y = cell
                visited.add(cell)
                neighbours = ((x+1,y), (x-1,y), (x,y+1), (x,y-1))
                if self.get(x, y) == CellType.UNKNOWN:
                    return False
                for neighbour in neighbours:
                    if self.get(neighbour[0], neighbour[1]) != CellType.WALL:
                        queue.append(neighbour)
        # We've searched the whole thing witout finding any UNKNOWN's
        return True

