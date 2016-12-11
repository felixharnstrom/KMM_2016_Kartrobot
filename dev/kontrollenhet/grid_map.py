
from enum import Enum
from geometry import Position

class CellType(Enum):

    """We don't yet know what's in the cell."""
    UNKNOWN = 0

    """There's open space in the cell."""
    OPEN = 1

    """There's walls around the cell."""
    WALL = 2

    LOCATION = 3

class GridMap:
    """A map representation with, indexed the following way:
    (0,0) (1,0) ...
    (0,1) (1,1)
    ...         ...
    No need to worry about out-of-bounds. Accessing indices
    expands the grid to fit them.

    Attributes:
    :attribute _grid (2D list of CellType): the internal representation of the map
    :attribute _origin (Position): the index of origin, the index that leads to the original (0, 0). Used internally.
    :attribute _width (int): the current width of the map. Do not modify from outside.
    :attribute _height (int): the current height of the map. Do not modify from outside.
    """

    def __init__(self):
        """Constructor."""
        # The starting area as defined in the course spec
        # Robot pos should be (1, 1)
        #self._grid = [[CellType.WALL, CellType.WALL, CellType.WALL],
        #             [CellType.WALL, CellType.OPEN, CellType.WALL],
        #             [CellType.UNKNOWN, CellType.OPEN, CellType.UNKNOWN]]
        #self._width = 3
        #self._height = 3
        # An empty grid:
        self._grid = []
        self._origin = Position(0,0)
        self._width = 0
        self._height = 0

    def _expand_down(self):
        """Add a new row at the bottom."""
        self._height += 1
        self._grid.append([CellType.UNKNOWN for i in range(self._width)])

    def _expand_up(self):
        """Add a new row at the top (y=0)."""
        self._height += 1
        self._grid.insert(0, [CellType.UNKNOWN for i in range(self._width)])
        self._origin.y += 1

    def _expand_right(self):
        """Add a new column to the right."""
        for row in self._grid:
            row.append(CellType.UNKNOWN)
        self._width += 1

    def _expand_left(self):
        """Add a new column to the left."""
        for row in self._grid:
            row.insert(0, CellType.UNKNOWN)
        self._origin.x += 1
        self._width += 1

    def _expand_to_fit(self, x:int, y:int):
        """
        Expands size to fit coordinates.
        Returns coordinates in new dimensions.

        Args:
            :param x (int): The x-position to fit.
            :param y (int): the y-position to fit.

        Returns:
            :return (Position): The offset for converting external coordinates to _grid indices.
        """
        while x >= self._width:
            self._expand_right()
        while y >= self._height:
            self._expand_down()
        xdif = 0
        ydif = 0
        if x < 0:
            xdif = abs(x)
            for i in range(abs(x)):
                self._expand_left()
        if y < 0:
            ydif = abs(y)
            for i in range(abs(y)):
                self._expand_up()
        return Position(xdif, ydif)

    def _pos_to_index(self, x:int, y:int):
        """
        Convert coordinates to index.
        Expands bounds as necessary.
        
        Args:
            :param x (int): The x-position to convert.
            :param y (int): The y-position to convert.

        Returns:
            :return (Position): The coordinates _grid indices.
        """
        xa = x + self._origin.x
        ya = y + self._origin.y
        dif = self._expand_to_fit(xa, ya)
        return Position(xa + dif.x, ya + dif.y)

    def get(self, x:int, y:int):
        """
        Get a cell value. Accepts values outside of bounds; expands map.

        Args:
            :param x (int): The x-position to read.
            :param y (int): The y-position to read.

        Returns:
            :return (CellType): The CellType at the given coordinates
        """
        new_pos = self._pos_to_index(x, y)
        return self._grid[new_pos.y][new_pos.x]

    def set(self, x:int, y:int, ctype:CellType):
        """
        Set a cell value. Accepts values outside of bounds; expands map.

        Args:
            :param x (int): The x-position to read.
            :param y (int): The y-position to read.
            :param ctype (CellType): The CellType to set the cell to.
        """
        new_pos = self._pos_to_index(x, y)
        self._grid[new_pos.y][new_pos.x] = ctype

    def width(self):
        """
        Return the current width of the map.

        Returns:
            :return (int): The current width of the map. Non-negative.
        """
        return self._width

    def height(self):
        """
        Return the current height of the map.

        Returns:
            :return (int): The current height of the map. Non-negative.
        """
        return self._height

    def top_left(self):
        """
        Return the top left index of the grid.

        Returns:
            :return (Position): The top-left external index of the GridMap.
        """
        return Position(-self._origin.x, -self._origin.y)

    def bottom_right(self):
        """
        Return the top left index of the grid.

        Returns:
            :return (Position): The top-left external index of the GridMap.
        """
        return Position(self._width - self._origin.x, self._height - self._origin.y)

    def gui_drawable(self):
        """
        Return a version as the GUI would like to see it.
        
        Returns:
            :return (2D list of ints): A 2D list containing 1's where there are walls, 0 in other places.
        """
        def row_to_drawable(row):
            return [(1 if cell == CellType.WALL else 0) for cell in row]
        return [row_to_drawable(row) for row in self._grid]

    def debug_print(self, print_origin=False):
        """
        Prints a human-readable version of the map.

        Args:
            :argument print_origin (bool): True if (0,0) should be printed as an O, False otherwise.
        """
        for y, row in enumerate(self._grid):
            for x, cell in enumerate(row):
                if (print_origin and
                    self._origin.x == x and
                    self._origin.y == y):
                    print("O", end="")
                elif cell == CellType.UNKNOWN:
                    print("?", end="")
                elif cell == CellType.OPEN:
                    print(" ", end="")
                elif cell == CellType.WALL:
                    print("X", end="")
                elif cell == CellType.LOCATION:
                    print("-", end="")
            print("")
    
    def is_complete(self, start_pos:Position):
        """
        Return True if all cells flooded from start_pos are known, 
        i.e it's impossible to find an unknown cell by moving from there.

        Args:
            :param start_pos (Position): The position to flood from.

        Returns:
            :return (bool): True if it's impossible to reach UNKNOWN from start_pos, False otherwise.

        Raises:
            :exception (ValueError): If the CellType at start_pos is WALL.
        """

        # Check for special case grids
        start_cell_type = self.get(start_pos.x, start_pos.y)
        if start_cell_type == CellType.WALL:
            raise ValueError("The start pos is not allowed to be inside a wall when checking completeness.")
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

