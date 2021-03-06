from enum import Enum
import math


class Direction(Enum):
    LEFT = 0
    RIGHT = 1
    DOWN = 2
    UP = 3
    NONE = 4


class Size:
    """
    Size vector.

    Attributes:
        :attribute w (int): Width.
        :attribute h (int): Height.

    Args:
        :param w (int): Width.
        :param h (int): Height.
    """

    def __init__(self, w:int, h:int):
        self.w = int(w)
        self.h = int(h)

    def __str__(self):
        return str(self.__dict__)

    def __eq__(self, other):
        return self.__dict__ == other.__dict__

    def __ne__(self, other): 
        return not self == other

    def __hash__(self):
        return hash(repr(self))

    def add(self, other):
        """
        Add a size to this one.

        Args:
            :param other (Size): The size to add to this one.
        """
        self.w += other.w
        self.h += other.h

class Position:
    """
    Size vector.

    Attributes:
        :attribute x (int): X position.
        :attribute y (int): Y position

    Args:
        :param x (int): X position.
        :param y (int): Y position
    """

    def __init__(self, x:int, y:int, direction: Direction = Direction.NONE):
        self.x = round(x)
        self.y = round(y)
        self.direction = direction

    def __str__(self):
        return str(self.__dict__)

    def __eq__(self, other): 
        return self.x == other.x and self.y == other.y


    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
#        return self.__dict__ == other.__dict__


    def __ne__(self, other): 
        return not self == other

    def __hash__(self):
        return hash(repr(self))


    def dist_to_squared(self, other):
        """
        Return euclidian distance squared to other Position.

        Args:
            :param other(Position): Pos to measure to

        Returns:
            :return (float): distance squared.
        """
        dx = self.x - other.x
        dy = self.y - other.y
        return dx*dx + dy*dy

    def dist(self, other):
        """
        Return euclidian distance squared to other Position.

        Args:
            :param other(Position): Pos to measure to

        Returns:
            :return (float): distance squared.
        """
        return math.sqrt(self.dist_to_squared(other))
        
    def difference(self, other):
        """
        Return the difference in x-position and y-position.

        Args:
            :param other (Position): The position to compare to.

        Returns:
            :return (Size): The difference in x- and y-position.
        """
        return Size(self.x - other.x, self.y - other.y)

class Line:
    """
    2D line.
    Wether a point is its start or end point indicates nothing; they are interchangeable.

    Attributes:
        :attribute start (Position): Starting point of Line.
        :attribute end (Position): End point of Line.

    Args:
        :param start (Position): Starting point of Line.
        :param end (Position): End point of Line.
    """

    def __init__(self, start:Position, end:Position):
        self.start = start
        self.end = end
    
    def __str__(self):
        return "{'start': " + self.start.__str__() + ", 'end': " + self.end.__str__() + "}"

    def __eq__(self, other): 
        return self.start == other.start and self.end == other.end

    def __ne__(self, other): 
        return not self == other
