
class Size:
    """Size vector."""

    def __init__(self, w:int, h:int):
        self.w = w
        self.h = h

    def __str__(self):
        return str(self.__dict__)

    def __eq__(self, other): 
        return self.__dict__ == other.__dict__

    def __ne__(self, other): 
        return not self == other

class Position:
    """Position vector."""

    def __init__(self, x:int, y:int):
        self.x = x
        self.y = y

    def __str__(self):
        return str(self.__dict__)

    def __eq__(self, other): 
        return self.__dict__ == other.__dict__

    def __ne__(self, other): 
        return not self == other

    def difference(other):
        return Size(other.x - self.x, other.y - self.y)

class Line:
    """2D line."""
    def __init__(self, start:Position, end:Position):
        self.start = start
        self.end = end
    
    def __str__(self):
        return str(self.__dict__)

    def __eq__(self, other): 
        return self.__dict__ == other.__dict__

    def __ne__(self, other): 
        return not self == other
