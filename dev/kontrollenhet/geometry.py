
class Size:
    """Size vector."""

    def __init__(self, w:int, h:int):
        self.w = int(w)
        self.h = int(h)

    def __str__(self):
        return str(self.__dict__)

    def __eq__(self, other): 
        return self.__dict__ == other.__dict__

    def __ne__(self, other): 
        return not self == other

    def add(self, other):
        self.w += other.w
        self.h += other.h

class Position:
    """Position vector."""

    def __init__(self, x:int, y:int):
        self.x = int(x)
        self.y = int(y)

    def __str__(self):
        return str(self.__dict__)

    def __eq__(self, other): 
        return self.__dict__ == other.__dict__

    def __ne__(self, other): 
        return not self == other

    def difference(self, other):
        return Size(self.x - other.x, self.y - other.y)

class Line:
    """2D line."""
    def __init__(self, start:Position, end:Position):
        self.start = start
        self.end = end
    
    def __str__(self):
        return "{'start': " + self.start.__str__() + ", 'end': " + self.end.__str__() + "}"

    def __eq__(self, other): 
        return self.start == other.start and self.end == other.end

    def __ne__(self, other): 
        return not self == other
