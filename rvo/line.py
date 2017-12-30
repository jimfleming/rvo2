from .vector import Vector2


class Line:
    """
    Defines a directed line.
    """

    def __init__(self, direction=None, point=None):
        """
        Constructs a directed line.

        Args:
            direction (Vector2)
            point (Vector2)
        """
        self.direction = direction if direction is not None else Vector2()
        self.point = point if point is not None else Vector2()
