from collections import namedtuple

class Obstacle:
    """
    Defines static obstacles in the simulation.
    """

    def __init__(self):
        """
        Constructs static obstacles in the simulation.

        Args:
            next_ (Obstacle)
            previous_ (Obstacle)
            direction_ (Vector2)
            point_ (Vector2)
            id_ (int)
            convex_ (bool)
        """
        self.next_ = None
        self.previous_ = None
        self.direction_ = None
        self.point_ = None
        self.id_ = -1
        self.convex_ = False
