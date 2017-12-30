class Obstacle:
    """
    Defines static obstacles in the simulation.
    """

    def __init__(self):
        self.next_ = None
        self.previous_ = None
        self.direction_ = None
        self.point_ = None
        self.id_ = -1
        self.convex_ = False
