import sys


class AStarNode(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.h = 0  # heuristic cost
        self.g = sys.maxsize  # move cost
        self.obstacle = False
        self.parent = None

    def move_cost(self, other):
        diagonal = abs(self.x - other.x) == 1 and abs(self.y - other.y) == 1
        return 14 if diagonal else 10  # fourteen comes from 1.414 = sqrt(2) for diagonals

    def reset_cost(self):
        self.g = sys.maxsize
