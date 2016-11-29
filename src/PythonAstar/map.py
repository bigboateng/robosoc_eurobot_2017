from astar import AStar
from astar_node import AStarNode
from math import sqrt

"""
This class contains higher level representation of the map.
It contains the obstacles as well as the locations of points of interest, e.g. location of cylinders, base, etc.
"""
class Map(object):

    def __init__(self, graph, nodes, width, height):
        self.graph = graph  # static representation of the map
        self.nodes = nodes
        self.colour = 0  # blue by default
        self.width = width
        self.height = height
        self.dynamic_objects = {"robot": nodes[0][0]}  # stores the position of anything that moves on the map

    #  number = 0 for blue and number = 1 for yellow
    def set__side(self, bit):
        self.colour = bit

    def update_position(self, new_pos):
        self.pri_robot_pos[self.colour] = new_pos

    def get_path_to(self, location):
        if location is not AStarNode:
            loc = self.nodes[location.y][location.x]
        else:
            loc = location

        astar = AStar(self.graph)
        path = astar.search(self.dynamic_objects["robot"], loc)
        return path

    def add_obstacle(self, location):
        self.add_obstacle(location, 0)

    def add_obstacle(self, location, radius):
        if location is not AStarNode:
            loc = self.nodes[location.y][location.x]
        else:
            loc = location

        for i in range(loc.x-radius, loc.x+radius):
            for j in range(loc.y-radius, loc.y+radius):
                if not (0 <= loc.x + i < self.width):
                    continue
                if not (0 <= loc.y + j < self.height):
                    continue
                if not (sqrt(i**2 + j**2) < radius):
                    continue
                self.graph[self.node[j][i]] = []

    def generalise(self, path):
        generalised_path = [path[0]]
        i = 0
        j = 1
        while j < len(path):
            v = generalised_path[i]
            while j < len(path) and self.can_draw_line(v, path[j]):
                j += 1
            generalised_path.append(path[j-1])
            i += 1
        return generalised_path

    def can_draw_line(self, v, w):
        distance = 99999
        for i in range(min(v.x, w.x), max(v.x, w.x) + 1):
            for j in range(min(v.y, w.y), max(v.y, w.y) + 1):
                if self.nodes[i][j].obstacle:
                    distance = min(self.distance_to_line(v, w, self.nodes[i][j]), distance)
                    if distance < 0.7:
                        return False
        return distance > 0.7

    def distance_to_line(self, v, w, n):
        return abs((w.y-v.y)*n.x-(w.x-v.x)*n.y+w.x*v.y-w.y*v.x)/sqrt((w.x-v.x)**2 + (w.y-v.y)**2)

