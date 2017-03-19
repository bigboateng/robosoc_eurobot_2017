from astar import AStar
from astar_node import AStarNode
from math import sqrt
import time

"""
This class contains higher level representation of the map.
It contains the obstacles as well as the locations of points of interest, e.g. location of cylinders, base, etc.
It also provides a path finding algorithm
"""
class Map(object):

    def __init__(self, graph, nodes):
        self.graph = graph  # static representation of the map
        self.nodes = nodes
        self.width = len(nodes[0])
        self.height = len(nodes)
        self.obstacles = {}

    # turns a pair into an astar node
    def get_node(self, location):
        if location is not AStarNode:
            return self.nodes[location[1]][location[0]]
        else:
            return location

    # returns a path in the form of points from source to destination
    def get_path(self, source, destination):
        s = get_node(source)
        d = get_node(destination)
        astar = AStar(self.graph)
        path = astar.search(s, d)
        return path

    # add obstacles of set radius to the map, 
    def add_obstacle(self, location, radius = 25):
        loc = get_node(location)
        self.obstacles[time.time()] = (location, radius)
        for i in range(loc.x-radius, loc.x+radius):
            for j in range(loc.y-radius, loc.y+radius):
                if not (0 <= loc.x + i < self.width):
                    continue
                if not (0 <= loc.y + j < self.height):
                    continue
                if not (sqrt(i**2 + j**2) < radius):
                    continue
                self.graph[self.node[j][i]] = []

    # update obstacles, if they've been longer than 15 seconds on the map then remove them from the list
    def update_obstacles(self):
        now = time.time()
        for timestamp in self.obstacles.keys():
            if now - timestamp > 15:
                self.remove_obstacle(timestamp)

    def remove_obstacle(self, timestamp):
        (loc, radius) = self.obstacles[timestamp]
        for i in range(loc.x-radius, loc.x+radius):
            for j in range(loc.y-radius, loc.y+radius):
                for x, y in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                    if not (0 <= x + i < self.width):
                        continue
                    if not (0 <= y + j < self.height):
                        continue
                    if not (sqrt(i**2 + j**2) < radius):
                        continue
                    if nodes[x+i][y+j] not in self.graph[self.nodes[j][i]]:
                        self.graph[self.nodes[j][i]].append(self.nodes[x+i][y+j])
        del self.obstacles[timestamp]

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

