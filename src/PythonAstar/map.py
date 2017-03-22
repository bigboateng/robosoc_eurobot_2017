from astar import AStar, AStarNode
from exception import *
from math import sqrt
import time

"""
This class contains higher level representation of the map.
It contains the obstacles as well as the locations of points of interest, e.g. location of cylinders, base, etc.
It also provides a path finding algorithm
"""
class Map(object):

    def __init__(self, array):
        self.array = array  # static representation of the map
        self.width = len(array[0])
        self.height = len(array)
        self.obstacles = {}
        self.mapfactor = 10  # map factor is the number of small squares per Astar node 
        self.allow = self.height / (self.mapfactor * 4)  # allowance which states the extra border around the smaller map.
        self.nodes = [[AStarNode(x, y) for x in range(self.width/self.mapfactor)] for y in range(self.height/self.mapfactor)]
        print("nodes", len(self.nodes[0]), len(self.nodes))
        self.graph = {}

    def withinArea(self, source, destination):
        print("Within Area", self.width, self.height)
        if (0 <= source[0] < self.width):
            return True    
        if (0 <= source[1] < self.height):
            return True
        if (0 <= destination[0] < self.width):
            return True
        if (0 <= destination[1] < self.height):
            return True
        return False

    # Coordinates in relation to the AStar coordinates
    def getAstarCoords(self, source, destination):
        factor = self.mapfactor
        a_src_x = int(source[0] / factor)
        a_src_y = int(source[1] / factor)
        a_dist_x = int(destination[0] / factor)
        a_dist_y = int(destination[1] / factor)
        return a_src_x, a_src_y, a_dist_x, a_dist_y

    def getAstarMapBoundary(self, a_src_x, a_src_y, a_dist_x, a_dist_y):
        allow = self.allow
        factor = self.mapfactor
        min_x = max(0, (min(a_src_x, a_dist_x) - allow))
        max_x = min(self.width/factor, (max(a_src_x, a_dist_x) + allow))
        min_y = max(0, (min(a_src_y, a_dist_y) - allow))
        max_y = min(self.height/factor, (max(a_src_y, a_dist_y) + allow))
        return min_x , max_x, min_y, max_y
    
    # Creates an AStarNodes map round the source and destination
    def make_graph(self, source, destination):
        print(source, destination)
        factor = self.mapfactor  # cells per AStarNode
        allow = self.allow  # allowance which states the extra border around the smaller map.
        nodes = self.nodes # a star nodes
        graph = self.graph

        if not self.withinArea(source, destination):
            raise CoordinatesOutsideAreaException

        a_src_x, a_src_y, a_dist_x, a_dist_y = self.getAstarCoords(source, destination)
        min_x, max_x, min_y, max_y = self.getAstarMapBoundary(a_src_x, a_src_y, a_dist_x, a_dist_y)
        print("AStar Coords", a_src_x, a_src_y, a_dist_x, a_dist_y)
        print("AStar Min Max x and then y", min_x, max_x, min_y, max_y)

        graph = {}
        
        for x in range(len(nodes[0])):
            for y in range(len(nodes)):
                graph[nodes[y][x]] = set()
        
        for a_x in range(min_x, max_x):
            for a_y in range(min_y, max_y):
                x = a_x*factor # coresponds to the x and y on the main map (not the A star)
                y = a_y*factor 

                # count the number of obstacles if more than factor number treat as obstacle in a-star
                count = 0
                for i in range(factor):
                    for j in range(factor):
                        if self.array[y+j][x+i] is 1:  # obstacle 
                            print("obstacle")
                            count += 1
                        if count > factor:
                            nodes[a_y][a_x].obstacle = True
                            print(self.array[y+j][x+i], a_x, a_y, "is obstacle")
                            break

                # add neighbours to the node if not an obstacle
                print(a_y, a_x)
                if not nodes[a_y][a_x].obstacle:
                    for i, j in [(0, -1), (-1, 0)]:
                        if not (min_x <= a_x + i < max_x-1):
                            continue
                        if not (min_y <= a_y + j < max_y-1):
                            continue
                        if not nodes[a_y+i][a_x+i].obstacle:
                            graph[nodes[a_y+j][a_x+i]].add(nodes[ a_y ][ a_x ])
                            graph[nodes[ a_y ][ a_x ]].add(nodes[a_y+j][a_x+i])
                else:
                    print(a_x, a_y, "is obstacle")
        print(" src " + str(a_src_x) + " " + str(a_src_y) )
        print(" dest " + str(a_dist_x) + " " + str(a_dist_y) )
        return graph, nodes, nodes[a_src_y][a_src_x], nodes[a_dist_y][a_dist_x]

    def generalise(self, path):
        print("Generalising path")
        generalised_path = [path[0]]
        i = 0
        j = 1
        while j < len(path):
            v = generalised_path[i]
            print(v)
            while j < len(path):
                j += 1
            generalised_path.append(path[j-1])
            i += 1
        return generalised_path

    def can_draw_line(self, v, w):
        distance = 99999
        for i in range(min(v[0], w[0]), max(v[0], w[0]) + 1):
            for j in range(min(v[1], w[1]), max(v[1], w[1]) + 1):
                if self.array[i][j] > 0 :
                    distance = min(self.distance_to_line( v[0], v[1], w[0], w[1], i, j), distance)
                    if distance < 0.7:
                        return False
        return distance > 0.7

    def distance_to_line(self, vx, vy, wx, wy, nx, ny):
        return abs((wy-vy)*nx-(wx-vx)*ny+wx*vy-wy*vx)/sqrt((wx-vx)**2 + (wy-vy)**2)

    # returns a path in the form of points from source to destination
    def get_path(self, source, destination):
        graph, nodes, s, d = self.make_graph(source, destination)
        astar = AStar(graph, nodes)
        path = astar.search(s, d)

        if path is None:
            raise PathNotFoundException

        path_in_context = []
        for n in range(len(path)):
            if n == 0:
                point = (source[0], source[1])
            elif n == len(path) - 1:
                point = (source[0], source[1])
            else:
                point = (path[n].x*self.mapfactor, path[n].y*self.mapfactor)
            path_in_context.append(point)

        print("path length",len(path_in_context))
        if len(path_in_context) < 3:
            return generalised, path_in_context
        generalised = self.generalise(path_in_context)

        if generalised is None:
            raise PathNotFoundException

        return generalised, path_in_context

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
                self.graph[self.node[i][j]] = []

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

