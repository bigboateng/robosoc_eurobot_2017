from math import sqrt
import sys

class AStarNode(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.h = 0  # heuristic cost
        self.g = 0  # move cost
        self.obstacle = False
        self.parent = None

    def move_cost(self, other):
        diagonal = abs(self.x - other.x) == 1 and abs(self.y - other.y) == 1
        return 14 if diagonal else 10  # fourteen comes from 1.414 = sqrt(2) for diagonals

    def reset_cost(self):
        self.g = sys.maxsize

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, other):
        return (self.x, self.y) == (other.x, other.y)

    def __ne__(self, other):
        return not(self == other)

    def __repr__(self):
        return '(%d %d)' % (self.x, self.y)

class AStar(object):
    def __init__(self, graph, nodes):
        self.graph = graph
        self.nodes = nodes

    def heuristic(self, node, start, end):
        return sqrt((end.x - node.x)**2 + (end.y - node.y)**2)
    
    # turns a pair into an astar node
    def get_node(self, location):
        if type(location) is not AStarNode:
            return self.nodes[location[1]][location[0]]
        else:
            return location

    def search(self, s, e):
        start = self.get_node(s)
        end = self.get_node(e)
        discovered = set()
        explored = set()
        current = start
        discovered.add(current)

        while discovered:
            current = min(discovered, key=lambda o: o.h + o.g)

            if current == end:
                path = []
                while current.parent:
                    path.append(current)
                    current = current.parent
                    try:
                        if current == current.parent.parent:
                            break
                    except:
                        pass
                path.append(current)
                return path[::-1]

            discovered.remove(current)
            explored.add(current)
            for node in self.graph[current]:
                if node in explored:
                    continue
                if node in discovered:
                    cost = current.move_cost(node)
                    if cost == 14:
                        continue
                    new_g = current.g + cost
                    if node.g > new_g:
                        node.g = new_g
                        node.parent = current
                else:
                    cost = current.move_cost(node)
                    if cost == 14:
                        continue
                    node.g = current.g + cost
                    node.h = self.heuristic(node, start, end)
                    node.parent = current
                    discovered.add(node)
        return None

