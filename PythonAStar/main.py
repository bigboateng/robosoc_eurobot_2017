from itertools import product
from astar_node import AStarNode
from astar import AStar
from map import Map
from visualiser import ImgGen


def make_graph(width, height):
    nodes = [[AStarNode(x, y) for y in range(height)] for x in range(width)]
    graph = {}
    for x, y in product(range(width), range(height)):
        node = nodes[x][y]
        graph[node] = []
        for i, j in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            if not (0 <= x + i < width):
                continue
            if not (0 <= y + j < height):
                continue
            graph[nodes[x][y]].append(nodes[x+i][y+j])

    return graph, nodes


def make_walls(gridString, graph, nodes, width, height):
    y = 0
    for row in gridString.split("\n"):
        for x in range(len(row)):
            if row[x] == '1':
                nodes[x][y].obstacle = True
                for i, j in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                    if not (0 <= x + i < width):
                        continue
                    if not (0 <= y + j < height):
                        continue
                    if nodes[x+i][y+j] in graph:
                        if nodes[x][y] in graph[nodes[x+i][y+j]]:
                            graph[nodes[x+i][y+j]].remove(nodes[x][y])
        y += 1

gridString = """"""

w = 300
h = 200

graph, nodes = make_graph(w, h)
make_walls(gridString, graph, nodes, w, h)
my_map = Map(graph, nodes, w, h)
path = my_map.get_path_to(nodes[180][70])
img_gen = ImgGen(w*6, h*6, nodes)

if path is None:
    print('No path found')
else:
    """
    print('Path found:')
    for node in path:
        print(node.x, ', ', node.y)
    print('Generalised')
    """
    gen_path = my_map.generalise(path)
    for node in gen_path:
        print(node.x, ', ', node.y)

    img_gen.draw_single(path, gen_path, w, h, 0, 1)


input("")

""" OLD A* algorithm
class AStar(object):
    def __init__(self, graph):
        self.graph = graph

    def heuristic(self, node, start, end):
        raise NotImplementedError

    def search(self, start, end):
        openset = set()
        closedset = set()
        current = start
        openset.add(current)
        while openset:
            current = min(openset, key=lambda o:o.g + o.h)
            if current == end:
                path = []
                while current.parent:
                    path.append(current)
                    current = current.parent
                path.append(current)
                return path[::-1]
            openset.remove(current)
            closedset.add(current)
            for node in self.graph[current]:
                if node in closedset:
                    continue
                if node in openset:
                    new_g = current.g + current.move_cost(node)
                    if node.g > new_g:
                        node.g = new_g
                        node.parent = current
                else:
                    node.g = current.g + current.move_cost(node)
                    node.h = self.heuristic(node, start, end)
                    node.parent = current
                    openset.add(node)
        return None


class AStarNode(object):
    def __init__(self):
        self.g = 0
        self.h = 0
        self.parent = None

    def move_cost(self, other):
        raise NotImplementedError

class AStarGrid(AStar):
    def heuristic(self, node, start, end):
        return sqrt((end.x - node.x)**2 + (end.y - node.y)**2)


class AStarGridNode(AStarNode):
    def __init__(self, x, y):
        self.x, self.y = x, y
        super(AStarGridNode, self).__init__()

    def move_cost(self, other):
        diagonal = abs(self.x - other.x) == 1 and abs(self.y - other.y) == 1
        return 14 if diagonal else 10 # forteen comes from 1.414 = sqrt(2) for diagonals

"""

