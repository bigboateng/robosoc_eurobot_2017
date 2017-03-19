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

smallGridString = """000000000001000000100000000000
000000000000000000000000000000
000000000000000000000000000000
111111100000000000000001111111
000000000000000000000000000000
000000000000000000000000000000
100000000000000000000000000000
110000000000000000000000000001
110000000000000000000000000001
110000000000000000000000000001
110000000000000000000000000001
100000000000000000000000000001
000000000000001100000000000001
100000000000001100000000000001
000000000110001100011000000000
000000000111001100111000000000
000000000011101101110000000000
000000000001111111100000000000
000000000000111111000000000000
000000000000010010000000000000"""

w = 30
h = 20

graph, nodes = make_graph(w, h)
make_walls(smallGridString, graph, nodes, w, h)
my_map = Map(graph, nodes, w, h)
path = my_map.get_path_to(nodes[18][18])
img_gen = ImgGen(nodes)

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

    img_gen.draw(path, gen_path, w, h, 0, 1)