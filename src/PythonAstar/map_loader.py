from itertools import product
from astar_node import AStarNode
from astar import AStar
from map import Map
from map_helper import MapHelper
from map_loader import MapLoader

"""
	This class serves to load the map before the start of the competition

	call load_map which returns Map object
"""
class MapLoader(object):

	"""
	file path points to the file where the 0's and 1's representation of the map is stored
	"""
	def __init__(self, file_path = "300by200"):
	self.file_path = file_path

	# call this function to the Map
	def load_map(self):
		print ("opening file: " + self.file_path)
		file = open(self.file_path, “r”)
		gridString = file.read()
		graph, nodes = make_graph(gridString)
		make_walls(graph, nodes)
		return Map(graph, nodes)

	# creates an AStarNodes map of the spefied dimensions
	def make_graph(gridString):
		width = len(gridString.split("\n")[0])
		height = len(gridString.split("\n"))
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

	# creates walls for where the map is not accessable i.e. the places where there are 1's
	def make_walls(graph, nodes):
		width = len(nodes[0])
		height = len(nodes)
		y = 0
		for row in :
			nodes = [[AStarNode(x, y) for y in range(height)] for x in range(width)]
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
								graph[nodes[x][y]] = []
			y += 1

