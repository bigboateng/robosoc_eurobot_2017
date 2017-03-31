# -*- coding: utf-8 -*-
from map import Map
from math import sqrt

"""
	This class serves to load the map before the start of the competition
	call load_map which returns Map object
"""
class MapLoader(object):

	"""
		file path points to the file where the 0's and 1's representation of the map is stored
	"""
	def __init__(self, file_path = "./300by200"):
		self.file_path = file_path

	def dist(self, x1, y1, x2, y2):
		return sqrt((x1 - x2)**2 + (y1 - y2)**2)

	def add_border(self, array, radius):
		new_array = [[ 0 for x in range(len(array[0]))] for y in range(len(array))]
		w = len(array[0])
		h = len(array)
		for y in range(h):
			for x in range(w):
				if array[y][x] > 0:
					for j in range(-radius,radius):
						for i in range(-radius,radius):
							if not (0 <= x + i < w):
								continue
							if not (0 <= y + j < h):
								continue
							if self.dist(x, y, x + i, y + j) <= radius:
								new_array[y + j][x + i] = 1

				if not (radius <= y < h-radius):
					new_array[y][x] = 1
					continue

				if not (radius <= x < w-radius):
					new_array[y][x] = 1
					continue

		return [[ array[y][x]+new_array[y][x] for x in range(len(array[0]))] for y in range(len(array))]



	# call this function to the Map
	def load_map(self, border=10):
		print ("opening file: " + self.file_path)
		file = open(self.file_path, 'r')
		gridString = file.read()
		str_array = [ list(line) for line in gridString.split("\n") ]
		array = [[ int(str_array[y][x]) for x in range(len(str_array[0]))] for y in range(len(str_array))]
		array = self.add_border(array, border)
		return Map(array), array



