# -*- coding: utf-8 -*-
from map import Map

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

	# call this function to the Map
	def load_map(self):
		print ("opening file: " + self.file_path)
		file = open(self.file_path, 'r')
		gridString = file.read()
		str_array = [ list(line) for line in gridString.split("\n")]
		array = [[ int(str_array[y][x]) for x in range(len(str_array[0]))] for y in range(len(str_array))]
		return Map(array), array

