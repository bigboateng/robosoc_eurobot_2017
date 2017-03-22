from map import Map
from map_helper import MapHelper
from map_loader import MapLoader
from visualiser import Visualiser
from exception import *

my_map_helper = MapHelper()

my_map_loader = MapLoader(file_path = "./300by200")
my_map, array = my_map_loader.load_map()

try:
    path, gen_path = my_map.get_path((200,100),(10,10))
    Visualiser(my_map.array).draw(path, gen_path, my_map.width, my_map.height)
except PathNotFoundException:
    print "Path Not Found"
except CoordinatesOutsideAreaException:
    print "Dimensions outside the area"
