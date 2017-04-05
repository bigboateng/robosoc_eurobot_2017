from PythonAstar.map_loader import MapLoader
from PythonAstar.map_helper import MapHelper

my_map_loader = MapLoader(file_path = "./PythonAstar/300by200")
my_map, array = my_map_loader.load_map(0)

position = [202, 31, 270]
end_goal_position = [183, 35, 0]

global_path_instructions = my_map.get_instructions(position, end_goal_position)
print(global_path_instructions)

