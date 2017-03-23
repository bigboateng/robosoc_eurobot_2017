"""
    This class contains information that is going to be pre-loaded before the start of each round. 
"""
class MapHelper():

    def __init__(self):
        self.cylinders = {}
        self.cylinders["blue"] = {}        
        self.cylinders["yellow"] = {}
        self.cylinders["blue"]["A"] = (200,600)  # list of all cylinders for blue, (x,y) with map being in horizontal position and (0,0) being in the upper left
        # ... add others ... like in the form above
