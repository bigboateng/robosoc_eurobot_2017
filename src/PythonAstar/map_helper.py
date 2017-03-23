"""
    This class contains information that is going to be pre-loaded before the start of each round. 
"""
class MapHelper():

    def __init__(self):
        self.cylinders = {}
        self.cylinders["blue"] = {}        
        self.cylinders["yellow"] = {}
        self.cylinders["blue"]["A"] = (20,60)  # list of all cylinders for blue, (x,y) with map being in horizontal position and (0,0) being in the upper left
        self.cylinders["blue"]["B"] = (50,110)
        self.cylinders["blue"]["C"] = (80,185)
        self.cylinders["blue"]["D"] = (90,140)
        self.cylinders["blue"]["E"] = (100,60)
        self.cylinders["blue"]["F"] = (95,20)
        self.cylinders["yellow"]["A"] = (280,60)
        self.cylinders["yellow"]["B"] = (250,110)
        self.cylinders["yellow"]["C"] = (220,185)
        self.cylinders["yellow"]["D"] = (210,140)
        self.cylinders["yellow"]["E"] = (60,200)
        self.cylinders["yellow"]["F"] = (205,20)
        
        # ... add others ... like in the form above
