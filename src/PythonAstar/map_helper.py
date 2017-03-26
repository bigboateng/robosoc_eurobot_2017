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
        self.moonbases = {}
        self.moonbases["blue"] = {}        
        self.moonbases["yellow"] = {}
        self.moonbases["blue"]["A"] = (90,140)
        self.moonbases["blue"]["B"] = (150,120)
        self.moonbases["yellow"]["A"] = (210,140)
        self.moonbases["yellow"]["B"] = (150,120)
        self.rockettowers = {}
        self.rockettowers["blue"] = {}        
        self.rockettowers["yellow"] = {}
        self.rockettowers["blue"]["A"] = (0,135)
        self.rockettowers["blue"]["B"] = (115,0)
        self.rockettowers["yellow"]["A"] = (135,300)
        self.rockettowers["yellow"]["B"] = (185,0)
        self.smallcraters = {}
        self.smallcraters["blue"] = {}        
        self.smallcraters["yellow"] = {}
        self.smallcraters["blue"]["A"] = (65,54)
        self.smallcraters["blue"]["B"] = (107,187)
        self.smallcraters["yellow"]["A"] = (235,54)
        self.smallcraters["yellow"]["B"] = (193,187)
        self.bigcraters = {}
        self.bigcraters["blue"] = {}        
        self.bigcraters["yellow"] = {}
        self.bigcraters["blue"]["A"] = (0,149)
        self.bigcraters["blue"]["B"] = (36,164)
        self.bigcraters["blue"]["C"] = (51,200)
        self.bigcraters["yellow"]["A"] = (300,149)
        self.bigcraters["yellow"]["B"] = (264,164)
        self.bigcraters["yellow"]["C"] = (249,200)
        self.ramp = {}
        self.ramp["blue"] = {}        
        self.ramp["yellow"] = {}
        self.ramp["blue"]["A"] = (149,0)
        self.ramp["yellow"]["A"] = (149,300)
        self.net = {}
        self.net["blue"] = {}        
        self.net["yellow"] = {}
        self.net["blue"]["A"] = (36,0)
        self.net["yellow"]["A"] = (167,0)
        
        # ... add others ... like in the form above
