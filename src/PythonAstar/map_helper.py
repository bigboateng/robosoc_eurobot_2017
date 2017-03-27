from math import sin, cos, radians, sqrt

"""
    This class contains information that is going to be pre-loaded before the start of each round. 
"""
class MapHelper():

    """
        Each coordincate:
        (x, y, orintation, distance)
        where x and y are the actual positions
        and orintation is the approach orientation expressed in bearings 
        and distance to the object

    """
    def __init__(self):
        self.objects = {}
        self.objects["cylinder"] = {}
        self.objects["cylinder"]["blue"] = {}      
        self.objects["cylinder"]["blue"]["A"] = (20, 60, 315)  # list of all cylinders for blue, (x,y) with map being in horizontal position and (0,0) being in the upper left
        self.objects["cylinder"]["blue"]["B"] = (50, 110, 270)
        self.objects["cylinder"]["blue"]["C"] = (80, 185, 135)
        self.objects["cylinder"]["blue"]["D"] = (90, 140, 135)
        self.objects["cylinder"]["blue"]["E"] = (100, 60, 315)
        self.objects["cylinder"]["blue"]["F"] = (95, 20, 0)

        self.objects["cylinder"]["yellow"] = {}
        self.objects["cylinder"]["yellow"]["A"] = (280, 60, 45)
        self.objects["cylinder"]["yellow"]["B"] = (250, 110, 90)
        self.objects["cylinder"]["yellow"]["C"] = (220, 185, 225)
        self.objects["cylinder"]["yellow"]["D"] = (210, 140, 225)
        self.objects["cylinder"]["yellow"]["E"] = (200, 60, 45)
        self.objects["cylinder"]["yellow"]["F"] = (205, 20, 0)

        self.objects["rocket"] = {}
        self.objects["rocket"]["blue"] = {} 
        self.objects["rocket"]["blue"]["A"] = (0, 135, 270)
        self.objects["rocket"]["blue"]["B"] = (115, 0, 0)

        self.objects["rocket"]["yellow"] = {}
        self.objects["rocket"]["yellow"]["A"] = (299, 135, 90)
        self.objects["rocket"]["yellow"]["B"] = (185, 0, 0)

        self.objects["smallcrater"] = {}
        self.objects["smallcrater"]["blue"] = {}
        self.objects["smallcrater"]["blue"]["A"] = (65, 54, 270)
        self.objects["smallcrater"]["blue"]["B"] = (107, 187, 120)

        self.objects["smallcrater"]["yellow"] = {}
        self.objects["smallcrater"]["yellow"]["A"] = (235, 54, 90)
        self.objects["smallcrater"]["yellow"]["B"] = (193, 187, 240)

        self.objects["bigcrater"] = {}
        self.objects["bigcrater"]["blue"] = {}
        self.objects["bigcrater"]["blue"]["A"] = (20,153, 225)
        self.objects["bigcrater"]["blue"]["B"] = (36,164, 225)
        self.objects["bigcrater"]["blue"]["C"] = (47,180, 225)

        self.objects["bigcrater"]["yellow"] = {}
        self.objects["bigcrater"]["yellow"]["A"] = (280,153, 155)
        self.objects["bigcrater"]["yellow"]["B"] = (264,164, 155)
        self.objects["bigcrater"]["yellow"]["C"] = (253,180, 155)

        self.objects["ramp"] = {}
        self.objects["ramp"]["blue"] = {}        
        self.objects["ramp"]["yellow"] = {}
        self.objects["ramp"]["blue"]["A"] = (89, 18, 270)
        self.objects["ramp"]["yellow"]["A"] = (211, 18, 270)

        self.objects["net"] = {}
        self.objects["net"]["blue"] = {}        
        self.objects["net"]["yellow"] = {}
        self.objects["net"]["blue"]["A"] = (25, 0, 270)
        self.objects["net"]["yellow"]["A"] = (275, 0, 270)

        self.offset = {}
        self.offset["cylinder"] = (-5,15) # (centre_offset, dist_to_object)
        self.offset["rocket"] = (-5,22)
        self.offset["smallcrater"] = (0, 10)
        self.offset["bigcrater"] = (0, 10)
        self.offset["ramp"] = (0, 0)
        self.offset["net"] = (-18, 0)


    def get(self, typ, color, obj):
        tpl = self.objects[typ][color][obj]
        orgnl_x = tpl[0]
        orgnl_y = tpl[1]
        bearing = tpl[2]

        offset = self.offset[typ]
        centre = offset[0]
        dist = offset[1]

        x = orgnl_x - dist*sin(radians(bearing)) + centre*sin(radians(bearing + 90))
        y = orgnl_y + dist*cos(radians(bearing)) - centre*cos(radians(bearing + 90))
        
        return (x, y, bearing)