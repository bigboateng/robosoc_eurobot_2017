"""
    This class contains information that is going to be pre-loaded before the start of each round. 
"""
class MapHelper(object):

    def __init__(self):
        self.cylinders = {}
        self.cylinders["blue"] = {}        
        self.cylinders["yellow"] = {}
        self.cylinders["blue"]["A"] = (200,600)  # list of all cylinders for blue
        # ... add others ... like in the form above
