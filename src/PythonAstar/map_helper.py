class MapHelper:

    def __init__(self):
        self.cylinders[0] = []  # list of all cylinders for blue
        self.cylinders[1] = []  # list of all cylinders for yellow
        self.cylinders[2] = []  # list of all cylinders for mixed
        self.craters[0] = []  # locations of craters for on blue side
        self.craters[1] = []  # locations of craters for the yellow side
        self.basket[0] = (0, 0)  # position in front of the blue basket
        self.basket[1] = (300, 0)  # position in front of the yellow basket
        self.pri_robot_pos[0] = () # primary robot position blue
        self.sec_robot_pos[0] = ()  # secondary robot position
        self.pri_robot_pos[1] = () # primary robot position yellow
        self.sec_robot_pos[1] = ()  # secondary robot position
