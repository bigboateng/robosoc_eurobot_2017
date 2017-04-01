try:
    # Python 3.x
    from tkinter import *
except ImportError:
    # Python 2.x
    from Tkinter import *

from map import Map
from map_helper import MapHelper
from map_loader import MapLoader
from exception import *
from math import sin, cos, radians

from PIL import Image, ImageDraw, ImageFont
import sys


class Visualiser(object):

    def __init__(self, array):
        self.array = array
        self.height = 600
        self.width = 900

    def to_hex(self, i):
        return "%0.2X" % i

    def to_gray(self, i, n_of_shades):
        return "#" + str(self.to_hex((255*(n_of_shades-i))/n_of_shades))*3
    """
        width and height dictate how many squares are drawn
    """
    def draw(self, path, gen_path, boundaries, width, height):
        #print("width = ", width, "height = ", height)
        x_size = self.width/width
        y_size = self.height/height

        canvas = Canvas(width=self.width, height=self.height, bg='white')
        canvas.pack(expand=YES, fill=BOTH)

        # PIL create an empty image and draw object to draw on
        # memory only, not visible
        # image1 = Image.new("RGB", (self.width, self.height), white)
        # drw = ImageDraw.Draw(image1)

        color = ['red', 'blue', 'green', 'yellow', 'white', 'black']

        for j in range(height):
            for i in range(width):
                if self.array[j][i] > 0:
                    col = self.to_gray(self.array[j][i], max(max(self.array)))
                else:
                    col = "white"
                canvas.create_rectangle(i*x_size, j*y_size, (i+1)*x_size, (j+1)*y_size, width=1, fill=col, outline="black")
                #draw.rectangle((i*x_size, j*y_size, (i+1)*x_size, (j+1)*y_size), width=0, fill=col)
        
        canvas.create_rectangle(boundaries[0]*x_size, boundaries[2]*y_size, boundaries[1]*x_size, boundaries[3]*y_size, width=3, outline= color[1])

        prev = path[0]
        x_ratio = self.width/width
        y_ratio = self.height/height
        for i in range(1, len(path)):
            node = path[i]
            x1 = int(prev[0]*x_ratio+0.5*x_ratio)
            y1 = int(prev[1]*y_ratio+0.5*y_ratio)
            x2 = int(node[0]*x_ratio+0.5*x_ratio)
            y2 = int(node[1]*y_ratio+0.5*y_ratio)
            canvas.create_line(( x1, y1, x2, y2), width=2,fill=color[2])
            prev = node

        prev = gen_path[0]
        x_ratio = self.width/width
        y_ratio = self.height/height
        for i in range(1, len(gen_path)):
            node = gen_path[i]
            x1 = int(prev[0]*x_ratio+0.5*x_ratio)
            y1 = int(prev[1]*y_ratio+0.5*y_ratio)
            x2 = int(node[0]*x_ratio+0.5*x_ratio)
            y2 = int(node[1]*y_ratio+0.5*y_ratio)
            canvas.create_line(( x1, y1, x2, y2), width=2,fill=color[3])
            prev = node

        map_helper = MapHelper()
        for obj in map_helper.objects:
            for col in ["yellow", "blue"]:
                for letter in map_helper.objects[obj][col]:
                    i, j, bearing = map_helper.objects[obj][col][letter]
                    canvas.create_rectangle(i*x_ratio, j*y_ratio, (i+1)*x_ratio, (j+1)*y_ratio, width=8, outline="#FF0000")

                    i, j, bearing = map_helper.get(obj,col,letter)
                    canvas.create_rectangle(i*x_ratio, j*y_ratio, (i+1)*x_ratio, (j+1)*y_ratio, width=4, outline="#AA3344")

                    offset = map_helper.offset[obj]
                    centre = offset[0]
                    dist = offset[1]

                    x = i + dist*sin(radians(bearing)) 
                    y = j - dist*cos(radians(bearing))
                    canvas.create_line(i*x_ratio, j*y_ratio, x*x_ratio, y*y_ratio, width=2, fill="#AA3344")

        mainloop()

"""
my_map, array = MapLoader(file_path = "./30by20").load_map()
for y in range(len(array)):
    for x in range(len(array[0])):
        print(array[y][x]),
    print()
Visualiser(array).draw([(0,1), (1, 0)], [(1,0), (0,1)], 30, 20)
"""
