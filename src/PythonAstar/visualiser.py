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

from PIL import Image, ImageDraw, ImageFont
import sys


class Visualiser(object):

    def __init__(self, array):
        self.array = array
        self.height = 600
        self.width = 900

    """
        width and height dictate how many squares are drawn
    """
    def draw(self, path, gen_path, width, height):
        path = gen_path
        print("width = ", width, "height = ", height)
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
                    col = color[5]
                else:
                    col = color[4]
                canvas.create_rectangle(i*x_size, j*y_size, (i+1)*x_size, (j+1)*y_size, width=1, fill=col)
                #draw.rectangle((i*x_size, j*y_size, (i+1)*x_size, (j+1)*y_size), width=0, fill=col)
        
        prev = path[0]
        x_ratio = self.width/width
        y_ratio = self.height/height
        for i in range(1, len(path)):
            node = path[i]
            canvas.create_line((prev[0]*x_ratio+0.5*x_ratio, prev[1]*y_ratio+0.5*y_ratio, node[0]*x_ratio+0.5*x_ratio, node[1]*y_ratio+0.5*y_ratio), width=2,fill=color[2])
            prev = node

        mainloop()

"""
my_map, array = MapLoader(file_path = "./30by20").load_map()
for y in range(len(array)):
    for x in range(len(array[0])):
        print(array[y][x]),
    print()
Visualiser(array).draw([(0,1), (1, 0)], [(1,0), (0,1)], 30, 20)
"""