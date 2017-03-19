try:
    # Python 3.x
    from tkinter import *
except ImportError:
    # Python 2.x
    from Tkinter import *

from PIL import Image, ImageDraw, ImageFont
import sys


class ImgGen(object):

    def __init__(self, nodes):
        self.nodes = nodes
        self.height = 600
        self.width = 900

    """
        width and height dictate how many squares are drawn
    """
    def draw(self, path, gen_path, width, height, id, debug):
        x_size = self.width/width
        y_size = self.height/height

        canvas = Canvas(width=self.width, height=self.height, bg='white')
        canvas.pack(expand=YES, fill=BOTH)

        # PIL create an empty image and draw object to draw on
        # memory only, not visible
        # image1 = Image.new("RGB", (self.width, self.height), white)
        #drw = ImageDraw.Draw(image1)

        color = ['red', 'blue', 'green', 'yellow', 'white', 'black']

        for j in range(height):
            for i in range(width):
                if self.nodes[i][j].obstacle:
                    col = color[5]
                else:
                    col = color[4]
                canvas.create_rectangle(i*x_size, j*y_size, (i+1)*x_size, (j+1)*y_size, width=1, fill=col)
                #draw.rectangle((i*x_size, j*y_size, (i+1)*x_size, (j+1)*y_size), width=0, fill=col)
        
        prev = gen_path[0]
        x_ratio = self.width/width
        y_ratio = self.height/height
        for i in range(1, len(gen_path)):
            node = gen_path[i]
            canvas.create_line((prev.x*x_ratio+0.5*x_ratio, prev.y*y_ratio+0.5*y_ratio, node.x*x_ratio+0.5*x_ratio, node.y*y_ratio+0.5*y_ratio), width=2,fill=color[2])
            prev = node

        mainloop()
        """
                # do the Tkinter canvas drawings (visible)
        canvas.create_line([0, center, width, center], fill='green')

        # do the PIL image/draw (in memory) drawings
        draw.line([0, center, width, center], green)

        # PIL image can be saved as .png .jpg .gif or .bmp file (among others)
        filename = "my_drawing.jpg"
        image1.save(filename)

        ""
        for j in range(height):
            draw.line((0, j*self.height/height, self.width, j*self.height/height), fill=fill)
        for i in range(width):
            draw.line((i*self.width/width, 0, i*self.width/width, self.height), fill=fill)

        draw.line((0, self.height-1, self.width-1, self.height-1), fill=fill)
        draw.line((self.width-1, 0, self.width-1, self.height-1), fill=fill)
        ""

        for i in range(len(self.nodes)):
            for j in range(len(self.nodes[0])):
                if self.nodes[i][j].obstacle:
                    draw.text((i*self.width/width, j*self.height/height), text="#", fill=fill, font=font)

        for node in path:
            draw.text((node.x*self.width/width, node.y*self.height/height), text="*", fill=fill, font=font)

        prev = gen_path[0]
        x_ratio = self.width/width
        y_ratio = self.height/height
        for i in range(1, len(gen_path)):
            node = gen_path[i]
            draw.line((prev.x*x_ratio+0.5*x_ratio, prev.y*y_ratio+0.5*y_ratio, node.x*x_ratio+0.5*x_ratio, node.y*y_ratio+0.5*y_ratio), fill=3)
            prev = node

        """
