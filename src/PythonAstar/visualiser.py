from PIL import Image, ImageDraw, ImageFont
import sys


class ImgGen(object):

    def __init__(self, width, height, nodes):
        self.width = width
        self.height = height
        self.nodes = nodes

    def draw_single(self, path, gen_path, width, height, id, debug):
        img = Image.new('RGB', (self.width, self.height), "white")  # create a new black image

        font = ImageFont.truetype("arial.ttf", 8)
        fill = 0

        draw = ImageDraw.Draw(img)
        """
        for j in range(height):
            draw.line((0, j*self.height/height, self.width, j*self.height/height), fill=fill)
        for i in range(width):
            draw.line((i*self.width/width, 0, i*self.width/width, self.height), fill=fill)

        draw.line((0, self.height-1, self.width-1, self.height-1), fill=fill)
        draw.line((self.width-1, 0, self.width-1, self.height-1), fill=fill)
        """

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

        del draw

        if debug == 0:
            img.save(id+".png")
        elif debug == 1:
            img.show()