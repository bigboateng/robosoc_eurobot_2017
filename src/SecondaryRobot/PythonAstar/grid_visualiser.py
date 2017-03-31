try:
    # Python 3.x
    from tkinter import *
except ImportError:
    # Python 2.x
    from Tkinter import *

import random
from PIL import Image
import os.path


def splitImage(imagepath, width, height):
	if os.path.exists(imagepath):
		print ("found: " + imagepath)
	im = Image.open(imagepath)
	pixels = im.load()
	img_width = im.size[0]
	img_height = im.size[1]
	small_w = int(img_width/width)
	small_h = int(img_height/height)
	image_arr = [[Image.new('RGB', (small_w, small_h), "white") for x in range(width)] for y in range(height)]
	image_value = [[0.0 for x in range(width)] for y in range(height)]
	x = 0
	y = 0
	for i in range(img_width):
		for j in range(img_height):
			image_value[int(j*height/img_height)][int(i*width/img_width)] += pixels[i, j]/(small_h*small_w)
	return image_value

width = 150
height = 100
size = 6

values = splitImage(r"SecondaryMap2.png", width, height)

canvas = Canvas(width=width*size, height=height*size, bg='white')
canvas.pack(expand=YES, fill=BOTH)                


color = ['red', 'blue', 'green', 'yellow', 'white', 'black']
for j in range(height):
	for i in range(width):
		print(values[j][i]),
		canvas.create_rectangle(i*size, j*size, (i+1)*size, (j+1)*size, width=0, fill=color[4] if 295-values[j][i] < 50 else color[5])
	print("")
mainloop()