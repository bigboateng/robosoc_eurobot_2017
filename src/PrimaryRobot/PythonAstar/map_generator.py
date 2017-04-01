from PIL import Image

"""
    This program will generate a sequence of 1's and 0's from image pixels:
    Just specify the image path, the new width and height of the string

    WARNING: this program is buggy when dealing with images smaller than the speficied height and width

    You can pipe the result of this program to file 300by200 like this:

    python3 map_generator.py > 300by200
"""

width = 3000
height = 2000
image_path = "SecondaryMap2.png"

""" ------------------------------------------------------------------ """

def split(path, width, height):
    img = Image.open(path)
    img = img.resize((width, height), Image.ANTIALIAS)
    pixels = img.load()

    image_value = [[0 for x in range(width)] for y in range(height)]

    for i in range(width):
        for j in range(height):
            image_value[j][i] += pixels[i, j][0]

    return image_value

values = split(image_path, width, height)
for y in range(height):
    for x in range(width):
        print (1 if values[y][x] < 180 else 0, end='')
    print("")


