from PIL import Image


def split(path, width, height):
    im = Image.open(path)
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
            image_value[int((j*height)/img_height)][int((i*width)/img_width)] += pixels[i, j]/(small_h*small_w)

    return image_value

"""
    image_arr = [[Image.new('RGB', (small_w, small_h), "white") for x in range(width)] for y in range(height)]
    image_value = [[0 for x in range(width)] for y in range(height)]

    for i in range(height):
        for j in range(width):
            box = (j*small_w, i*small_h, (j+1)*small_w, (i+1)*small_h)
            a = im.crop(box)
            # image_arr[i][j] = a
            total = 0
            for x in range(small_w):
                for y in range(small_h):
                    tempr = a[x,y]
                    total += tempr
            image_value[i][j] = a

    return image_value[i][i]
"""

values = split(r"D:\University\Robosoc\Eurobot 2017\SecondaryMap.png", 30, 20)

for y in range(20):
    for x in range(30):
        print(0 if 295-values[y][x] < 50 else 1, end='')
    print()


