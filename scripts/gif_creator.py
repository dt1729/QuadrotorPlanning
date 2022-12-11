import imageio
import os, re

numbers     = re.compile(r'(\d+)')
def numericalSort(value):
    parts = numbers.split(value)
    parts[1::2] = map(int, parts[1::2])
    return parts

path = '.' # on Mac: right click on a folder, hold down option, and click "copy as pathname"

image_folder = os.fsencode(path)



filenames = []

for file in os.listdir(image_folder):
    filename = os.fsdecode(file)
    if filename.endswith( ('.jpeg', '.png') ):
        filenames.append(filename)

# filenames.sort() # this iteration technique has no built in order, so sort the frames
filenames = [str(i) + ".png" for i in range(58)]
images = list(map(lambda filename: imageio.imread(filename), filenames))

imageio.mimsave(os.path.join('movie.gif'), images, duration = 0.4) # modify duration as needed
