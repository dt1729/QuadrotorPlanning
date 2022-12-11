import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import csv


# def cubes(sides):
#     # Creating data points for the sides
#     data = np.ones(sides)
#     # Creating the figure object
#     fig = plt.figure(figsize=(9, 9))
#     # Creating axes object to the plot
#     ax = fig.add_subplot(111 , projection = '3d')
#     # Plotting the figure
#     ax.voxels(data, facecolors="yellow")
#     # Displaying the figure
#     plt.show()
# # Creating the main () function
# def main():
#     # Defining side for the cube
#     sides = np.array([ 1, 1, 1 ])
#     # Calling the cubes () function
#     cubes(sides)
# # Calling the main () function
# if __name__ == "__main__":
#     main ()

x, y, z = np.indices((11, 8, 11))
data = np.loadtxt('paths.txt')

# draw cuboids in the top left and bottom right corners, and a link between them
cube1 = (x < 5) & (x >= 4) & (y < 5) & (y > 1) & (z <= 5) & (z > 0) 
cube2 = (x < 9) & (x >= 8) & (y < 5) & (y > 1) & (z <= 10) & (z > 5) 
voxels = cube1 | cube2 
colors = np.empty(voxels.shape, dtype=object)
colors[cube1] = 'blue'
colors[cube2] = 'green'

# and plot everything
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.voxels(voxels, facecolors=colors, edgecolor='k')
ax.plot(data[:,0],data[:,1],data[:,2],'.-')
plt.show()
