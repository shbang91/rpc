import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# Define the parametric equations
def func(s):
    x = 0.5 * (1 - np.cos(np.pi * s))
    y = 0.5*(1 - np.cos(np.pi * s))
    z = 2 * (-s**2 + s)
    return x, y, z

# Create a grid of s values
s = np.linspace(0, 1, 1000)

# Calculate the x, y, and z coordinates
x, y, z = func(s)

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the curve
ax.plot(x, y, z)

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Show the plot
plt.show()

