import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import h5py

# Generate some random data for 3D trajectories

data = h5py.File('pick.hdf5', 'r')['data']


# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

idx = 0
for demo in data.values():
    trajectory = np.array(demo['obs/rh_eef_pos'])
    ax.plot(trajectory[:,0], trajectory[:,1], trajectory[:,2])
    idx += 1
    # if idx > 20:
    #     break

# Set the axis labels and title
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_title('Local Right Hand Trajectories')

# Show the plot
plt.show()
