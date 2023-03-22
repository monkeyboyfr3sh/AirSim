import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import pandas as pd
import numpy as np

# Read data from CSV file
df = pd.read_csv("colliders.csv")

print(df)

# # Extract the location coordinates
# lat0 = df.iloc[-2]["lat0"]
# lon0 = df.iloc[-1]["lon0"]

# Extract the collider data
collider_data = df.iloc[1:]

# Define the 8 corners of the collider
posX = collider_data["posX"].astype(float)
posY = collider_data["posY"].astype(float)
posZ = collider_data["posZ"].astype(float)
halfSizeX = collider_data["halfSizeX"].astype(float)
halfSizeY = collider_data["halfSizeY"].astype(float)
halfSizeZ = collider_data["halfSizeZ"].astype(float)

x_corners = np.array([halfSizeX, -halfSizeX, -halfSizeX, halfSizeX, halfSizeX, -halfSizeX, -halfSizeX, halfSizeX])
y_corners = np.array([halfSizeY, halfSizeY, -halfSizeY, -halfSizeY, halfSizeY, halfSizeY, -halfSizeY, -halfSizeY])
z_corners = np.array([halfSizeZ, halfSizeZ, halfSizeZ, halfSizeZ, -halfSizeZ, -halfSizeZ, -halfSizeZ, -halfSizeZ])
corners_3d = np.vstack((x_corners, y_corners, z_corners)).T

# Define the polygons of the collider
verts = [corners_3d[0:4], corners_3d[4:], [corners_3d[0], corners_3d[3], corners_3d[4], corners_3d[7]],
         [corners_3d[1], corners_3d[2], corners_3d[5], corners_3d[6]], [corners_3d[0], corners_3d[1], corners_3d[6], corners_3d[7]],
         [corners_3d[3], corners_3d[2], corners_3d[5], corners_3d[4]]]

# Plot the collider
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-50, 50])
ax.set_ylim([-50, 50])
ax.set_zlim([0, 100])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Collider')
ax.view_init(elev=20, azim=-35)
ax.add_collection3d(Poly3DCollection(verts, alpha=0.25, facecolor='blue', edgecolor='black'))
plt.show()
