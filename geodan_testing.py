import laspy
import sys
import numpy as np
import geopandas as gpd
import building_boundary
from shapely.geometry import Polygon
# import open3d as o3d
import matplotlib.pyplot as plt
import os, shutil

# Input can be cluster taking into account Z or not
# input_laz = 'cluster3d.laz'
input_laz = 'cluster2d.laz'
# output_folder = 'cluster3d_laz_files'
output_folder = 'cluster2d_laz_files'
# output_json = 'output3d.geojson'
output_json = 'output2d.geojson'

# Load cluster.laz and read useful values
laz = laspy.read(input_laz)
cluster_min = laz.ClusterID.min()
cluster_max = laz.ClusterID.max()
x_scale_factor = laz.header.x_scale
y_scale_factor = laz.header.y_scale
z_scale_factor = laz.header.z_scale

polygons = {}

# In order to output cluster data to laz file
if os.path.exists(output_folder):
    shutil.rmtree(output_folder)
    os.mkdir(output_folder)
else:
    os.mkdir(output_folder)

clusterid_not_found = []

# Iterate over cluster
for i in range(cluster_min+1,cluster_max+1):
    
    # To check a specific cluster
    # if (i!=139) and (i!=72):
    #     continue

    print("dealing with cluster " + str(i))
    cluster = laz.points[laz.ClusterID == i]

    # In order to output cluster data to laz file
    # new_file = laspy.create(point_format=laz.header.point_format, file_version=laz.header.version)
    # new_file.points = cluster
    # new_file.write(os.path.join(output_folder,'cluster_'+str(i)+'_output.laz'))

    # In order to visualize cluster data in 3D directly
    # point_data = np.stack([cluster.X, cluster.Y, cluster.Z], axis=0).transpose((1, 0))
    # geom = o3d.geometry.PointCloud()
    # geom.points = o3d.utility.Vector3dVector(point_data)
    # o3d.visualization.draw_geometries([geom])

    X=cluster.X*x_scale_factor
    Y=cluster.Y*y_scale_factor
    cluster_xy = np.column_stack((X,Y))

    # Testing compute_shape
    shape = building_boundary.shapes.fit.compute_shape(cluster_xy, alpha=0.5, k=5)

    # In order to visualize cluster points and generated footprint
    plt.scatter(X, Y)
    plt.plot(*shape.exterior.xy)
    plt.show()
