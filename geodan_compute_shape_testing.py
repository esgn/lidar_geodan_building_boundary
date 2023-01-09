import laspy
import sys
import numpy as np
import geopandas as gpd
import building_boundary
from shapely.geometry import Polygon
# import open3d as o3d
import matplotlib.pyplot as plt
import os, shutil
from scipy.spatial import Delaunay
import traceback

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
    # if (i!=31):
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
    found = False
    try:
        shape = building_boundary.shapes.fit.compute_shape(cluster_xy, alpha=0.8, k=None)
        boundary_points = np.array(shape.exterior.coords)
        segments = building_boundary.core.segmentation.boundary_segmentation(boundary_points, 0.3)
        boundary_segments_0 = [building_boundary.core.segment.BoundarySegment(s) for s in segments]
        primary_orientations =  building_boundary.core.regularize.get_primary_orientations(
            boundary_segments_0,
            5,
            angle_epsilon=0.05
        )
        boundary_segments_1 = building_boundary.core.regularize.regularize_segments(boundary_segments_0,
            primary_orientations,
            max_error=0.4)
        boundary_segments_2 = building_boundary.core.merge.merge_segments(boundary_segments_1,
            angle_epsilon=0.05,
            max_distance=0.6,
            max_error=0.4)
        vertices = building_boundary.core.intersect.compute_intersections(boundary_segments_2,
            perp_dist_weight=3)
        found = True
    except:
        traceback.print_exc()

    for s in boundary_segments_0:
        plt.plot(s.points[:, 0], s.points[:,1])
    plt.show()

    for s in boundary_segments_1:
        plt.plot(s.points[:, 0], s.points[:,1])
    plt.show()
  
    for s in boundary_segments_2:
        plt.plot(s.points[:, 0], s.points[:,1])  
    plt.show()

    polygon = Polygon(vertices)
    plt.plot(*polygon.exterior.xy, color="red", linewidth=2)
    plt.show()

    sys.exit(0)
    # In order to visualize cluster points and generated footprint
    # plt.scatter(X, Y)
    # tri = Delaunay(cluster_xy)
    # plt.triplot(X, Y, tri.simplices,linewidth=0.5)
    # if found:
    #     plt.plot(*shape.exterior.xy, color="red", linewidth=2)
    # plt.show()
