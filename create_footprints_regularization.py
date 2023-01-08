import laspy
import sys
import numpy as np
import geopandas as gpd
import building_boundary
from shapely.geometry import Polygon
import open3d as o3d
import matplotlib.pyplot as plt
from regularisation.core import Context
from regularisation.strategies import Geodan

import json

# Load cluster.laz and read useful values
laz = laspy.read('cluster.laz')
cluster_min = laz.ClusterID.min()
cluster_max = laz.ClusterID.max()
x_scale_factor = laz.header.x_scale
y_scale_factor = laz.header.y_scale
z_scale_factor = laz.header.z_scale

polygons = []

# Iterate over cluster
for i in range(cluster_min+1,cluster_max+1):
    
    print("dealing with cluster " + str(i))

    cluster = laz.points[laz.ClusterID == i]

    # In order to output cluster data to laz file
    # new_file = laspy.create(point_format=laz.header.point_format, file_version=laz.header.version)
    # new_file.points = cluster
    # new_file.write('cluster_'+str(i)+'_output.laz')

    # In order to visualize cluster data in 3D directly
    # point_data = np.stack([cluster.X, cluster.Y, cluster.Z], axis=0).transpose((1, 0))
    # geom = o3d.geometry.PointCloud()
    # geom.points = o3d.utility.Vector3dVector(point_data)
    # o3d.visualization.draw_geometries([geom])

    X=cluster.X*x_scale_factor
    Y=cluster.Y*y_scale_factor
    cluster_xy = np.column_stack((X,Y))

    # Call to building_boundary
    try:
        vertices = building_boundary.trace_boundary(
            cluster_xy,
            ransac_threshold=0.5,
            max_error=0.4,
            alpha=0.15,
            k=5,
            num_points=10,
            merge_distance=0.6
        )
        polygons.append(Polygon(vertices))
    except Exception as e:
        print(e)

    # In order to visualize cluster points and generated footprint
    # p = Polygon(vertices)
    # plt.plot(*p.exterior.xy)
    # plt.scatter(X, Y)
    # plt.show()

print(polygons)
s = gpd.GeoSeries(polygons,crs="epsg:2154")
s.to_file('output.geojson', driver='GeoJSON')
# gdf = gpd.GeoDataFrame(geometry=polygons, crs="epsg:2154")
# gdf.to_file('output.geojson', driver='GeoJSON')
