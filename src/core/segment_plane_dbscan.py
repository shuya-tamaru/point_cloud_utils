import numpy as np
import open3d as o3d

from src.utils.generate_distinct_color_by_direction import (
    generate_distinct_random_color, generate_unique_color)


def segment_plane_dbscan(plane_points, direction, counter,  min_points=300):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(plane_points)

    labels = np.array(
        pcd.cluster_dbscan(eps=1.0, min_points=100)
    )

    max_label = labels.max()
    plane_points_array = np.asarray(plane_points)

    segmented_dbscan_planes = []

    for label in range(max_label + 1):
        cluster_indices = np.where(labels == label)[0]
        cluster_points = plane_points_array[cluster_indices]

        if len(cluster_points) >= min_points:
            cluster_pcd = o3d.geometry.PointCloud()
            cluster_pcd.points = o3d.utility.Vector3dVector(cluster_points)

            color = generate_unique_color(counter.count)
            cluster_pcd.paint_uniform_color(color)
            segmented_dbscan_planes.append(cluster_pcd)
            counter.next()
            print(counter.count)

    return segmented_dbscan_planes
