import numpy as np
import open3d as o3d
from .calculate_normal import calculate_normal


def segment_plane_dbscan(plane_points, eps, min_points=100):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(plane_points)

    labels = np.array(
        pcd.cluster_dbscan(eps=eps, min_points=min_points)
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
            cluster_pcd = calculate_normal(cluster_pcd)

            segmented_dbscan_planes.append(cluster_pcd)

    return segmented_dbscan_planes
