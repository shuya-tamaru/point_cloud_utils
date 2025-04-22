import numpy as np
import open3d as o3d
from .calculate_normal import calculate_normal


def segment_plane_dbscan(plane_pcd, eps, min_points=100, use_original_color=False):
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(plane_points)

    # if use_original_color:
    #     pcd.colors = o3d.utility.Vector3dVector(
    #         np.asarray(plane_points.colors)[plane_points])

    labels = np.array(
        plane_pcd.cluster_dbscan(eps=eps, min_points=min_points)
    )

    max_label = labels.max()
    plane_points_array = np.asarray(plane_pcd.points)

    segmented_dbscan_planes = []

    for label in range(max_label + 1):
        cluster_indices = np.where(labels == label)[0]
        cluster_points = plane_points_array[cluster_indices]

        if len(cluster_points) >= min_points:
            cluster_pcd = o3d.geometry.PointCloud()
            cluster_pcd.points = o3d.utility.Vector3dVector(cluster_points)
            cluster_pcd = calculate_normal(cluster_pcd)
            if use_original_color:
                cluster_pcd.colors = o3d.utility.Vector3dVector(
                    np.asarray(plane_pcd.colors)[cluster_indices])

            segmented_dbscan_planes.append(cluster_pcd)

    return segmented_dbscan_planes
