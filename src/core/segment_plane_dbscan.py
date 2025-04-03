import numpy as np
import open3d as o3d

def segment_plane_dbscan(plane_points, min_points=100):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(plane_points)

    labels = np.array(
        pcd.cluster_dbscan(eps=0.4, min_points=10)
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
            cluster_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

            segmented_dbscan_planes.append(cluster_pcd)

    return segmented_dbscan_planes
