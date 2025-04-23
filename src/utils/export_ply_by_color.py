import open3d as o3d
import numpy as np
from plyfile import PlyData
from ..core.load_point_cloud import load_point_cloud


def export_ply_by_color(input_file_path: str):
    ply = PlyData.read(input_file_path)
    pcd = load_point_cloud(input_file_path)
    # o3d.visualization.draw_geometries([pcd])

    vertices = ply['vertex']
    labels = vertices["scalar_Original_cloud_index"]
    segmented_planes = segment_by_labels(pcd, labels)
    # o3d.visualization.draw_geometries(segmented_planes)

    return segmented_planes


def segment_by_labels(pcd, labels):
    unique_labels = np.unique(labels)
    print(len(unique_labels))
    segmented_planes = []
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    colors = np.asarray(pcd.colors)
    for label in unique_labels:

        mask = labels == label
        segmented_plane = o3d.geometry.PointCloud()
        segmented_plane.points = o3d.utility.Vector3dVector(points[mask])
        segmented_plane.colors = o3d.utility.Vector3dVector(colors[mask])
        segmented_plane.normals = o3d.utility.Vector3dVector(normals[mask])
        segmented_planes.append(segmented_plane)

    return segmented_planes
