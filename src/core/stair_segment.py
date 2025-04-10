from src.utils.get_normals import get_normals
from src.core.extract_directional_planes import extract_directional_planes_xyz
from src.core.segment_plane_dbscan import segment_plane_dbscan
from src.core.assign_unique_color import assign_unique_color
from src.core.segment_planes import extract_planes
import numpy as np
import open3d as o3d


def stair_segment(pcd):
    x_planes = []
    y_planes = []
    z_planes = []
    unclassified_planes = []

    pcd_with_normal, normals = get_normals(pcd)
    direction_clouds = extract_directional_planes_xyz(pcd_with_normal, normals)
    for direction, cloud in direction_clouds.items():
        segmented_dbscan_planes = segment_stair_component(cloud, direction)
        if direction == "x":
            x_planes.extend(segmented_dbscan_planes)
        elif direction == "y":
            y_planes.extend(segmented_dbscan_planes)
        elif direction == "z":
            z_planes.extend(segmented_dbscan_planes)
        else:
            unclassified_planes.extend(segmented_dbscan_planes)

    all_planes = {
        "x": x_planes,
        "y": y_planes,
        "z": z_planes,
        "unclassified": unclassified_planes
    }

    return all_planes


def segment_stair_component(pcd, direction):
    if (direction == "unclassified"):
        min_point = 30
    else:
        min_point = 30

    segmented_dbscan_planes = segment_plane_dbscan(
        plane_points=pcd.points, eps=0.1, min_points=min_point)

    return segmented_dbscan_planes
