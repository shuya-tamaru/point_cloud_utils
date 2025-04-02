import numpy as np
import open3d as o3d

from src.core.extract_directional_planes import extract_directional_planes_xyz
from src.core.segment_plane_dbscan import segment_plane_dbscan
from src.core.segment_plane_ransac import (segment_plane_ransac,
                                           segment_plane_ransac_all_points)
from src.utils.generate_distinct_color_by_direction import \
    generate_unique_color
from src.utils.get_normals import get_normals
from src.utils.global_counter import Counter


def segment_planes(pcd):
    all_planes = []
    pcd_with_normal, normals = get_normals(pcd)
    direction_clouds = extract_directional_planes_xyz(pcd_with_normal, normals)
    # planes = extract_planes_all(pcd_with_normal, counter)
    # all_planes.extend(planes)
    for direction, cloud in direction_clouds.items():
        planes = extract_planes(cloud, direction)
        all_planes.extend(planes)
    print(f"Number of planes: {len(all_planes)}")
    return all_planes


def extract_planes(cloud, direction):
    planes = []
    remaining_points = cloud

    min_points = 100
    distance_threshold = 0.02

    iteration = 0
    max_iterations = 30

    while len(np.asarray(remaining_points.points)) > min_points and iteration < max_iterations:

        plane_pcd, plane_points, plane_points_count, is_valid_direction, rmse, inliers = segment_plane_ransac(
            remaining_points,
            distance_threshold=distance_threshold,
            direction=direction
        )

        # color = generate_unique_color(counter.count)
        # plane_pcd.paint_uniform_color(color)
        # planes.append(plane_pcd)
        # counter.next()
        if (plane_points_count >= min_points and is_valid_direction and rmse <= 0.05):
            segmented_dbscan_planes = segment_plane_dbscan(
                plane_points,   min_points=min_points)
            planes.extend(segmented_dbscan_planes)

        remaining_points = remaining_points.select_by_index(inliers, invert=True)
        iteration += 1

    return planes


def extract_planes_all(cloud, counter):
    planes = []
    remaining_points = cloud

    min_points = 100
    distance_threshold = 0.01

    iteration = 0
    max_iterations = 30

    while len(np.asarray(remaining_points.points)) > min_points and iteration < max_iterations:

        plane_pcd, plane_points, plane_points_count, rmse, inliers = segment_plane_ransac_all_points(
            remaining_points,
            distance_threshold=distance_threshold
        )

        # color = generate_unique_color(counter.count)
        # plane_pcd.paint_uniform_color(color)
        # planes.append(plane_pcd)
        # counter.next()
        if (plane_points_count >= min_points and rmse <= 0.05):
            segmented_dbscan_planes = segment_plane_dbscan(
                plane_points,  counter,  min_points=min_points)
            planes.extend(segmented_dbscan_planes)

        remaining_points = remaining_points.select_by_index(inliers, invert=True)
        iteration += 1

    return planes
