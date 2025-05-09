import numpy as np
import open3d as o3d


def create_directional_cloud(points, mask, pcd, use_original_color, color, min_points):
    if np.sum(mask) > min_points:
        directional_cloud = o3d.geometry.PointCloud()
        directional_cloud.points = o3d.utility.Vector3dVector(points[mask])
        if use_original_color:
            directional_cloud.colors = o3d.utility.Vector3dVector(
                np.asarray(pcd.colors)[mask])
        else:
            directional_cloud.paint_uniform_color(color)
        return directional_cloud
    return None


def extract_directional_planes_xyz(pcd, normals, use_original_color=False, normal_threshold=0.8, min_points=100):
    points = np.asarray(pcd.points)
    normals_array = np.asarray(normals)

    abs_normals = np.abs(normals_array)

    x_positive = (abs_normals[:, 0] > normal_threshold) & (
        normals_array[:, 0] > 0)
    x_negative = (abs_normals[:, 0] > normal_threshold) & (
        normals_array[:, 0] < 0)
    y_positive = (abs_normals[:, 1] > normal_threshold) & (
        normals_array[:, 1] > 0)
    y_negative = (abs_normals[:, 1] > normal_threshold) & (
        normals_array[:, 1] < 0)
    z_positive = (abs_normals[:, 2] > normal_threshold) & (
        normals_array[:, 2] > 0)
    z_negative = (abs_normals[:, 2] > normal_threshold) & (
        normals_array[:, 2] < 0)

    direction_clouds = {}

    x_wall = create_directional_cloud(
        points, x_positive | x_negative, pcd, use_original_color, [1, 0.4, 0.4], min_points)
    if x_wall:
        direction_clouds["x"] = x_wall

    y_wall = create_directional_cloud(
        points, y_positive | y_negative, pcd, use_original_color, [0.4, 1, 0.4], min_points)
    if y_wall:
        direction_clouds["y"] = y_wall

    z_wall = create_directional_cloud(
        points, z_positive | z_negative, pcd, use_original_color, [0.4, 0.4, 1], min_points)
    if z_wall:
        direction_clouds["z"] = z_wall

    all_masks = x_positive | x_negative | y_positive | y_negative | z_positive | z_negative
    unclassified = ~all_masks

    other_points = create_directional_cloud(
        points, unclassified, pcd, use_original_color, [0.5, 0.5, 0.5], min_points)
    if other_points:
        direction_clouds["unclassified"] = other_points

    for direction, cloud in direction_clouds.items():
        print(f"{direction}: {len(np.asarray(cloud.points))} points")

    return direction_clouds
