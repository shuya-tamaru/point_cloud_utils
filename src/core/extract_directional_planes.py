import numpy as np
import open3d as o3d


def extract_directional_planes_xyz(pcd, normals, use_original_color=False):
    points = np.asarray(pcd.points)
    normals_array = np.asarray(normals)

    abs_normals = np.abs(normals_array)
    dominant_threshold = 0.8

    x_positive = (abs_normals[:, 0] > dominant_threshold) & (
        normals_array[:, 0] > 0)
    x_negative = (abs_normals[:, 0] > dominant_threshold) & (
        normals_array[:, 0] < 0)
    y_positive = (abs_normals[:, 1] > dominant_threshold) & (
        normals_array[:, 1] > 0)
    y_negative = (abs_normals[:, 1] > dominant_threshold) & (
        normals_array[:, 1] < 0)
    z_positive = (abs_normals[:, 2] > dominant_threshold) & (
        normals_array[:, 2] > 0)
    z_negative = (abs_normals[:, 2] > dominant_threshold) & (
        normals_array[:, 2] < 0)

    direction_clouds = {}

    if np.sum(x_positive) > 100 or np.sum(x_negative) > 100:
        x_wall = o3d.geometry.PointCloud()
        x_wall.points = o3d.utility.Vector3dVector(
            points[x_positive | x_negative])
        if use_original_color:
            x_wall.colors = o3d.utility.Vector3dVector(
                np.asarray(pcd.colors)[x_positive | x_negative])
        else:
            x_wall.paint_uniform_color([1, 0.4, 0.4])
        direction_clouds["x"] = x_wall

    if np.sum(y_positive) > 100 or np.sum(y_negative) > 100:
        y_wall = o3d.geometry.PointCloud()
        y_wall.points = o3d.utility.Vector3dVector(
            points[y_positive | y_negative])
        if use_original_color:
            y_wall.colors = o3d.utility.Vector3dVector(
                np.asarray(pcd.colors)[y_positive | y_negative])
        else:
            y_wall.paint_uniform_color([0.4, 1, 0.4])  # 緑
        direction_clouds["y"] = y_wall

    if np.sum(z_positive) > 100 or np.sum(z_negative) > 100:
        z_wall = o3d.geometry.PointCloud()
        z_wall.points = o3d.utility.Vector3dVector(
            points[z_positive | z_negative])
        if use_original_color:
            z_wall.colors = o3d.utility.Vector3dVector(
                np.asarray(pcd.colors)[z_positive | z_negative])
        else:
            z_wall.paint_uniform_color([0.4, 0.4, 1])  # 青
        direction_clouds["z"] = z_wall

    all_masks = x_positive | x_negative | y_positive | y_negative | z_positive | z_negative
    unclassified = ~all_masks

    if np.sum(unclassified) > 100:
        other_points = o3d.geometry.PointCloud()
        other_points.points = o3d.utility.Vector3dVector(points[unclassified])
        if use_original_color:
            other_points.colors = o3d.utility.Vector3dVector(
                np.asarray(pcd.colors)[unclassified])
        else:
            other_points.paint_uniform_color([0.5, 0.5, 0.5])
        direction_clouds["unclassified"] = other_points

    for direction, cloud in direction_clouds.items():
        print(f"{direction}: {len(np.asarray(cloud.points))} points")

    return direction_clouds
